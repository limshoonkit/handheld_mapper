#!/usr/bin/env python3
"""
Convert FAST-LIVO2 Log/ output into the directory layout expected by Global-LVBA.

FAST-LIVO2 produces (with interval>0, colmap_output_en:true, pose_output_en:true):
  Log/PCD/1.pcd, 2.pcd, ...           per-frame world-frame point clouds
  Log/PCD/scans_pos.json               px py pz qw qx qy qz  per PCD
  Log/Colmap/images/00001.png, ...     per-frame undistorted images
  Log/Colmap/sparse/0/images.txt       COLMAP T_camera_world (QW QX QY QZ TX TY TZ)
  Log/result/<seq>.txt                 TUM trajectory (ts tx ty tz qx qy qz qw) = T_world_imu

Global-LVBA expects:
  <out>/all_pcd_body/<ts>.pcd           per-frame body-frame point clouds
  <out>/all_pcd_body/lidar_poses.txt    TUM format T_world_imu
  <out>/all_image/<ts>.png              per-frame images
  <out>/all_image/image_poses.txt       TUM format T_world_imu

The PCDs from FAST-LIVO2 are in world frame, so we transform them back to body
frame using the recorded pose: p_body = R^T * (p_world - t).
"""

import argparse
import os
import sys
import shutil
import struct
import numpy as np
from scipy.spatial.transform import Rotation


def parse_evo_trajectory(path):
    """Parse TUM-format trajectory: ts tx ty tz qx qy qz qw -> list of (ts, t, R)."""
    entries = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts = float(parts[0])
            t = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            entries.append((ts, t, R))
    return entries


def parse_scans_pos(path):
    """Parse scans_pos.json: px py pz qw qx qy qz -> list of (t, R)."""
    entries = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 7:
                continue
            t = np.array([float(parts[0]), float(parts[1]), float(parts[2])])
            qw, qx, qy, qz = float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6])
            R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            entries.append((t, R))
    return entries


def parse_colmap_images(path):
    """Parse COLMAP images.txt -> list of (image_name, T_world_imu_t, T_world_imu_R).
    COLMAP stores T_camera_world; we invert to T_world_camera, but LVBA actually
    wants T_world_imu. Since we don't have a direct map, we return the image names
    for timestamp extraction and rely on the EVO trajectory for poses."""
    names = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 10 and parts[-1].endswith('.png'):
                names.append(parts[-1])
                next(f, None)  # skip the POINTS2D line
    return names


def match_scans_to_evo(scan_poses, evo_entries, tol=0.01):
    """Match each scan pose to the closest EVO trajectory entry by position.
    Returns list of (evo_index, timestamp) for each scan."""
    evo_positions = np.array([e[1] for e in evo_entries])
    matches = []
    used = set()
    for scan_t, scan_R in scan_poses:
        dists = np.linalg.norm(evo_positions - scan_t, axis=1)
        idx = int(np.argmin(dists))
        if dists[idx] > tol:
            # fallback: find closest unused
            sorted_idx = np.argsort(dists)
            for si in sorted_idx:
                if si not in used:
                    idx = int(si)
                    break
        used.add(idx)
        matches.append((idx, evo_entries[idx][0]))
    return matches


def read_pcd_binary(path):
    """Read a binary PCD with XYZI fields, return (header_lines, Nx4 float32 array)."""
    with open(path, 'rb') as f:
        header_lines = []
        num_points = 0
        while True:
            line = f.readline().decode('ascii', errors='replace').rstrip('\n').rstrip('\r')
            header_lines.append(line)
            if line.startswith('POINTS'):
                num_points = int(line.split()[1])
            if line.startswith('DATA'):
                break
        data = f.read()

    point_size = 16  # 4 floats x 4 bytes
    points = np.frombuffer(data[:num_points * point_size], dtype=np.float32).reshape(num_points, 4)
    return header_lines, points


def write_pcd_binary(path, header_lines, points):
    """Write a binary PCD file."""
    n = points.shape[0]
    new_header = []
    for line in header_lines:
        if line.startswith('WIDTH'):
            new_header.append(f'WIDTH {n}')
        elif line.startswith('POINTS'):
            new_header.append(f'POINTS {n}')
        else:
            new_header.append(line)
    with open(path, 'wb') as f:
        for line in new_header:
            f.write((line + '\n').encode('ascii'))
        f.write(points.astype(np.float32).tobytes())


def transform_pcd_world_to_body(points, t_wi, R_wi):
    """Transform world-frame XYZ to body(IMU)-frame: p_body = R^T * (p_world - t)."""
    xyz = points[:, :3].astype(np.float64)
    xyz_body = (R_wi.T @ (xyz - t_wi).T).T
    result = points.copy()
    result[:, :3] = xyz_body.astype(np.float32)
    return result


def fmt_ts(ts):
    return f"{ts:.6f}"


def convert(fl2_log_dir, output_dir, seq_name="lvba_run"):
    evo_path = os.path.join(fl2_log_dir, "result", seq_name + ".txt")
    scans_pos_path = os.path.join(fl2_log_dir, "PCD", "scans_pos.json")
    colmap_images_path = os.path.join(fl2_log_dir, "Colmap", "sparse", "0", "images.txt")
    colmap_img_dir = os.path.join(fl2_log_dir, "Colmap", "images")

    if not os.path.isfile(evo_path):
        print(f"ERROR: EVO trajectory not found: {evo_path}")
        sys.exit(1)

    print(f"[1/6] Parsing EVO trajectory: {evo_path}")
    evo = parse_evo_trajectory(evo_path)
    print(f"       {len(evo)} poses")

    # --- PCDs ---
    pcd_out = os.path.join(output_dir, "all_pcd_body")
    os.makedirs(pcd_out, exist_ok=True)

    has_scans = os.path.isfile(scans_pos_path)
    if has_scans:
        print(f"[2/6] Parsing scans_pos: {scans_pos_path}")
        scan_poses = parse_scans_pos(scans_pos_path)
        print(f"       {len(scan_poses)} scan poses")

        print(f"[3/6] Matching scans to EVO timestamps")
        matches = match_scans_to_evo(scan_poses, evo)

        print(f"[4/6] Converting PCDs world->body and writing to {pcd_out}")
        lidar_pose_lines = []
        for i, (evo_idx, ts) in enumerate(matches):
            pcd_src = os.path.join(fl2_log_dir, "PCD", f"{i+1}.pcd")
            if not os.path.isfile(pcd_src):
                continue
            header, pts = read_pcd_binary(pcd_src)
            _, t_wi, R_wi = evo[evo_idx]
            pts_body = transform_pcd_world_to_body(pts, t_wi, R_wi)
            pcd_dst = os.path.join(pcd_out, fmt_ts(ts) + ".pcd")
            write_pcd_binary(pcd_dst, header, pts_body)

            q = Rotation.from_matrix(R_wi).as_quat()  # [qx, qy, qz, qw]
            lidar_pose_lines.append(
                f"{fmt_ts(ts)} {t_wi[0]:.6f} {t_wi[1]:.6f} {t_wi[2]:.6f} "
                f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"
            )

        with open(os.path.join(pcd_out, "lidar_poses.txt"), 'w') as f:
            f.write('\n'.join(lidar_pose_lines) + '\n')
        print(f"       Wrote {len(lidar_pose_lines)} PCDs + lidar_poses.txt")
    else:
        print(f"[2-4/6] No scans_pos.json found, writing lidar_poses.txt from full EVO trajectory")
        lidar_pose_lines = []
        for ts, t_wi, R_wi in evo:
            q = Rotation.from_matrix(R_wi).as_quat()
            lidar_pose_lines.append(
                f"{fmt_ts(ts)} {t_wi[0]:.6f} {t_wi[1]:.6f} {t_wi[2]:.6f} "
                f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"
            )
        with open(os.path.join(pcd_out, "lidar_poses.txt"), 'w') as f:
            f.write('\n'.join(lidar_pose_lines) + '\n')
        print(f"       Wrote lidar_poses.txt ({len(lidar_pose_lines)} entries, no per-frame PCDs)")

    # --- Images ---
    img_out = os.path.join(output_dir, "all_image")
    os.makedirs(img_out, exist_ok=True)

    has_colmap = os.path.isfile(colmap_images_path) and os.path.isdir(colmap_img_dir)
    if has_colmap:
        print(f"[5/6] Parsing COLMAP images.txt: {colmap_images_path}")
        colmap_names = parse_colmap_images(colmap_images_path)
        print(f"       {len(colmap_names)} images")

        # COLMAP images are saved once per VIO frame. Match each to nearest EVO timestamp.
        # Images are sequential (00001.png, 00002.png, ...). The VIO frames are a subset
        # of the LIO updates; we distribute image timestamps evenly across the EVO trajectory
        # or match by index stride.
        n_img = len(colmap_names)
        n_evo = len(evo)

        # Heuristic: VIO runs less frequently than LIO. Spread image indices across EVO.
        if n_img > 0 and n_evo > 0:
            stride = max(1, n_evo // n_img)
            img_pose_lines = []
            for j, name in enumerate(colmap_names):
                evo_idx = min(j * stride, n_evo - 1)
                ts, t_wi, R_wi = evo[evo_idx]

                src = os.path.join(colmap_img_dir, name)
                if os.path.isfile(src):
                    dst = os.path.join(img_out, fmt_ts(ts) + ".png")
                    shutil.copy2(src, dst)

                q = Rotation.from_matrix(R_wi).as_quat()
                img_pose_lines.append(
                    f"{fmt_ts(ts)} {t_wi[0]:.6f} {t_wi[1]:.6f} {t_wi[2]:.6f} "
                    f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"
                )

            with open(os.path.join(img_out, "image_poses.txt"), 'w') as f:
                f.write('\n'.join(img_pose_lines) + '\n')
            print(f"       Wrote {len(img_pose_lines)} images + image_poses.txt")
    else:
        print(f"[5/6] No COLMAP output found, skipping image conversion")

    # --- Copy COLMAP DB if present ---
    colmap_db = os.path.join(fl2_log_dir, "Colmap", "match.db")
    if os.path.isfile(colmap_db):
        colmap_out = os.path.join(output_dir, "colmap")
        os.makedirs(colmap_out, exist_ok=True)
        shutil.copy2(colmap_db, os.path.join(colmap_out, "match.db"))
        print(f"[6/6] Copied COLMAP match.db")
    else:
        print(f"[6/6] No COLMAP match.db found (can run COLMAP separately)")

    print(f"\nDone! LVBA dataset at: {output_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert FAST-LIVO2 output to Global-LVBA format")
    parser.add_argument("fl2_log_dir", help="Path to FAST-LIVO2 Log/ directory")
    parser.add_argument("output_dir", help="Output directory for LVBA dataset")
    parser.add_argument("--seq-name", default="lvba_run", help="EVO sequence name (default: lvba_run)")
    args = parser.parse_args()
    convert(args.fl2_log_dir, args.output_dir, args.seq_name)
