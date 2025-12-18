/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

LIVMapper::LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name, const rclcpp::NodeOptions &options, const std::string &camera_config)
    : node_(std::make_shared<rclcpp::Node>(node_name, options)),
      ext_t_(0, 0, 0),
      ext_r_(M3D::Identity()),
      camera_config_(camera_config)
{
  extrin_t_.assign(3, 0.0);
  extrin_r_.assign(9, 0.0);
  cameraextrin_t_.assign(3, 0.0);
  cameraextrin_r_.assign(9, 0.0);

  p_pre_.reset(new Preprocess());
  p_imu_.reset(new ImuProcess());

  ReadParameters();
  VoxelMapConfig voxel_config;
  loadVoxelConfig(node_, voxel_config);

  visual_sub_map_.reset(new PointCloudXYZIN());
  feats_undistort_.reset(new PointCloudXYZIN());
  feats_down_body_.reset(new PointCloudXYZIN());
  feats_down_world_.reset(new PointCloudXYZIN());
  pcl_w_wait_pub_.reset(new PointCloudXYZIN());
  pcl_wait_pub_.reset(new PointCloudXYZIN());
  pcl_wait_save_.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity_.reset(new PointCloudXYZIN());
  voxel_map_manager_.reset(new VoxelMapManager(voxel_config));
  vio_manager_.reset(new VIOManager());
  root_dir_ = ROOT_DIR;
  InitializeFiles();
  InitializeComponents();
  path_.header.stamp = node_->now();
  path_.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::ReadParameters()
{
  node_->declare_parameter("common.lid_topic", "/livox/lidar");
  node_->declare_parameter("common.imu_topic", "/livox/imu");
  node_->declare_parameter("common.ros_driver_bug_fix", false);
  node_->declare_parameter("common.img_en", 1);
  node_->declare_parameter("common.lidar_en", 1);
  node_->declare_parameter("common.img_topic", "/left_camera/image");

  node_->declare_parameter("vio.normal_en", true);
  node_->declare_parameter("vio.inverse_composition_en", false);
  node_->declare_parameter("vio.max_iterations", 5);
  node_->declare_parameter("vio.img_point_cov", 100.0);
  node_->declare_parameter("vio.raycast_en", false);
  node_->declare_parameter("vio.exposure_estimate_en", true);
  node_->declare_parameter("vio.inv_expo_cov", 0.2);
  node_->declare_parameter("vio.grid_size", 5);
  node_->declare_parameter("vio.patch_pyrimid_level", 3);
  node_->declare_parameter("vio.patch_size", 8);
  node_->declare_parameter("vio.outlier_threshold", 1000.0);

  node_->declare_parameter("time_offset.exposure_time_init", 0.0);
  node_->declare_parameter("time_offset.img_time_offset", 0.0);
  node_->declare_parameter("time_offset.imu_time_offset", 0.0);
  node_->declare_parameter("time_offset.lidar_time_offset", 0.0);
  node_->declare_parameter("uav.imu_rate_odom", false);
  node_->declare_parameter("uav.gravity_align_en", false);

  node_->declare_parameter("evo.seq_name", "01");
  node_->declare_parameter("evo.pose_output_en", false);
  node_->declare_parameter("imu.gyr_cov", 1.0);
  node_->declare_parameter("imu.acc_cov", 1.0);
  node_->declare_parameter("imu.imu_int_frame", 3);
  node_->declare_parameter("imu.gravity_est_en", true);
  node_->declare_parameter("imu.ba_bg_est_en", true);

  node_->declare_parameter("preprocess.blind", 0.01);
  node_->declare_parameter("preprocess.filter_size_surf", 0.5);
  node_->declare_parameter("preprocess.hilti_en", false);
  node_->declare_parameter("preprocess.lidar_type", static_cast<int>(AVIA));
  node_->declare_parameter("preprocess.scan_line", 6);
  node_->declare_parameter("preprocess.point_filter_num", 3);
  node_->declare_parameter("preprocess.feature_extract_enabled", false);

  node_->declare_parameter("pcd_save.interval", -1);
  node_->declare_parameter("pcd_save.pcd_save_en", false);
  node_->declare_parameter("pcd_save.colmap_output_en", false);
  node_->declare_parameter("pcd_save.filter_size_pcd", 0.5);
  node_->declare_parameter("extrin_calib.extrinsic_T", vector<double>());
  node_->declare_parameter("extrin_calib.extrinsic_R", vector<double>());
  node_->declare_parameter("extrin_calib.Pcl", vector<double>());
  node_->declare_parameter("extrin_calib.Rcl", vector<double>());
  node_->declare_parameter("debug.plot_time", -10.0);
  node_->declare_parameter("debug.frame_cnt", 6);

  node_->declare_parameter("publish.blind_rgb_points", 0.01);
  node_->declare_parameter("publish.pub_scan_num", 1);
  node_->declare_parameter("publish.pub_effect_point_en", false);
  node_->declare_parameter("publish.dense_map_en", false);

  lid_topic_ = node_->get_parameter("common.lid_topic").as_string();
  imu_topic_ = node_->get_parameter("common.imu_topic").as_string();
  ros_driver_fix_en_ = node_->get_parameter("common.ros_driver_bug_fix").as_bool();
  img_en_ = node_->get_parameter("common.img_en").as_int();
  lidar_en_ = node_->get_parameter("common.lidar_en").as_int();
  img_topic_ = node_->get_parameter("common.img_topic").as_string();

  normal_en_ = node_->get_parameter("vio.normal_en").as_bool();
  inverse_composition_en_ = node_->get_parameter("vio.inverse_composition_en").as_bool();
  max_iterations_ = node_->get_parameter("vio.max_iterations").as_int();
  img_point_cov_ = node_->get_parameter("vio.img_point_cov").as_double();
  raycast_en_ = node_->get_parameter("vio.raycast_en").as_bool();
  exposure_estimate_en_ = node_->get_parameter("vio.exposure_estimate_en").as_bool();
  inv_expo_cov_ = node_->get_parameter("vio.inv_expo_cov").as_double();
  grid_size_ = node_->get_parameter("vio.grid_size").as_int();
  patch_pyrimid_level_ = node_->get_parameter("vio.patch_pyrimid_level").as_int();
  patch_size_ = node_->get_parameter("vio.patch_size").as_int();
  outlier_threshold_ = node_->get_parameter("vio.outlier_threshold").as_double();

  exposure_time_init_ = node_->get_parameter("time_offset.exposure_time_init").as_double();
  img_time_offset_ = node_->get_parameter("time_offset.img_time_offset").as_double();
  imu_time_offset_ = node_->get_parameter("time_offset.imu_time_offset").as_double();
  lidar_time_offset_ = node_->get_parameter("time_offset.lidar_time_offset").as_double();
  imu_prop_enable_ = node_->get_parameter("uav.imu_rate_odom").as_bool();
  gravity_align_en_ = node_->get_parameter("uav.gravity_align_en").as_bool();

  seq_name_ = node_->get_parameter("evo.seq_name").as_string();
  pose_output_en_ = node_->get_parameter("evo.pose_output_en").as_bool();
  gyr_cov_ = node_->get_parameter("imu.gyr_cov").as_double();
  acc_cov_ = node_->get_parameter("imu.acc_cov").as_double();
  imu_int_frame_ = node_->get_parameter("imu.imu_int_frame").as_int();
  gravity_est_en_ = node_->get_parameter("imu.gravity_est_en").as_bool();
  ba_bg_est_en_ = node_->get_parameter("imu.ba_bg_est_en").as_bool();

  p_pre_->blind_ = node_->get_parameter("preprocess.blind").as_double();
  filter_size_surf_min_ = node_->get_parameter("preprocess.filter_size_surf").as_double();
  hilti_en_ = node_->get_parameter("preprocess.hilti_en").as_bool();
  p_pre_->lidar_type_ = node_->get_parameter("preprocess.lidar_type").as_int();
  p_pre_->n_scans_ = node_->get_parameter("preprocess.scan_line").as_int();
  p_pre_->point_filter_num_ = node_->get_parameter("preprocess.point_filter_num").as_int();
  p_pre_->feature_enabled_ = node_->get_parameter("preprocess.feature_extract_enabled").as_bool();

  pcd_save_interval_ = node_->get_parameter("pcd_save.interval").as_int();
  pcd_save_en_ = node_->get_parameter("pcd_save.pcd_save_en").as_bool();
  colmap_output_en_ = node_->get_parameter("pcd_save.colmap_output_en").as_bool();
  filter_size_pcd_ = node_->get_parameter("pcd_save.filter_size_pcd").as_double();
  extrin_t_ = node_->get_parameter("extrin_calib.extrinsic_T").as_double_array();
  extrin_r_ = node_->get_parameter("extrin_calib.extrinsic_R").as_double_array();
  cameraextrin_t_ = node_->get_parameter("extrin_calib.Pcl").as_double_array();
  cameraextrin_r_ = node_->get_parameter("extrin_calib.Rcl").as_double_array();
  plot_time_ = node_->get_parameter("debug.plot_time").as_double();
  frame_cnt_ = node_->get_parameter("debug.frame_cnt").as_int();

  blind_rgb_points_ = node_->get_parameter("publish.blind_rgb_points").as_double();
  pub_scan_num_ = node_->get_parameter("publish.pub_scan_num").as_int();
  pub_effect_point_en_ = node_->get_parameter("publish.pub_effect_point_en").as_bool();
  dense_map_en = node_->get_parameter("publish.dense_map_en").as_bool();

  p_pre_->blind_sqr_ = p_pre_->blind_ * p_pre_->blind_;
}

void LIVMapper::InitializeComponents()
{
  downSize_filter_surf_.setLeafSize(
      filter_size_surf_min_, filter_size_surf_min_, filter_size_surf_min_);
  ext_t_ << VEC_FROM_ARRAY(extrin_t_);
  ext_r_ << MAT_FROM_ARRAY(extrin_r_);

  voxel_map_manager_->extT_ << VEC_FROM_ARRAY(extrin_t_);
  voxel_map_manager_->extR_ << MAT_FROM_ARRAY(extrin_r_);
  // 载入相机参数
  YAML::Node camera_config = YAML::LoadFile(camera_config_);
  if (!vk::camera_loader::loadFromYaml(camera_config, vio_manager_->cam_))
    throw std::runtime_error("Camera model not correctly specified.");
  // 视觉里程计初始化
  vio_manager_->grid_size_ = grid_size_;
  vio_manager_->patch_size_ = patch_size_;
  vio_manager_->outlier_threshold_ = outlier_threshold_;
  vio_manager_->SetImuToLidarExtrinsic(ext_t_, ext_r_);
  vio_manager_->SetLidarToCameraExtrinsic(cameraextrin_r_, cameraextrin_t_);
  // vio_manager的state_和LIVMapper的state_和state_propagat_是一致的，但是voxel_map的需要手动同步
  vio_manager_->state_ = &state_;
  vio_manager_->state_propagat_ = &state_propagat_;
  vio_manager_->max_iterations_ = max_iterations_;
  vio_manager_->img_point_cov_ = img_point_cov_;
  vio_manager_->normal_en_ = normal_en_;
  vio_manager_->inverse_composition_en_ = inverse_composition_en_;
  vio_manager_->raycast_en_ = raycast_en_;
  vio_manager_->patch_pyrimid_level_ = patch_pyrimid_level_;
  vio_manager_->exposure_estimate_en_ = exposure_estimate_en_;
  vio_manager_->colmap_output_en_ = colmap_output_en_;
  vio_manager_->InitializeVIO();

  p_imu_->SetExtrinsic(ext_t_, ext_r_);
  p_imu_->SetGyrCovScale(V3D(gyr_cov_, gyr_cov_, gyr_cov_));
  p_imu_->SetAccCovScale(V3D(acc_cov_, acc_cov_, acc_cov_));
  p_imu_->SetInvExpoCov(inv_expo_cov_);
  p_imu_->SetGyrBiasCov(V3D(0.0001, 0.0001, 0.0001));
  p_imu_->SetAccBiasCov(V3D(0.0001, 0.0001, 0.0001));
  p_imu_->SetImuInitFrameNum(imu_int_frame_);

  if (!gravity_est_en_)
    p_imu_->DisableGravityEst();
  if (!ba_bg_est_en_)
    p_imu_->DisableBiasEst();
  if (!exposure_estimate_en_)
    p_imu_->DisableExposureEst();

  slam_mode_ = (img_en_ && lidar_en_) ? LIVO : ONLY_LIO;
}

void LIVMapper::InitializeFiles()
{
  if (pcd_save_en_ && colmap_output_en_)
  {
    const std::string folderPath =
        std::string(ROOT_DIR) + "/scripts/colmap_output.sh";

    std::string chmodCommand = "chmod +x " + folderPath;

    int chmodRet = system(chmodCommand.c_str());
    if (chmodRet != 0)
    {
      std::cerr << "Failed to set execute permissions for the script."
                << std::endl;
      return;
    }

    int executionRet = system(folderPath.c_str());
    if (executionRet != 0)
    {
      std::cerr << "Failed to execute the script." << std::endl;
      return;
    }
  }
  if (colmap_output_en_)
    fout_points_.open(
        std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt",
        std::ios::out);
  if (pcd_save_interval_ > 0)
    fout_pcd_pos_.open(std::string(ROOT_DIR) + "Log/PCD/scans_pos.json",
                       std::ios::out);
  fout_pre_.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out_.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::InitializeSubscribersAndPublishers()
{
  // Create QoS profile
  auto qos_small = rclcpp::QoS(rclcpp::KeepLast(10));
  auto qos = rclcpp::QoS(rclcpp::KeepLast(100));
  auto qos_large = rclcpp::QoS(rclcpp::KeepLast(200000));

  // Subscribers
  if (p_pre_->lidar_type_ == AVIA)
  {
    sub_pcl_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lid_topic_, qos_large,
        std::bind(&LIVMapper::LivoxCbk, this, std::placeholders::_1));
  }
  else
  {
    sub_pcl_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic_, qos_large,
        std::bind(&LIVMapper::PointCloud2Cbk, this, std::placeholders::_1));
  }

  sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos_large,
      std::bind(&LIVMapper::ImuCbk, this, std::placeholders::_1));

  sub_img_ = node_->create_subscription<sensor_msgs::msg::Image>(
      img_topic_, qos_large,
      std::bind(&LIVMapper::ImageCbk, this, std::placeholders::_1));

  // Publishers
  pubLaser_cloud_full_res_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", qos);
  pub_sub_visual_map_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_visual_sub_map_before", qos);
  pub_laser_cloud_effect_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", qos);
  pub_laser_cloud_map_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", qos);
  pub_odom_aft_mapped_ =
      node_->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", qos);
  pub_path_ = node_->create_publisher<nav_msgs::msg::Path>("/path", qos);
  pub_laser_cloud_dyn_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj", qos);
  pub_laser_cloud_dyn_rmed_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_removed", qos);
  pub_laser_cloud_dyn_dbg_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_dbg_hist", qos);
  mavros_pose_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", qos);
  pub_imu_prop_odom_ =
      node_->create_publisher<nav_msgs::msg::Odometry>("/LIVO2/imu_propagate", qos_large);

  // Image transport publisher
  pub_image_ = std::make_shared<image_transport::Publisher>(
      image_transport::create_publisher(node_.get(), "/rgb_img"));

  // Timer for IMU propagation
  imu_prop_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(4),
      std::bind(&LIVMapper::ImuPropCallback, this));

  // TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

void LIVMapper::HandleFirstFrame()
{
  if (!first_frame_finished_)
  {
    first_lidar_time_ = Lidar_measures_.last_lio_update_time;
    p_imu_->first_lidar_time_ = first_lidar_time_; // Only for IMU data log
    first_frame_finished_ = true;
    std::cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::GravityAlignment()
{
  if (!p_imu_->imu_need_init_ && !gravity_align_finished_)
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(state_.gravity);
    Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    state_.pos_end = G_R_I0 * state_.pos_end;
    state_.rot_end = G_R_I0 * state_.rot_end;
    state_.vel_end = G_R_I0 * state_.vel_end;
    state_.gravity = G_R_I0 * state_.gravity;
    gravity_align_finished_ = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::ProcessImu()
{
  p_imu_->Process(Lidar_measures_, state_, feats_undistort_);

  if (gravity_align_en_)
    GravityAlignment();

  state_propagat_ = state_;
  voxel_map_manager_->state_ = state_;
  voxel_map_manager_->undistort_size_ = feats_undistort_->size();
}

void LIVMapper::StateEstimationAndMapping()
{
  HandleLIO();
  state_propagat_ = state_;
  if (img_en_)
    HandleVIO();
}

void LIVMapper::HandleVIO()
{
  euler_cur_ = RotMtoEuler(state_.rot_end);
  fout_pre_ << std::setw(20)
            << Lidar_measures_.last_lio_update_time - first_lidar_time_ << " "
            << euler_cur_.transpose() * 57.3 << " "
            << state_.pos_end.transpose() << " " << state_.vel_end.transpose()
            << " " << state_.bias_g.transpose() << " "
            << state_.bias_a.transpose() << " "
            << V3D(state_.inv_expo_time, 0, 0).transpose() << std::endl;

  if (pcl_w_wait_pub_->empty() || (pcl_w_wait_pub_ == nullptr))
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }

  std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub_->points.size()
            << std::endl;

  if (fabs((Lidar_measures_.last_lio_update_time - first_lidar_time_) -
           plot_time_) < (frame_cnt_ / 2 * 0.1))
  {
    vio_manager_->plot_flag_ = true;
  }
  else
  {
    // 实际上只走这个分支
    vio_manager_->plot_flag_ = false;
  }

  vio_manager_->ProcessFrame(Lidar_measures_.measures.back().img, pv_list_,
                             voxel_map_manager_->vm_map_);

  // vio_manager_->ProcessFrame(
  //     Lidar_measures_.measures.back().img, pv_list_,
  //     voxelmap_manager_->voxel_map_, Lidar_measures_.last_lio_update_time -
  //     first_lidar_time_);

  if (imu_prop_enable_)
  {
    ekf_finish_once_ = true;
    latest_ekf_state_ = state_;
    latest_ekf_time_ = Lidar_measures_.last_lio_update_time;
    state_update_flg_ = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++)
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  PublishFrameWorld(vio_manager_);
  PublishImgRGB(vio_manager_);

  euler_cur_ = RotMtoEuler(state_.rot_end);
  fout_out_ << std::setw(20)
            << Lidar_measures_.last_lio_update_time - first_lidar_time_ << " "
            << euler_cur_.transpose() * 57.3 << " "
            << state_.pos_end.transpose() << " " << state_.vel_end.transpose()
            << " " << state_.bias_g.transpose() << " "
            << state_.bias_a.transpose() << " "
            << V3D(state_.inv_expo_time, 0, 0).transpose() << " "
            << feats_undistort_->points.size() << std::endl;
}

void LIVMapper::HandleLIO()
{
  euler_cur_ = RotMtoEuler(state_.rot_end);
  fout_pre_ << setw(20)
            << Lidar_measures_.last_lio_update_time - first_lidar_time_ << " "
            << euler_cur_.transpose() * 57.3 << " "
            << state_.pos_end.transpose() << " " << state_.vel_end.transpose()
            << " " << state_.bias_g.transpose() << " "
            << state_.bias_a.transpose() << " "
            << V3D(state_.inv_expo_time, 0, 0).transpose() << endl;

  if (feats_undistort_->empty() || (feats_undistort_ == nullptr))
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSize_filter_surf_.setInputCloud(feats_undistort_);
  downSize_filter_surf_.filter(*feats_down_body_);

  double t_down = omp_get_wtime();

  feats_down_size_ = feats_down_body_->points.size();
  voxel_map_manager_->feats_down_body_ = feats_down_body_;
  voxel_map_manager_->feats_down_size_ = feats_down_size_;

  if (!lidar_map_inited_)
  {
    // 第一帧，建立VoxelMap
    lidar_map_inited_ = true;
    // voxelmap_manager_->BuildVoxelMap();
    TransformLidar(state_.rot_end, state_.pos_end, feats_down_body_,
                   feats_down_world_);
    voxel_map_manager_->BuildVoxelMapLRU(feats_down_world_);
  }

  double t1 = omp_get_wtime();
  // 位姿估计
  voxel_map_manager_->StateEstimation(state_propagat_);
  state_ = voxel_map_manager_->state_;

  double t2 = omp_get_wtime();

  if (imu_prop_enable_)
  {
    ekf_finish_once_ = true;
    latest_ekf_state_ = state_;
    latest_ekf_time_ = Lidar_measures_.last_lio_update_time;
    state_update_flg_ = true;
  }

  if (pose_output_en_)
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend)
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name_ + ".txt",
                   std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open())
        RCLCPP_ERROR(node_->get_logger(), "open fail");
    }
    else
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name_ + ".txt",
                   std::ios::app);
      if (!evoFile.is_open())
        RCLCPP_ERROR(node_->get_logger(), "open fail");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(state_.rot_end);
    evoFile << std::fixed;
    evoFile << Lidar_measures_.last_lio_update_time << " " << state_.pos_end[0]
            << " " << state_.pos_end[1] << " " << state_.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
            << std::endl;
  }

  euler_cur_ = RotMtoEuler(state_.rot_end);
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(euler_cur_(0), euler_cur_(1), euler_cur_(2));
  geo_quat_ = tf2::toMsg(tf2_quat);
  PublishOdometry();

  double t3 = omp_get_wtime();

  // 更新VoxelMap
  PointCloudXYZIN::Ptr world_lidar(new PointCloudXYZIN());
  TransformLidar(state_.rot_end, state_.pos_end, feats_down_body_, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++)
  {
    voxel_map_manager_->pv_list_[i].point_w << world_lidar->points[i].x,
        world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxel_map_manager_->cross_mat_list_[i];
    M3D var = voxel_map_manager_->body_cov_list_[i];
    var = (state_.rot_end * ext_r_) * var *
              (state_.rot_end * ext_r_).transpose() +
          (-point_crossmat) * state_.cov.block<3, 3>(0, 0) *
              (-point_crossmat).transpose() +
          state_.cov.block<3, 3>(3, 3);
    voxel_map_manager_->pv_list_[i].var = var;
  }
  // voxelmap_manager_->UpdateVoxelMap(voxelmap_manager_->pv_list_);
  voxel_map_manager_->UpdateVoxelMapLRU(voxel_map_manager_->pv_list_);
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  pv_list_ = voxel_map_manager_->pv_list_;

  double t4 = omp_get_wtime();

  if (voxel_map_manager_->config_setting_.map_sliding_en)
  {
    voxel_map_manager_->MapSliding();
  }

  PointCloudXYZIN::Ptr laserCloudFullRes(dense_map_en ? feats_undistort_
                                                      : feats_down_body_);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZIN::Ptr laserCloudWorld(new PointCloudXYZIN(size, 1));

  for (int i = 0; i < size; i++)
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                        &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub_ = *laserCloudWorld;

  if (!img_en_)
    PublishFrameWorld(vio_manager_);
  if (pub_effect_point_en_)
    PublishEffectWorld(voxel_map_manager_->ptpl_list_);
  if (voxel_map_manager_->config_setting_.is_pub_plane_map_)
  {
    // voxelmap_manager_->PubVoxelMap();
    voxel_map_manager_->PubVoxelMapLRU();
  }
  PublishPath();
  PublishMavros();

  frame_num_++;
  aver_time_consu_ =
      aver_time_consu_ * (frame_num_ - 1) / frame_num_ + (t4 - t0) / frame_num_;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) /
  // frame_num; aver_time_map_inre = aver_time_map_inre * (frame_num - 1) /
  // frame_num + (t4 - t3) / frame_num; aver_time_solve = aver_time_solve *
  // (frame_num - 1) / frame_num + (solve_time_) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) /
  // frame_num + solve_const_H_time_ / frame_num; printf("[ mapping time ]: per
  // scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP:
  // %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total:
  //         %0.6f \n", t_prop - t0, t1 - t_prop, match_time, solve_time_, t3 -
  //         t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time,
  //         aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map
  // incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map
  //         incre: %0.6f secs, total: %0.6f secs.\033[0m\n", t2 - t1, t4 - t3,
  //         t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  printf(
      "\033[1;34m+-------------------------------------------------------------"
      "+\033[0m\n");
  printf(
      "\033[1;34m|                         LIO Mapping Time                    "
      "|\033[0m\n");
  printf(
      "\033[1;34m+-------------------------------------------------------------"
      "+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage",
         "Time (secs)");
  printf(
      "\033[1;34m+-------------------------------------------------------------"
      "+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf(
      "\033[1;34m+-------------------------------------------------------------"
      "+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time",
         aver_time_consu_);
  printf(
      "\033[1;34m+-------------------------------------------------------------"
      "+\033[0m\n");

  euler_cur_ = RotMtoEuler(state_.rot_end);
  fout_out_ << std::setw(20)
            << Lidar_measures_.last_lio_update_time - first_lidar_time_ << " "
            << euler_cur_.transpose() * 57.3 << " "
            << state_.pos_end.transpose() << " " << state_.vel_end.transpose()
            << " " << state_.bias_g.transpose() << " "
            << state_.bias_a.transpose() << " "
            << V3D(state_.inv_expo_time, 0, 0).transpose() << " "
            << feats_undistort_->points.size() << std::endl;
}

void LIVMapper::SavePCD()
{
  if (pcd_save_en_ &&
      (pcl_wait_save_->points.size() > 0 ||
       pcl_wait_save_intensity_->points.size() > 0) &&
      pcd_save_interval_ < 0)
  {
    std::string raw_points_dir =
        std::string(ROOT_DIR) + "Log/PCD/all_raw_points.pcd";
    std::string downsampled_points_dir =
        std::string(ROOT_DIR) + "Log/PCD/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en_)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save_);
      voxel_filter.setLeafSize(filter_size_pcd_, filter_size_pcd_,
                               filter_size_pcd_);
      voxel_filter.filter(*downsampled_cloud);

      pcd_writer.writeBinary(raw_points_dir,
                             *pcl_wait_save_); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
                << " with point count: " << pcl_wait_save_->points.size()
                << RESET << std::endl;

      pcd_writer.writeBinary(
          downsampled_points_dir,
          *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: "
                << downsampled_points_dir
                << " with point count after filtering: "
                << downsampled_cloud->points.size() << RESET << std::endl;

      if (colmap_output_en_)
      {
        fout_points_ << "# 3D point list with one line of data per point\n";
        fout_points_ << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i)
        {
          const auto &point = downsampled_cloud->points[i];
          fout_points_ << i << " " << std::fixed << std::setprecision(6)
                       << point.x << " " << point.y << " " << point.z << " "
                       << static_cast<int>(point.r) << " "
                       << static_cast<int>(point.g) << " "
                       << static_cast<int>(point.b) << " " << 0 << std::endl;
        }
      }
    }
    else
    {
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity_);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
                << " with point count: "
                << pcl_wait_save_intensity_->points.size() << RESET
                << std::endl;
    }
  }
}

// 主函数
void LIVMapper::Run()
{
  rclcpp::Rate rate(5000);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
    if (!SyncPackages(Lidar_measures_))
    {
      rate.sleep();
      continue;
    }
    HandleFirstFrame();

    ProcessImu();

    StateEstimationAndMapping();
  }
  SavePCD();
}

void LIVMapper::PropImuOnce(StatesGroup &imu_prop_state, const double dt,
                            V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu_->imu_mean_acc_norm_;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr +
                V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1],
                    imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end +
                           imu_prop_state.vel_end * dt +
                           0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::ImuPropCallback()
{
  if (p_imu_->imu_need_init_ || !new_imu_ || !ekf_finish_once_)
  {
    return;
  }
  mtx_buffer_imu_prop_.lock();
  new_imu_ = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable_ && !prop_imu_buffer_.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg_)
    {
      imu_propagate_ = latest_ekf_state_;
      // drop all useless imu pkg
      while (
          (!prop_imu_buffer_.empty() &&
           rclcpp::Time(prop_imu_buffer_.front().header.stamp).seconds() < latest_ekf_time_))
      {
        prop_imu_buffer_.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer_.size(); i++)
      {
        double t_from_lidar_end_time =
            rclcpp::Time(prop_imu_buffer_[i].header.stamp).seconds() - latest_ekf_time_;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        V3D acc_imu(prop_imu_buffer_[i].linear_acceleration.x,
                    prop_imu_buffer_[i].linear_acceleration.y,
                    prop_imu_buffer_[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer_[i].angular_velocity.x,
                    prop_imu_buffer_[i].angular_velocity.y,
                    prop_imu_buffer_[i].angular_velocity.z);
        PropImuOnce(imu_propagate_, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg_ = false;
    }
    else
    {
      V3D acc_imu(newest_imu_.linear_acceleration.x,
                  newest_imu_.linear_acceleration.y,
                  newest_imu_.linear_acceleration.z);
      V3D omg_imu(newest_imu_.angular_velocity.x,
                  newest_imu_.angular_velocity.y,
                  newest_imu_.angular_velocity.z);
      double t_from_lidar_end_time =
          rclcpp::Time(newest_imu_.header.stamp).seconds() - latest_ekf_time_;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      PropImuOnce(imu_propagate_, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate_.pos_end;
    vel_i = imu_propagate_.vel_end;
    q = Eigen::Quaterniond(imu_propagate_.rot_end);
    imu_prop_odom_.header.frame_id = "world";
    imu_prop_odom_.header.stamp = newest_imu_.header.stamp;
    imu_prop_odom_.pose.pose.position.x = posi.x();
    imu_prop_odom_.pose.pose.position.y = posi.y();
    imu_prop_odom_.pose.pose.position.z = posi.z();
    imu_prop_odom_.pose.pose.orientation.w = q.w();
    imu_prop_odom_.pose.pose.orientation.x = q.x();
    imu_prop_odom_.pose.pose.orientation.y = q.y();
    imu_prop_odom_.pose.pose.orientation.z = q.z();
    imu_prop_odom_.twist.twist.linear.x = vel_i.x();
    imu_prop_odom_.twist.twist.linear.y = vel_i.y();
    imu_prop_odom_.twist.twist.linear.z = vel_i.z();
    pub_imu_prop_odom_->publish(imu_prop_odom_);
  }
  mtx_buffer_imu_prop_.unlock();
}

void LIVMapper::TransformLidar(const Eigen::Matrix3d rot,
                               const Eigen::Vector3d t,
                               const PointCloudXYZIN::Ptr &input_cloud,
                               PointCloudXYZIN::Ptr &trans_cloud)
{
  PointCloudXYZIN().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (ext_r_ * p + ext_t_) + t);
    PointXYZIN pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::PointBodyToWorld(const PointXYZIN &pi, PointXYZIN &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(state_.rot_end * (ext_r_ * p_body + ext_t_) + state_.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T>
void LIVMapper::PointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi,
                                 Eigen::Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(state_.rot_end * (ext_r_ * p_body + ext_t_) + state_.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T>
Eigen::Matrix<T, 3, 1> LIVMapper::PointBodyToWorld(
    const Eigen::Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (state_.rot_end * (ext_r_ * p + ext_t_) + state_.pos_end);
  Eigen::Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointXYZIN const *const pi,
                                    PointXYZIN *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_.rot_end * (ext_r_ * p_body + ext_t_) + state_.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::PointCloud2Cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!lidar_en_)
    return;
  mtx_buffer_.lock();

  double cur_head_time = rclcpp::Time(msg->header.stamp).seconds() + lidar_time_offset_;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar_)
  {
    RCLCPP_ERROR(node_->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer_.clear();
  }
  // RCLCPP_INFO(node_->get_logger(), "get point cloud at time: %.6f", rclcpp::Time(msg->header.stamp).seconds());
  PointCloudXYZIN::Ptr ptr(new PointCloudXYZIN());
  p_pre_->Process(msg, ptr);
  lid_raw_data_buffer_.push_back(ptr);
  lid_header_time_buffer_.push_back(cur_head_time);
  last_timestamp_lidar_ = cur_head_time;

  mtx_buffer_.unlock();
  sig_buffer_.notify_all();
}

void LIVMapper::LivoxCbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in)
{
  if (!lidar_en_)
    return;
  mtx_buffer_.lock();
  livox_ros_driver2::msg::CustomMsg::SharedPtr msg(new livox_ros_driver2::msg::CustomMsg(*msg_in));

  if (abs(last_timestamp_imu_ - toSec(msg->header.stamp)) > 1.0 &&
      !imu_buffer_.empty())
  {
    double timediff_imu_wrt_lidar =
        last_timestamp_imu_ - toSec(msg->header.stamp);
    RCLCPP_INFO(node_->get_logger(), "\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset_ = timediff_imu_wrt_lidar;
  }

  double cur_head_time = toSec(msg->header.stamp);
  RCLCPP_INFO(node_->get_logger(), "Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar_)
  {
    RCLCPP_ERROR(node_->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer_.clear();
  }
  // RCLCPP_INFO(node_->get_logger(), "get point cloud at time: %.6f", toSec(msg->header.stamp));
  PointCloudXYZIN::Ptr ptr(new PointCloudXYZIN());
  p_pre_->Process(msg, ptr);

  if (!ptr || ptr->empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Received an empty point cloud");
    mtx_buffer_.unlock();
    return;
  }

  lid_raw_data_buffer_.push_back(ptr);
  lid_header_time_buffer_.push_back(cur_head_time);
  last_timestamp_lidar_ = cur_head_time;

  mtx_buffer_.unlock();
  sig_buffer_.notify_all();
}

void LIVMapper::ImuCbk(const sensor_msgs::msg::Imu::SharedPtr msg_in)
{
  if (last_timestamp_lidar_ < 0.0)
    return;
  // RCLCPP_INFO(node_->get_logger(), "get imu at time: %.6f", rclcpp::Time(msg_in->header.stamp).seconds());
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  msg->header.stamp =
      rclcpp::Time(rclcpp::Time(msg->header.stamp).seconds() - imu_time_offset_, RCL_ROS_TIME);
  double timestamp = rclcpp::Time(msg->header.stamp).seconds();

  if (fabs(last_timestamp_lidar_ - timestamp) > 0.5 && (!ros_driver_fix_en_))
  {
    RCLCPP_WARN(node_->get_logger(), "IMU and LiDAR not synced! delta time: %lf", last_timestamp_lidar_ - timestamp);
  }

  if (ros_driver_fix_en_)
    timestamp += std::round(last_timestamp_lidar_ - timestamp);
  msg->header.stamp = rclcpp::Time(timestamp, RCL_ROS_TIME);

  mtx_buffer_.lock();

  if (last_timestamp_imu_ > 0.0 && timestamp < last_timestamp_imu_)
  {
    mtx_buffer_.unlock();
    sig_buffer_.notify_all();
    RCLCPP_ERROR(node_->get_logger(), "imu loop back, offset: %lf", last_timestamp_imu_ - timestamp);
    return;
  }

  // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  // {

  //   ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp -
  //   last_timestamp_imu); mtx_buffer.unlock(); sig_buffer.notify_all();
  //   return;
  // }

  last_timestamp_imu_ = timestamp;

  imu_buffer_.push_back(msg);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer_.unlock();
  if (imu_prop_enable_)
  {
    mtx_buffer_imu_prop_.lock();
    if (imu_prop_enable_ && !p_imu_->imu_need_init_)
    {
      prop_imu_buffer_.push_back(*msg);
    }
    newest_imu_ = *msg;
    new_imu_ = true;
    mtx_buffer_imu_prop_.unlock();
  }
  sig_buffer_.notify_all();
}

cv::Mat LIVMapper::GetImageFromMsg(const sensor_msgs::msg::Image::SharedPtr msg_img)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(msg_img, "bgr8")->image;
  return img;
}

// static int i = 0;
void LIVMapper::ImageCbk(const sensor_msgs::msg::Image::SharedPtr msg_in)
{
  if (!img_en_)
    return;
  sensor_msgs::msg::Image::SharedPtr msg = msg_in;
  // if ((abs(toSec(msg->header.stamp) - last_timestamp_img) > 0.2 &&
  // last_timestamp_img > 0) || sync_jump_flag_)
  // {
  //   ROS_WARN("img jumps %.3f\n", toSec(msg->header.stamp) -
  //   last_timestamp_img); sync_jump_flag_ = true; msg->header.stamp =
  //   ros::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en_)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0)
      return;
  }
  // double msg_header_time =  toSec(msg->header.stamp);
  double msg_header_time = toSec(msg->header.stamp) + img_time_offset_;
  if (abs(msg_header_time - last_timestamp_img_) < 0.001)
    return;
  RCLCPP_INFO(node_->get_logger(), "Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar_ < 0)
    return;

  if (msg_header_time < last_timestamp_img_)
  {
    RCLCPP_ERROR(node_->get_logger(), "image loop back");
    return;
  }

  mtx_buffer_.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img_ < 0.02)
  {
    RCLCPP_WARN(node_->get_logger(), "Image need Jumps: %.6f", img_time_correct);
    mtx_buffer_.unlock();
    sig_buffer_.notify_all();
    return;
  }

  cv::Mat img_cur = GetImageFromMsg(msg);
  img_buffer_.push_back(img_cur);
  img_time_buffer_.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img_ = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer_.unlock();
  sig_buffer_.notify_all();
}

bool LIVMapper::SyncPackages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer_.empty() && lidar_en_)
    return false;
  if (img_buffer_.empty() && img_en_)
    return false;
  if (imu_buffer_.empty())
    return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0)
      meas.last_lio_update_time = lid_header_time_buffer_.front();
    if (!lidar_pushed_)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar =
          lid_raw_data_buffer_.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1)
        return false;

      meas.lidar_frame_beg_time =
          lid_header_time_buffer_.front(); // generate lidar_frame_beg_time
      meas.lidar_frame_end_time =
          meas.lidar_frame_beg_time +
          meas.lidar->points.back().curvature /
              double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed_ = true; // flag
    }

    if (last_timestamp_imu_ <
        meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer_.lock();
    while (!imu_buffer_.empty())
    {
      if (toSec(imu_buffer_.front()->header.stamp) >
          meas.lidar_frame_end_time)
        break;
      m.imu.push_back(imu_buffer_.front());
      imu_buffer_.pop_front();
    }
    lid_raw_data_buffer_.pop_front();
    lid_header_time_buffer_.pop_front();
    mtx_buffer_.unlock();
    sig_buffer_.notify_all();

    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed_ = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    // LIVO模式下，LIO和VIO的更新时间相同，LIO在前，VIO紧随其后。
    // 注意：此处整理的是LIO使用的数据
    // 图像开始时间+曝光时间
    double img_capture_time = img_time_buffer_.front() + exposure_time_init_;
    // 存在图像话题，但图像话题时间戳大于激光雷达结束时间此时处理激光雷达话题。LIO更新后，meas.lidar_frame_end_time将被刷新。
    if (meas.last_lio_update_time < 0.0)
    {
      meas.last_lio_update_time = lid_header_time_buffer_.front();
    }

    // 取雷达和IMU最新的时间
    double lid_newest_time =
        lid_header_time_buffer_.back() +
        lid_raw_data_buffer_.back()->points.back().curvature / double(1000);
    double imu_newest_time = toSec(imu_buffer_.back()->header.stamp);

    // 图像数据时间戳过小，丢弃过时数据
    if (img_capture_time < meas.last_lio_update_time + 0.00001)
    {
      img_buffer_.pop_front();
      img_time_buffer_.pop_front();
      RCLCPP_ERROR(node_->get_logger(), "[ Data Cut ] Throw one image frame!");
      return false;
    }

    // 图像的获取时间大于雷达和IMU最新的时间，等待雷达和IMU数据
    if (img_capture_time > lid_newest_time ||
        img_capture_time > imu_newest_time)
    {
      return false;
    }

    struct MeasureGroup m;
    // 处理IMU数据
    m.lio_time = img_capture_time;
    mtx_buffer_.lock();
    while (!imu_buffer_.empty())
    {
      if (toSec(imu_buffer_.front()->header.stamp) > m.lio_time)
        break;

      if (toSec(imu_buffer_.front()->header.stamp) >
          meas.last_lio_update_time)
        m.imu.push_back(imu_buffer_.front());

      imu_buffer_.pop_front();
    }
    mtx_buffer_.unlock();
    sig_buffer_.notify_all();

    // 处理激光雷达数据
    // 上一帧的next移动当前帧的cur，同时清理next
    *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
    PointCloudXYZIN().swap(*meas.pcl_proc_next);

    int lid_frame_num = lid_raw_data_buffer_.size();
    int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
    meas.pcl_proc_cur->reserve(max_size);
    meas.pcl_proc_next->reserve(max_size);

    while (!lid_raw_data_buffer_.empty())
    {
      if (lid_header_time_buffer_.front() > img_capture_time)
        break;
      auto pcl(lid_raw_data_buffer_.front()->points);
      double frame_header_time(lid_header_time_buffer_.front());
      float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;
      // 时间小于图像获取时间的点，放入当前帧
      for (int i = 0; i < pcl.size(); i++)
      {
        auto pt = pcl[i];
        if (pcl[i].curvature < max_offs_time_ms)
        {
          pt.curvature +=
              (frame_header_time - meas.last_lio_update_time) * 1000.0f;
          meas.pcl_proc_cur->points.push_back(pt);
        }
        else
        {
          pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
          meas.pcl_proc_next->points.push_back(pt);
        }
      }
      lid_raw_data_buffer_.pop_front();
      lid_header_time_buffer_.pop_front();
    }

    meas.measures.clear();
    if (img_en_)
    {
      // 注意：此处整理的是VIO使用的数据
      // 只添加图片
      m.img = img_buffer_.front();
      mtx_buffer_.lock();
      img_buffer_.pop_front();
      img_time_buffer_.pop_front();
      mtx_buffer_.unlock();
    }
    sig_buffer_.notify_all();
    meas.measures.push_back(m);
    return true;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  RCLCPP_ERROR(node_->get_logger(), "out sync");
}

void LIVMapper::PublishImgRGB(VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp_;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = node_->now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pub_image_->publish(out_msg.toImageMsg());
}

void LIVMapper::PublishFrameWorld(VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub_->empty())
    return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  if (img_en_)
  {
    static int pub_num = 1;
    *pcl_wait_pub_ += *pcl_w_wait_pub_;
    if (pub_num == pub_scan_num_)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub_->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb_;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub_->points[i].x;
        pointRGB.y = pcl_wait_pub_->points[i].y;
        pointRGB.z = pcl_wait_pub_->points[i].z;

        V3D p_w(pcl_wait_pub_->points[i].x, pcl_wait_pub_->points[i].y,
                pcl_wait_pub_->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w));
        if (pf[2] < 0)
          continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->GetInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo;
          // pointRGB.b = pixel[0] * inv_expo; if (pointRGB.r > 255) pointRGB.r
          // = 255; else if (pointRGB.r < 0) pointRGB.r = 0; if (pointRGB.g >
          // 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0; if
          // (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0)
          // pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points_)
            laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
    else
    {
      pub_num++;
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  if (img_en_)
  {
    // cout << "RGB pointcloud size: " << laserCloudWorldRGB->size() << endl;
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  else
  {
    pcl::toROSMsg(*pcl_w_wait_pub_, laserCloudmsg);
  }
  laserCloudmsg.header.stamp =
      node_->now(); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaser_cloud_full_res_->publish(laserCloudmsg);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en_)
  {
    int size = feats_undistort_->points.size();
    PointCloudXYZIN::Ptr laserCloudWorld(new PointCloudXYZIN(size, 1));
    static int scan_wait_num = 0;

    if (img_en_)
    {
      *pcl_wait_save_ += *laserCloudWorldRGB;
    }
    else
    {
      *pcl_wait_save_intensity_ += *pcl_w_wait_pub_;
    }
    scan_wait_num++;

    if ((pcl_wait_save_->size() > 0 || pcl_wait_save_intensity_->size() > 0) &&
        pcd_save_interval_ > 0 && scan_wait_num >= pcd_save_interval_)
    {
      pcd_index_++;
      string all_points_dir(string(string(ROOT_DIR) + "Log/PCD/") +
                            to_string(pcd_index_) + string(".pcd"));
      pcl::PCDWriter pcd_writer;
      if (pcd_save_en_)
      {
        std::cout << "current scan saved to /PCD/" << all_points_dir << endl;
        if (img_en_)
        {
          pcd_writer.writeBinary(
              all_points_dir,
              *pcl_wait_save_); // pcl::io::savePCDFileASCII(all_points_dir,
                                // *pcl_wait_save);
          PointCloudXYZRGB().swap(*pcl_wait_save_);
        }
        else
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity_);
          PointCloudXYZIN().swap(*pcl_wait_save_intensity_);
        }
        Eigen::Quaterniond q(state_.rot_end);
        fout_pcd_pos_ << state_.pos_end[0] << " " << state_.pos_end[1] << " "
                      << state_.pos_end[2] << " " << q.w() << " " << q.x()
                      << " " << q.y() << " " << q.z() << " " << endl;
        scan_wait_num = 0;
      }
    }
  }
  if (laserCloudWorldRGB->size() > 0)
    PointCloudXYZIN().swap(*pcl_wait_pub_);
  PointCloudXYZIN().swap(*pcl_w_wait_pub_);
}

void LIVMapper::PublishVisualSubMap()
{
  PointCloudXYZIN::Ptr laserCloudFullRes(visual_sub_map_);
  int size = laserCloudFullRes->points.size();
  if (size == 0)
    return;
  PointCloudXYZIN::Ptr sub_pcl_visual_map_pub(new PointCloudXYZIN());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = node_->now();
    laserCloudmsg.header.frame_id = "camera_init";
    pub_sub_visual_map_->publish(laserCloudmsg);
  }
}

void LIVMapper::PublishEffectWorld(const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZIN::Ptr laserCloudWorld(new PointCloudXYZIN(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = node_->now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pub_laser_cloud_effect_->publish(laserCloudFullRes3);
}

template <typename T>
void LIVMapper::SetPosestamp(T &out)
{
  out.position.x = state_.pos_end(0);
  out.position.y = state_.pos_end(1);
  out.position.z = state_.pos_end(2);
  out.orientation.x = geo_quat_.x;
  out.orientation.y = geo_quat_.y;
  out.orientation.z = geo_quat_.z;
  out.orientation.w = geo_quat_.w;
}

void LIVMapper::PublishOdometry()
{
  odom_aft_mapped_.header.frame_id = "camera_init";
  odom_aft_mapped_.child_frame_id = "aft_mapped";
  odom_aft_mapped_.header.stamp = node_->now();
  SetPosestamp(odom_aft_mapped_.pose.pose);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = odom_aft_mapped_.header.stamp;
  transform_stamped.header.frame_id = "camera_init";
  transform_stamped.child_frame_id = "aft_mapped";
  transform_stamped.transform.translation.x = state_.pos_end(0);
  transform_stamped.transform.translation.y = state_.pos_end(1);
  transform_stamped.transform.translation.z = state_.pos_end(2);
  transform_stamped.transform.rotation = geo_quat_;

  tf_broadcaster_->sendTransform(transform_stamped);
  pub_odom_aft_mapped_->publish(odom_aft_mapped_);
}

void LIVMapper::PublishMavros()
{
  msg_body_pose_.header.stamp = node_->now();
  msg_body_pose_.header.frame_id = "camera_init";
  SetPosestamp(msg_body_pose_.pose);
  mavros_pose_publisher_->publish(msg_body_pose_);
}

void LIVMapper::PublishPath()
{
  SetPosestamp(msg_body_pose_.pose);
  msg_body_pose_.header.stamp = node_->now();
  msg_body_pose_.header.frame_id = "camera_init";
  path_.poses.push_back(msg_body_pose_);
  pub_path_->publish(path_);
}
