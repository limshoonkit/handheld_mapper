/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml_loader.h>

#include "IMU_Processing.h"
#include "preprocess.h"
#include "vio.h"

class LIVMapper {
 public:
  LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name, const rclcpp::NodeOptions & options);
  ~LIVMapper();
  void InitializeSubscribersAndPublishers();
  void InitializeComponents();
  void InitializeFiles();
  void Run();
  void GravityAlignment();
  void HandleFirstFrame();
  void StateEstimationAndMapping();
  void HandleVIO();
  void HandleLIO();
  void SavePCD();
  void ProcessImu();

  bool SyncPackages(LidarMeasureGroup &meas);
  void PropImuOnce(StatesGroup &imu_prop_state, const double dt, V3D acc_avr,
                   V3D angvel_avr);
  void ImuPropCallback();
  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t,
                      const PointCloudXYZIN::Ptr &input_cloud,
                      PointCloudXYZIN::Ptr &trans_cloud);
  void PointBodyToWorld(const PointXYZIN &pi, PointXYZIN &po);

  void RGBpointBodyToWorld(PointXYZIN const *const pi, PointXYZIN *const po);
  void PointCloud2Cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void LivoxCbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in);
  void ImuCbk(const sensor_msgs::msg::Imu::SharedPtr msg_in);
  void ImageCbk(const sensor_msgs::msg::Image::SharedPtr msg_in);
  void PublishImgRGB(VIOManagerPtr vio_manager);
  void PublishFrameWorld(VIOManagerPtr vio_manager);
  void PublishVisualSubMap();
  void PublishEffectWorld(const std::vector<PointToPlane> &ptpl_list);
  void PublishOdometry();
  void PublishMavros();
  void PublishPath();
  void ReadParameters();
  template <typename T>
  void SetPosestamp(T &out);
  template <typename T>
  void PointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi,
                        Eigen::Matrix<T, 3, 1> &po);
  template <typename T>
  Eigen::Matrix<T, 3, 1> PointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat GetImageFromMsg(const sensor_msgs::msg::Image::SharedPtr msg_img);

  // ROS2 node
  rclcpp::Node::SharedPtr node_;

  std::mutex mtx_buffer_, mtx_buffer_imu_prop_;
  std::condition_variable sig_buffer_;

  SLAM_MODE slam_mode_;

  string root_dir_;
  std::string camera_config_;
  string lid_topic_, imu_topic_, seq_name_, img_topic_;
  V3D ext_t_;
  M3D ext_r_;

  int feats_down_size_ = 0, max_iterations_ = 0;

  double res_mean_last_ = 0.05;
  double gyr_cov_ = 0, acc_cov_ = 0, inv_expo_cov_ = 0;
  double blind_rgb_points_ = 0.0;
  double last_timestamp_lidar_ = -1.0, last_timestamp_imu_ = -1.0,
         last_timestamp_img_ = -1.0;
  double filter_size_surf_min_ = 0;
  double filter_size_pcd_ = 0;
  double first_lidar_time_ = 0.0;
  double match_time_ = 0, solve_time_ = 0, solve_const_H_time_ = 0;

  bool lidar_map_inited_ = false, pcd_save_en_ = false,
       pub_effect_point_en_ = false, pose_output_en_ = false,
       ros_driver_fix_en_ = false, hilti_en_ = false;
  int pcd_save_interval_ = -1, pcd_index_ = 0;
  int pub_scan_num_ = 1;

  StatesGroup imu_propagate_, latest_ekf_state_;

  bool new_imu_ = false, state_update_flg_ = false, imu_prop_enable_ = true,
       ekf_finish_once_ = false;
  deque<sensor_msgs::msg::Imu> prop_imu_buffer_;
  sensor_msgs::msg::Imu newest_imu_;
  double latest_ekf_time_;
  nav_msgs::msg::Odometry imu_prop_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_prop_odom_;
  double imu_time_offset_ = 0.0;
  double lidar_time_offset_ = 0.0;

  bool gravity_align_en_ = false, gravity_align_finished_ = false;

  bool sync_jump_flag_ = false;

  bool lidar_pushed_ = false, gravity_est_en_, flg_reset_ = false,
       ba_bg_est_en_ = true;
  bool dense_map_en = false;
  int img_en_ = 1, imu_int_frame_ = 3;
  bool normal_en_ = true;
  bool exposure_estimate_en_ = false;
  double exposure_time_init_ = 0.0;
  bool inverse_composition_en_ = false;
  bool raycast_en_ = false;
  int lidar_en_ = 1;
  bool first_frame_finished_ = false;
  int grid_size_, patch_size_, patch_pyrimid_level_;
  double outlier_threshold_;
  double plot_time_;
  int frame_cnt_;
  double img_time_offset_ = 0.0;
  deque<PointCloudXYZIN::Ptr> lid_raw_data_buffer_;
  deque<double> lid_header_time_buffer_;
  deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
  deque<cv::Mat> img_buffer_;
  deque<double> img_time_buffer_;
  vector<pointWithVar> pv_list_;
  vector<double> extrin_t_;
  vector<double> extrin_r_;
  vector<double> cameraextrin_t_;
  vector<double> cameraextrin_r_;
  double img_point_cov_;

  PointCloudXYZIN::Ptr visual_sub_map_;
  PointCloudXYZIN::Ptr feats_undistort_;
  PointCloudXYZIN::Ptr feats_down_body_;
  PointCloudXYZIN::Ptr feats_down_world_;
  PointCloudXYZIN::Ptr pcl_w_wait_pub_;
  PointCloudXYZIN::Ptr pcl_wait_pub_;
  PointCloudXYZRGB::Ptr pcl_wait_save_;
  PointCloudXYZIN::Ptr pcl_wait_save_intensity_;

  ofstream fout_pre_, fout_out_, fout_pcd_pos_, fout_points_;

  pcl::VoxelGrid<PointXYZIN> downSize_filter_surf_;

  V3D euler_cur_;

  LidarMeasureGroup Lidar_measures_;
  StatesGroup state_;
  StatesGroup state_propagat_;

  nav_msgs::msg::Path path_;
  nav_msgs::msg::Odometry odom_aft_mapped_;
  geometry_msgs::msg::Quaternion geo_quat_;
  geometry_msgs::msg::PoseStamped msg_body_pose_;

  PreprocessPtr p_pre_;
  ImuProcessPtr p_imu_;
  VoxelMapManagerPtr voxel_map_manager_;
  VIOManagerPtr vio_manager_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
  rclcpp::SubscriptionBase::SharedPtr sub_pcl_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaser_cloud_full_res_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_normal_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sub_visual_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_effect_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_map_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_aft_mapped_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_dyn_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_dyn_rmed_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_dyn_dbg_;
  std::shared_ptr<image_transport::Publisher> pub_image_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pose_publisher_;
  rclcpp::TimerBase::SharedPtr imu_prop_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  int frame_num_ = 0;
  double aver_time_consu_ = 0;
  double aver_time_icp_ = 0;
  double aver_time_map_inre_ = 0;
  bool colmap_output_en_ = false;
};
#endif