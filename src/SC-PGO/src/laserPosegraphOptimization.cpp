#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <iomanip>
#include <sstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#ifdef HAS_LIVOX_ROS_DRIVER2
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "sc_pgo/common.h"
#include "sc_pgo/tic_toc.h"
#include "Scancontext.h"

using namespace gtsam;

using std::cout;
using std::endl;

class SCPGONode : public rclcpp::Node
{
private:
    // Submap accumulation parameters
    double submapDistanceThreshold_;
    int submapScanThreshold_;
    double submapAccumulatedDistance_ = 0.0;
    int submapScanCount_ = 0;
    Pose6D submapStartPose_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<pcl::PointCloud<PointType>::Ptr> submapScanBuffer_;
    std::vector<Pose6D> submapPoseBuffer_;
    bool submapInitialized_ = false;

    Pose6D odom_pose_prev_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Pose6D odom_pose_curr_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Buffers
    std::queue<nav_msgs::msg::Odometry::SharedPtr> odometryBuf_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> fullResBuf_;
    std::queue<sensor_msgs::msg::NavSatFix::SharedPtr> gpsBuf_;
    std::queue<std::pair<int, int>> scLoopICPBuf_;

    std::mutex mBuf_;
    std::mutex mKF_;

    double timeLaserOdometry_ = 0.0;
    double timeLaser_ = 0.0;

    pcl::PointCloud<PointType>::Ptr laserCloudFullRes_;
    pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO_;

    std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds_;
    std::vector<Pose6D> keyframePoses_;
    std::vector<Pose6D> keyframePosesUpdated_;
    std::vector<double> keyframeTimes_;
    int recentIdxUpdated_ = 0;

    gtsam::NonlinearFactorGraph gtSAMgraph_;
    bool gtSAMgraphMade_ = false;
    gtsam::Values initialEstimate_;
    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::Values isamCurrentEstimate_;

    noiseModel::Diagonal::shared_ptr priorNoise_;
    noiseModel::Diagonal::shared_ptr odomNoise_;
    noiseModel::Base::shared_ptr robustLoopNoise_;
    noiseModel::Base::shared_ptr robustGPSNoise_;

    pcl::VoxelGrid<PointType> downSizeFilterScancontext_;
    SCManager scManager_;
    double scDistThres_, scMaximumRadius_;

    pcl::VoxelGrid<PointType> downSizeFilterICP_;
    std::mutex mtxICP_;
    std::mutex mtxPosegraph_;
    std::mutex mtxRecentPose_;

    pcl::PointCloud<PointType>::Ptr laserCloudMapPGO_;
    pcl::VoxelGrid<PointType> downSizeFilterMapPGO_;
    bool laserCloudMapPGORedraw_ = true;

    bool useGPS_ = false;
    sensor_msgs::msg::NavSatFix::SharedPtr currGPS_;
    bool hasGPSforThisKF_ = false;
    bool gpsOffsetInitialized_ = false;
    double gpsAltitudeInitOffset_ = 0.0;
    double recentOptimizedX_ = 0.0;
    double recentOptimizedY_ = 0.0;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS_;

    // TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Threads
    std::thread posegraph_slam_thread_;
    std::thread lc_detection_thread_;
    std::thread icp_calculation_thread_;
    std::thread isam_update_thread_;
    std::thread viz_map_thread_;
    std::thread viz_path_thread_;

    // Save directories
    std::string save_directory_;
    std::string pgScansDirectory_;
    std::string pgTUMformat_, odomTUMformat_;
    std::fstream pgTimeSaveStream_;

public:
    SCPGONode() : Node("sc_pgo_node")
    {
        // Initialize point clouds
        laserCloudFullRes_.reset(new pcl::PointCloud<PointType>());
        laserCloudMapAfterPGO_.reset(new pcl::PointCloud<PointType>());
        laserCloudMapPGO_.reset(new pcl::PointCloud<PointType>());

        // Declare parameters
        this->declare_parameter<std::string>("save_directory", "/tmp/sc_pgo/");
        this->declare_parameter<double>("submap_distance_threshold", 5.0);
        this->declare_parameter<int>("submap_scan_threshold", 10);
        this->declare_parameter<double>("sc_dist_thres", 0.2);
        this->declare_parameter<double>("sc_max_radius", 80.0);
        this->declare_parameter<double>("mapviz_filter_size", 0.4);
        this->declare_parameter<bool>("use_gps", false);

        // Get parameters
        this->get_parameter("save_directory", save_directory_);
        this->get_parameter("submap_distance_threshold", submapDistanceThreshold_);
        this->get_parameter("submap_scan_threshold", submapScanThreshold_);
        this->get_parameter("sc_dist_thres", scDistThres_);
        this->get_parameter("sc_max_radius", scMaximumRadius_);
        this->get_parameter("use_gps", useGPS_);

        RCLCPP_INFO(this->get_logger(), "Submap accumulation: distance threshold = %.2f m, scan threshold = %d scans",
                    submapDistanceThreshold_, submapScanThreshold_);

        // Setup save directories
        pgTUMformat_ = save_directory_ + "optimized_poses_tum.txt";
        odomTUMformat_ = save_directory_ + "odom_poses_tum.txt";
        pgTimeSaveStream_ = std::fstream(save_directory_ + "times.txt", std::fstream::out);
        pgTimeSaveStream_.precision(std::numeric_limits<double>::max_digits10);
        pgScansDirectory_ = save_directory_ + "Scans/";
        auto unused = system((std::string("exec rm -r ") + pgScansDirectory_).c_str());
        unused = system((std::string("mkdir -p ") + pgScansDirectory_).c_str());
        (void)unused;

        // Initialize GTSAM
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam_ = std::make_unique<ISAM2>(parameters);
        initNoises();

        // Setup Scan Context
        scManager_.setSCdistThres(scDistThres_);
        scManager_.setMaximumRadius(scMaximumRadius_);

        // Setup filters
        float filter_size = 0.4;
        downSizeFilterScancontext_.setLeafSize(filter_size, filter_size, filter_size);
        downSizeFilterICP_.setLeafSize(filter_size, filter_size, filter_size);

        double mapVizFilterSize;
        this->get_parameter("mapviz_filter_size", mapVizFilterSize);
        downSizeFilterMapPGO_.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

        // Setup subscribers
        subLaserCloudFullRes_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sc_pgo_cloud_registered", 10,
            std::bind(&SCPGONode::laserCloudFullResHandler, this, std::placeholders::_1));

        subLaserOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/sc_pgo_odometry", 10,
            std::bind(&SCPGONode::laserOdometryHandler, this, std::placeholders::_1));

        if (useGPS_) {
            subGPS_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/gps/fix", 10,
                std::bind(&SCPGONode::gpsHandler, this, std::placeholders::_1));
        }

        // Setup publishers
        pubOdomAftPGO_ = this->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", 10);
        pubPathAftPGO_ = this->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", 10);
        pubMapAftPGO_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_map", 10);
        pubLoopScanLocal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", 10);
        pubLoopSubmapLocal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Start threads
        posegraph_slam_thread_ = std::thread(&SCPGONode::process_pg, this);
        lc_detection_thread_ = std::thread(&SCPGONode::process_lcd, this);
        icp_calculation_thread_ = std::thread(&SCPGONode::process_icp, this);
        isam_update_thread_ = std::thread(&SCPGONode::process_isam, this);
        viz_map_thread_ = std::thread(&SCPGONode::process_viz_map, this);
        viz_path_thread_ = std::thread(&SCPGONode::process_viz_path, this);

        RCLCPP_INFO(this->get_logger(), "SC-PGO node initialized");
    }

    ~SCPGONode()
    {
        if (posegraph_slam_thread_.joinable()) posegraph_slam_thread_.join();
        if (lc_detection_thread_.joinable()) lc_detection_thread_.join();
        if (icp_calculation_thread_.joinable()) icp_calculation_thread_.join();
        if (isam_update_thread_.joinable()) isam_update_thread_.join();
        if (viz_map_thread_.joinable()) viz_map_thread_.join();
        if (viz_path_thread_.joinable()) viz_path_thread_.join();

        if (pgTimeSaveStream_.is_open()) {
            pgTimeSaveStream_.close();
        }
    }

private:
    std::string padZeros(int val, int num_digits = 6) {
        std::ostringstream out;
        out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
        return out.str();
    }

    gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p) {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
    }

    void saveOdometryVerticesTUMformat(std::string _filename) {
        std::fstream stream(_filename.c_str(), std::fstream::out);
        stream.precision(std::numeric_limits<double>::max_digits10);

        for(size_t i = 0; i < keyframePoses_.size(); i++) {
            gtsam::Pose3 pose = Pose6DtoGTSAMPose3(keyframePoses_[i]);
            Point3 t = pose.translation();
            Rot3 R = pose.rotation();
            auto quat = R.toQuaternion();  // Returns Quaternion in GTSAM (w, x, y, z)

            // TUM format: timestamp tx ty tz qx qy qz qw
            double timestamp = keyframeTimes_[i];
            stream << timestamp << " "
                   << t.x() << " " << t.y() << " " << t.z() << " "
                   << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
        }
    }

    void saveOptimizedVerticesTUMformat(gtsam::Values _estimates, std::string _filename) {
        std::fstream stream(_filename.c_str(), std::fstream::out);
        stream.precision(std::numeric_limits<double>::max_digits10);

        int idx = 0;
        for(const auto& key_value: _estimates) {
            auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
            if (!p) continue;

            const Pose3& pose = p->value();
            Point3 t = pose.translation();
            Rot3 R = pose.rotation();
            auto quat = R.toQuaternion();  // Returns Quaternion in GTSAM (w, x, y, z)

            // TUM format: timestamp tx ty tz qx qy qz qw
            double timestamp = keyframeTimes_[idx];
            stream << timestamp << " "
                   << t.x() << " " << t.y() << " " << t.z() << " "
                   << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
            idx++;
        }
    }

    void laserOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr _laserOdometry) {
        mBuf_.lock();
        odometryBuf_.push(_laserOdometry);
        mBuf_.unlock();
    }

    void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::SharedPtr _laserCloudFullRes) {
        mBuf_.lock();
        fullResBuf_.push(_laserCloudFullRes);
        mBuf_.unlock();
    }

    void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr _gps) {
        if(useGPS_) {
            mBuf_.lock();
            gpsBuf_.push(_gps);
            mBuf_.unlock();
        }
    }

    void initNoises() {
        gtsam::Vector priorNoiseVector6(6);
        priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
        priorNoise_ = noiseModel::Diagonal::Variances(priorNoiseVector6);

        gtsam::Vector odomNoiseVector6(6);
        odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
        odomNoise_ = noiseModel::Diagonal::Variances(odomNoiseVector6);

        double loopNoiseScore = 0.5;
        gtsam::Vector robustNoiseVector6(6);
        robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
        robustLoopNoise_ = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
                        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

        double bigNoiseTolerentToXY = 1000000000.0;
        double gpsAltitudeNoiseScore = 250.0;
        gtsam::Vector robustNoiseVector3(3);
        robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore;
        robustGPSNoise_ = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
                        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3));
    }

    Pose6D getOdom(const nav_msgs::msg::Odometry::SharedPtr _odom) {
        auto tx = _odom->pose.pose.position.x;
        auto ty = _odom->pose.pose.position.y;
        auto tz = _odom->pose.pose.position.z;

        double roll, pitch, yaw;
        tf2::Quaternion q(
            _odom->pose.pose.orientation.x,
            _odom->pose.pose.orientation.y,
            _odom->pose.pose.orientation.z,
            _odom->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        return Pose6D{tx, ty, tz, roll, pitch, yaw};
    }

    Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2) {
        Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
        Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
        Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
        Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
        float dx, dy, dz, droll, dpitch, dyaw;
        pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);

        return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
    }

    pcl::PointCloud<PointType>::Ptr accumulateSubmap() {
        pcl::PointCloud<PointType>::Ptr submapAccumulated(new pcl::PointCloud<PointType>());

        // Use the first pose in the buffer as reference frame
        if (submapPoseBuffer_.empty()) {
            return submapAccumulated;
        }

        Pose6D referencePose = submapPoseBuffer_[0];

        // Transform all scans to the reference frame and accumulate
        for (size_t i = 0; i < submapScanBuffer_.size(); i++) {
            pcl::PointCloud<PointType>::Ptr transformedCloud = local2global(submapScanBuffer_[i], submapPoseBuffer_[i]);

            // Transform to reference frame
            Eigen::Affine3f refTransform = pcl::getTransformation(referencePose.x, referencePose.y, referencePose.z,
                                                                   referencePose.roll, referencePose.pitch, referencePose.yaw);
            Eigen::Affine3f refTransformInv = refTransform.inverse();

            pcl::PointCloud<PointType>::Ptr cloudInRef(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*transformedCloud, *cloudInRef, refTransformInv);

            *submapAccumulated += *cloudInRef;
        }

        // Downsample the accumulated submap
        pcl::PointCloud<PointType>::Ptr submapDS(new pcl::PointCloud<PointType>());
        downSizeFilterScancontext_.setInputCloud(submapAccumulated);
        downSizeFilterScancontext_.filter(*submapDS);

        RCLCPP_INFO(this->get_logger(), "Accumulated submap: %d scans, %.2f meters, %zu points -> %zu points after downsampling",
                    submapScanCount_, submapAccumulatedDistance_, submapAccumulated->points.size(), submapDS->points.size());

        return submapDS;
    }

    void resetSubmapBuffer() {
        submapScanBuffer_.clear();
        submapPoseBuffer_.clear();
        submapAccumulatedDistance_ = 0.0;
        submapScanCount_ = 0;
        submapInitialized_ = false;
    }

    pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf) {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

        int numberOfCores = 16;
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }

        return cloudOut;
    }

    void pubPath() {
        nav_msgs::msg::Odometry odomAftPGO;
        nav_msgs::msg::Path pathAftPGO;
        pathAftPGO.header.frame_id = "camera_init";
        pathAftPGO.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes_.at(recentIdxUpdated_ - 1) * 1e9));

        mKF_.lock();
        for (int node_idx=0; node_idx < recentIdxUpdated_; node_idx++)
        {
            const Pose6D& pose_est = keyframePosesUpdated_.at(node_idx);

            nav_msgs::msg::Odometry odomAftPGOthis;
            odomAftPGOthis.header.frame_id = "camera_init";
            odomAftPGOthis.child_frame_id = "/aft_pgo";
            odomAftPGOthis.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes_.at(node_idx) * 1e9));
            odomAftPGOthis.pose.pose.position.x = pose_est.x;
            odomAftPGOthis.pose.pose.position.y = pose_est.y;
            odomAftPGOthis.pose.pose.position.z = pose_est.z;

            tf2::Quaternion q;
            q.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
            odomAftPGOthis.pose.pose.orientation = tf2::toMsg(q);
            odomAftPGO = odomAftPGOthis;

            geometry_msgs::msg::PoseStamped poseStampAftPGO;
            poseStampAftPGO.header = odomAftPGOthis.header;
            poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

            pathAftPGO.poses.push_back(poseStampAftPGO);
        }
        mKF_.unlock();

        pubOdomAftPGO_->publish(odomAftPGO);
        pubPathAftPGO_->publish(pathAftPGO);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = odomAftPGO.header.stamp;
        transform.header.frame_id = "camera_init";
        transform.child_frame_id = "/aft_pgo";
        transform.transform.translation.x = odomAftPGO.pose.pose.position.x;
        transform.transform.translation.y = odomAftPGO.pose.pose.position.y;
        transform.transform.translation.z = odomAftPGO.pose.pose.position.z;
        transform.transform.rotation = odomAftPGO.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform);
    }

    void updatePoses() {
        mKF_.lock();
        for (int node_idx=0; node_idx < int(isamCurrentEstimate_.size()); node_idx++)
        {
            Pose6D& p = keyframePosesUpdated_[node_idx];
            p.x = isamCurrentEstimate_.at<gtsam::Pose3>(node_idx).translation().x();
            p.y = isamCurrentEstimate_.at<gtsam::Pose3>(node_idx).translation().y();
            p.z = isamCurrentEstimate_.at<gtsam::Pose3>(node_idx).translation().z();
            p.roll = isamCurrentEstimate_.at<gtsam::Pose3>(node_idx).rotation().roll();
            p.pitch = isamCurrentEstimate_.at<gtsam::Pose3>(node_idx).rotation().pitch();
            p.yaw = isamCurrentEstimate_.at<gtsam::Pose3>(node_idx).rotation().yaw();
        }
        mKF_.unlock();

        mtxRecentPose_.lock();
        const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate_.at<gtsam::Pose3>(int(isamCurrentEstimate_.size())-1);
        recentOptimizedX_ = lastOptimizedPose.translation().x();
        recentOptimizedY_ = lastOptimizedPose.translation().y();
        recentIdxUpdated_ = int(keyframePosesUpdated_.size()) - 1;
        mtxRecentPose_.unlock();
    }

    void runISAM2opt() {
        isam_->update(gtSAMgraph_, initialEstimate_);
        isam_->update();

        gtSAMgraph_.resize(0);
        initialEstimate_.clear();

        isamCurrentEstimate_ = isam_->calculateEstimate();
        updatePoses();
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn) {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(
                                        transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(),
                                        transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw());

        int numberOfCores = 8;
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto& pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    void loopFindNearKeyframesCloud(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx) {
        nearKeyframes->clear();
        for (int i = -submap_size; i <= submap_size; ++i) {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= int(keyframeLaserClouds_.size()))
                continue;

            mKF_.lock();
            *nearKeyframes += * local2global(keyframeLaserClouds_[keyNear], keyframePosesUpdated_[root_idx]);
            mKF_.unlock();
        }

        if (nearKeyframes->empty())
            return;

        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP_.setInputCloud(nearKeyframes);
        downSizeFilterICP_.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    std::optional<gtsam::Pose3> doICPVirtualRelative(int _loop_kf_idx, int _curr_kf_idx) {
        int historyKeyframeSearchNum = 25;
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
        loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx);
        loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx);

        sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
        pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
        cureKeyframeCloudMsg.header.frame_id = "camera_init";
        cureKeyframeCloudMsg.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes_.at(_curr_kf_idx) * 1e9));
        pubLoopScanLocal_->publish(cureKeyframeCloudMsg);

        sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
        pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
        targetKeyframeCloudMsg.header.frame_id = "camera_init";
        targetKeyframeCloudMsg.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes_.at(_loop_kf_idx) * 1e9));
        pubLoopSubmapLocal_->publish(targetKeyframeCloudMsg);

        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(150);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(targetKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        float loopFitnessScoreThreshold = 0.3;
        if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
            RCLCPP_INFO(this->get_logger(), "[SC loop] ICP fitness test failed (%.3f > %.3f). Reject this SC loop.",
                       icp.getFitnessScore(), loopFitnessScoreThreshold);
            return std::nullopt;
        } else {
            RCLCPP_INFO(this->get_logger(), "[SC loop] ICP fitness test passed (%.3f < %.3f). Add this SC loop.",
                       icp.getFitnessScore(), loopFitnessScoreThreshold);
        }

        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

        return poseFrom.between(poseTo);
    }

    void process_pg() {
        while(rclcpp::ok())
        {
            while (!odometryBuf_.empty() && !fullResBuf_.empty())
            {
                mBuf_.lock();
                while (!odometryBuf_.empty() &&
                       rclcpp::Time(odometryBuf_.front()->header.stamp).seconds() <
                       rclcpp::Time(fullResBuf_.front()->header.stamp).seconds())
                    odometryBuf_.pop();
                if (odometryBuf_.empty())
                {
                    mBuf_.unlock();
                    break;
                }

                timeLaserOdometry_ = rclcpp::Time(odometryBuf_.front()->header.stamp).seconds();
                timeLaser_ = rclcpp::Time(fullResBuf_.front()->header.stamp).seconds();

                laserCloudFullRes_->clear();
                pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
                pcl::fromROSMsg(*fullResBuf_.front(), *thisKeyFrame);
                fullResBuf_.pop();

                Pose6D pose_curr = getOdom(odometryBuf_.front());
                odometryBuf_.pop();

                double eps = 0.1;
                while (!gpsBuf_.empty()) {
                    auto thisGPS = gpsBuf_.front();
                    auto thisGPSTime = rclcpp::Time(thisGPS->header.stamp).seconds();
                    if(abs(thisGPSTime - timeLaserOdometry_) < eps) {
                        currGPS_ = thisGPS;
                        hasGPSforThisKF_ = true;
                        break;
                    } else {
                        hasGPSforThisKF_ = false;
                    }
                    gpsBuf_.pop();
                }
                mBuf_.unlock();

                odom_pose_prev_ = odom_pose_curr_;
                odom_pose_curr_ = pose_curr;

                // Initialize submap buffer on first scan
                if (!submapInitialized_) {
                    submapStartPose_ = pose_curr;
                    submapInitialized_ = true;
                }

                // Add current scan to submap buffer
                submapScanBuffer_.push_back(thisKeyFrame);
                submapPoseBuffer_.push_back(pose_curr);
                submapScanCount_++;

                // Calculate distance from submap start
                Pose6D dtf_from_start = diffTransformation(submapStartPose_, pose_curr);
                double distance_from_start = sqrt(dtf_from_start.x*dtf_from_start.x +
                                                   dtf_from_start.y*dtf_from_start.y +
                                                   dtf_from_start.z*dtf_from_start.z);
                submapAccumulatedDistance_ = distance_from_start;

                // Check if we should create a keyframe (either threshold crossed)
                bool shouldCreateKeyframe = (submapAccumulatedDistance_ >= submapDistanceThreshold_) ||
                                           (submapScanCount_ >= submapScanThreshold_);

                if (!shouldCreateKeyframe)
                    continue;

                // Accumulate submap from buffer
                pcl::PointCloud<PointType>::Ptr thisKeyFrameDS = accumulateSubmap();

                // Use the first pose of the submap as the keyframe pose
                Pose6D keyframePose = submapPoseBuffer_[0];
                double keyframeTime = timeLaserOdometry_;

                if(!gpsOffsetInitialized_) {
                    if(hasGPSforThisKF_) {
                        gpsAltitudeInitOffset_ = currGPS_->altitude;
                        gpsOffsetInitialized_ = true;
                    }
                }

                mKF_.lock();
                keyframeLaserClouds_.push_back(thisKeyFrameDS);
                keyframePoses_.push_back(keyframePose);
                keyframePosesUpdated_.push_back(keyframePose);
                keyframeTimes_.push_back(keyframeTime);

                scManager_.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

                laserCloudMapPGORedraw_ = true;
                mKF_.unlock();

                // Reset submap buffer for next accumulation
                resetSubmapBuffer();
                submapStartPose_ = pose_curr;
                submapInitialized_ = true;

                const int prev_node_idx = keyframePoses_.size() - 2;
                const int curr_node_idx = keyframePoses_.size() - 1;
                if(!gtSAMgraphMade_) {
                    const int init_node_idx = 0;
                    gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses_.at(init_node_idx));

                    mtxPosegraph_.lock();
                    {
                        gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise_));
                        initialEstimate_.insert(init_node_idx, poseOrigin);
                    }
                    mtxPosegraph_.unlock();

                    gtSAMgraphMade_ = true;

                    RCLCPP_INFO(this->get_logger(), "posegraph prior node %d added", init_node_idx);
                } else {
                    gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses_.at(prev_node_idx));
                    gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses_.at(curr_node_idx));

                    mtxPosegraph_.lock();
                    {
                        gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise_));

                        if(hasGPSforThisKF_) {
                            double curr_altitude_offseted = currGPS_->altitude - gpsAltitudeInitOffset_;
                            mtxRecentPose_.lock();
                            gtsam::Point3 gpsConstraint(recentOptimizedX_, recentOptimizedY_, curr_altitude_offseted);
                            mtxRecentPose_.unlock();
                            gtSAMgraph_.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise_));
                            RCLCPP_INFO(this->get_logger(), "GPS factor added at node %d", curr_node_idx);
                        }
                        initialEstimate_.insert(curr_node_idx, poseTo);
                    }
                    mtxPosegraph_.unlock();

                    if(curr_node_idx % 100 == 0)
                        RCLCPP_INFO(this->get_logger(), "posegraph odom node %d added.", curr_node_idx);
                }

                std::string curr_node_idx_str = padZeros(curr_node_idx);
                pcl::io::savePCDFileBinary(pgScansDirectory_ + curr_node_idx_str + ".pcd", *thisKeyFrameDS);
                pgTimeSaveStream_ << keyframeTime << std::endl;
            }

            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }
    }

    void performSCLoopClosure() {
        if(int(keyframePoses_.size()) < scManager_.NUM_EXCLUDE_RECENT)
            return;

        auto detectResult = scManager_.detectLoopClosureID();
        int SCclosestHistoryFrameID = detectResult.first;
        if(SCclosestHistoryFrameID != -1) {
            const int prev_node_idx = SCclosestHistoryFrameID;
            const int curr_node_idx = keyframePoses_.size() - 1;
            RCLCPP_INFO(this->get_logger(), "Loop detected! - between %d and %d", prev_node_idx, curr_node_idx);

            mBuf_.lock();
            scLoopICPBuf_.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
            mBuf_.unlock();
        }
    }

    void process_lcd() {
        float loopClosureFrequency = 1.0;
        rclcpp::Rate rate(loopClosureFrequency);
        while (rclcpp::ok())
        {
            rate.sleep();
            performSCLoopClosure();
        }
    }

    void process_icp() {
        while(rclcpp::ok())
        {
            while (!scLoopICPBuf_.empty())
            {
                if(scLoopICPBuf_.size() > 30) {
                    RCLCPP_WARN(this->get_logger(), "Too many loop closure candidates to be ICPed is waiting ...");
                }

                mBuf_.lock();
                std::pair<int, int> loop_idx_pair = scLoopICPBuf_.front();
                scLoopICPBuf_.pop();
                mBuf_.unlock();

                const int prev_node_idx = loop_idx_pair.first;
                const int curr_node_idx = loop_idx_pair.second;
                auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx);
                if(relative_pose_optional) {
                    gtsam::Pose3 relative_pose = relative_pose_optional.value();
                    mtxPosegraph_.lock();
                    gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise_));
                    mtxPosegraph_.unlock();
                }
            }

            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }
    }

    void process_viz_path() {
        float hz = 10.0;
        rclcpp::Rate rate(hz);
        while (rclcpp::ok()) {
            rate.sleep();
            if(recentIdxUpdated_ > 1) {
                pubPath();
            }
        }
    }

    void process_isam() {
        float hz = 1;
        rclcpp::Rate rate(hz);
        while (rclcpp::ok()) {
            rate.sleep();
            if(gtSAMgraphMade_) {
                mtxPosegraph_.lock();
                runISAM2opt();
                RCLCPP_INFO(this->get_logger(), "running isam2 optimization ...");
                mtxPosegraph_.unlock();

                saveOptimizedVerticesTUMformat(isamCurrentEstimate_, pgTUMformat_);
                saveOdometryVerticesTUMformat(odomTUMformat_);
            }
        }
    }

    void pubMap() {
        int SKIP_FRAMES = 2;
        int counter = 0;

        laserCloudMapPGO_->clear();

        mKF_.lock();
        for (int node_idx=0; node_idx < recentIdxUpdated_; node_idx++) {
            if(counter % SKIP_FRAMES == 0) {
                *laserCloudMapPGO_ += *local2global(keyframeLaserClouds_[node_idx], keyframePosesUpdated_[node_idx]);
            }
            counter++;
        }
        mKF_.unlock();

        downSizeFilterMapPGO_.setInputCloud(laserCloudMapPGO_);
        downSizeFilterMapPGO_.filter(*laserCloudMapPGO_);

        sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
        pcl::toROSMsg(*laserCloudMapPGO_, laserCloudMapPGOMsg);
        laserCloudMapPGOMsg.header.frame_id = "camera_init";
        laserCloudMapPGOMsg.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes_.at(recentIdxUpdated_ - 1) * 1e9));
        pubMapAftPGO_->publish(laserCloudMapPGOMsg);
    }

    void process_viz_map() {
        float vizmapFrequency = 0.1;
        rclcpp::Rate rate(vizmapFrequency);
        while (rclcpp::ok()) {
            rate.sleep();
            if(recentIdxUpdated_ > 1) {
                pubMap();
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SCPGONode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
