#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <set>

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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

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
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "common/common.h"
#include "common/tic_toc.h"

#include "scancontext/Scancontext.h"

using namespace gtsam;

using std::cout;
using std::endl;

class LaserPGO : public rclcpp::Node
{
public:
    LaserPGO() : Node("laser_pgo")
    {
        // Declare parameters
        this->declare_parameter<std::string>("save_directory", "/tmp/sc_pgo/");
        this->declare_parameter<double>("keyframe_meter_gap", 2.0);
        this->declare_parameter<double>("keyframe_deg_gap", 10.0);
        this->declare_parameter<double>("sc_dist_thres", 0.2);
        this->declare_parameter<double>("sc_max_radius", 80.0);
        this->declare_parameter<double>("mapviz_filter_size", 0.4);
        this->declare_parameter<bool>("save_trajectory", true);

        // Get parameters
        this->get_parameter("save_directory", save_directory);
        this->get_parameter("keyframe_meter_gap", keyframeMeterGap);
        this->get_parameter("keyframe_deg_gap", keyframeDegGap);
        this->get_parameter("sc_dist_thres", scDistThres);
        this->get_parameter("sc_max_radius", scMaximumRadius);
        double mapVizFilterSize;
        this->get_parameter("mapviz_filter_size", mapVizFilterSize);
        this->get_parameter("save_trajectory", saveTrajectory);

        keyframeRadGap = deg2rad(keyframeDegGap);

        // Setup save directory
        pgScansDirectory = save_directory + "Scans/";
        auto unused1 = system((std::string("exec rm -r ") + pgScansDirectory).c_str());
        auto unused2 = system((std::string("mkdir -p ") + pgScansDirectory).c_str());
        (void)unused1;
        (void)unused2;

        pgTimeSaveStream = std::fstream(save_directory + "times.txt", std::fstream::out);
        pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);

        // Initialize GTSAM
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);
        initNoises();

        // Setup Scan Context
        scManager.setSCdistThres(scDistThres);
        scManager.setMaximumRadius(scMaximumRadius);

        float filter_size = 0.4;
        downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
        downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);
        downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

        // Subscribers
        subLaserCloudFullRes = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 100,
            std::bind(&LaserPGO::laserCloudFullResHandler, this, std::placeholders::_1));

        subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", 100,
            std::bind(&LaserPGO::laserOdometryHandler, this, std::placeholders::_1));

        // Publishers
        pubOdomAftPGO = this->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", 100);
        pubPathAftPGO = this->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", 100);
        pubMapAftPGO = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_map", 100);
        pubLoopScanLocal = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", 100);
        pubLoopSubmapLocal = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", 100);

        // TF broadcaster
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Start threads
        posegraph_slam_thread = std::thread(&LaserPGO::process_pg, this);
        lc_detection_thread = std::thread(&LaserPGO::process_lcd, this);
        icp_calculation_thread = std::thread(&LaserPGO::process_icp, this);
        isam_update_thread = std::thread(&LaserPGO::process_isam, this);
        viz_map_thread = std::thread(&LaserPGO::process_viz_map, this);
        viz_path_thread = std::thread(&LaserPGO::process_viz_path, this);

        RCLCPP_INFO(this->get_logger(), "SC-PGO node initialized");
    }

    ~LaserPGO()
    {
        if (posegraph_slam_thread.joinable()) posegraph_slam_thread.join();
        if (lc_detection_thread.joinable()) lc_detection_thread.join();
        if (icp_calculation_thread.joinable()) icp_calculation_thread.join();
        if (isam_update_thread.joinable()) isam_update_thread.join();
        if (viz_map_thread.joinable()) viz_map_thread.join();
        if (viz_path_thread.joinable()) viz_path_thread.join();

        delete isam;
        pgTimeSaveStream.close();
    }

private:
    // Parameters
    double keyframeMeterGap;
    double keyframeDegGap, keyframeRadGap;
    double translationAccumulated = 1000000.0;
    double rotaionAccumulated = 1000000.0;
    bool isNowKeyFrame = false;

    Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> odometryBuf;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> fullResBuf;
    std::queue<std::pair<int, int>> scLoopICPBuf;
    std::set<std::pair<int, int>> processedLoopPairs;  // Track processed loop closures

    std::mutex mBuf;
    std::mutex mKF;

    double timeLaserOdometry = 0.0;
    double timeLaser = 0.0;

    pcl::PointCloud<PointType>::Ptr laserCloudFullRes{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO{new pcl::PointCloud<PointType>()};

    std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds;
    std::vector<Pose6D> keyframePoses;
    std::vector<Pose6D> keyframePosesUpdated;
    std::vector<double> keyframeTimes;
    int recentIdxUpdated = 0;

    gtsam::NonlinearFactorGraph gtSAMgraph;
    bool gtSAMgraphMade = false;
    gtsam::Values initialEstimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isamCurrentEstimate;
    bool aLoopIsClosed = false;

    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr odomNoise;
    noiseModel::Base::shared_ptr robustLoopNoise;

    pcl::VoxelGrid<PointType> downSizeFilterScancontext;
    SCManager scManager;
    double scDistThres, scMaximumRadius;

    pcl::VoxelGrid<PointType> downSizeFilterICP;
    std::mutex mtxICP;
    std::mutex mtxPosegraph;
    std::mutex mtxRecentPose;

    pcl::PointCloud<PointType>::Ptr laserCloudMapPGO{new pcl::PointCloud<PointType>()};
    pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
    bool laserCloudMapPGORedraw = true;

    double recentOptimizedX = 0.0;
    double recentOptimizedY = 0.0;

    std::string save_directory;
    std::string pgScansDirectory;
    std::fstream pgTimeSaveStream;
    bool saveTrajectory;

    // ROS2 objects
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // Threads
    std::thread posegraph_slam_thread;
    std::thread lc_detection_thread;
    std::thread icp_calculation_thread;
    std::thread isam_update_thread;
    std::thread viz_map_thread;
    std::thread viz_path_thread;

    // Helper functions
    std::string padZeros(int val, int num_digits = 6) {
        std::ostringstream out;
        out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
        return out.str();
    }

    gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
    }

    void laserOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr _laserOdometry)
    {
        mBuf.lock();
        odometryBuf.push(_laserOdometry);
        mBuf.unlock();
    }

    void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr _laserCloudFullRes)
    {
        mBuf.lock();
        fullResBuf.push(_laserCloudFullRes);
        mBuf.unlock();
    }

    void initNoises(void)
    {
        gtsam::Vector priorNoiseVector6(6);
        priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
        priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

        gtsam::Vector odomNoiseVector6(6);
        odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
        odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

        double loopNoiseScore = 0.5;
        gtsam::Vector robustNoiseVector6(6);
        robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
        robustLoopNoise = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
                        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
    }

    Pose6D getOdom(nav_msgs::msg::Odometry::ConstSharedPtr _odom)
    {
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

    Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
    {
        Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
        Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
        Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
        Eigen::Affine3f SE3_delta;
        SE3_delta.matrix() = SE3_delta0;
        float dx, dy, dz, droll, dpitch, dyaw;
        pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);

        return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
    }

    pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
    {
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

    void pubPath(void)
    {
        nav_msgs::msg::Odometry odomAftPGO;
        nav_msgs::msg::Path pathAftPGO;
        pathAftPGO.header.frame_id = "camera_init";

        mKF.lock();
        for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++)
        {
            const Pose6D& pose_est = keyframePosesUpdated.at(node_idx);

            nav_msgs::msg::Odometry odomAftPGOthis;
            odomAftPGOthis.header.frame_id = "camera_init";
            odomAftPGOthis.child_frame_id = "/aft_pgo";
            odomAftPGOthis.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes.at(node_idx) * 1e9));
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

            pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
            pathAftPGO.header.frame_id = "camera_init";
            pathAftPGO.poses.push_back(poseStampAftPGO);
        }
        mKF.unlock();

        pubOdomAftPGO->publish(odomAftPGO);
        pubPathAftPGO->publish(pathAftPGO);

        // Broadcast TF
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = odomAftPGO.header.stamp;
        transformStamped.header.frame_id = "camera_init";
        transformStamped.child_frame_id = "/aft_pgo";
        transformStamped.transform.translation.x = odomAftPGO.pose.pose.position.x;
        transformStamped.transform.translation.y = odomAftPGO.pose.pose.position.y;
        transformStamped.transform.translation.z = odomAftPGO.pose.pose.position.z;
        transformStamped.transform.rotation = odomAftPGO.pose.pose.orientation;
        tf_broadcaster->sendTransform(transformStamped);
    }

    void updatePoses(void)
    {
        mKF.lock();
        for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
        {
            Pose6D& p = keyframePosesUpdated[node_idx];
            p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
            p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
            p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
            p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
            p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
            p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
        }
        mKF.unlock();

        mtxRecentPose.lock();
        const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
        recentOptimizedX = lastOptimizedPose.translation().x();
        recentOptimizedY = lastOptimizedPose.translation().y();
        recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;
        mtxRecentPose.unlock();

        // Save trajectory after every optimization if enabled
        if (saveTrajectory) {
            saveOptimizedTrajectory();
        }
    }

    void saveOptimizedTrajectory(void)
    {
        // Save in TUM format: timestamp tx ty tz qx qy qz qw
        std::string trajFilePath = save_directory + "optimized_trajectory_tum.txt";
        std::ofstream trajFile(trajFilePath);
        trajFile.precision(std::numeric_limits<double>::max_digits10);

        mKF.lock();
        for (size_t i = 0; i < keyframePosesUpdated.size(); i++)
        {
            const Pose6D& p = keyframePosesUpdated[i];
            double timestamp = keyframeTimes[i];

            // Convert roll, pitch, yaw to quaternion
            tf2::Quaternion q;
            q.setRPY(p.roll, p.pitch, p.yaw);

            trajFile << timestamp << " "
                     << p.x << " " << p.y << " " << p.z << " "
                     << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
        mKF.unlock();

        trajFile.close();
    }

    void runISAM2opt(void)
    {
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        isamCurrentEstimate = isam->calculateEstimate();
        updatePoses();

        aLoopIsClosed = false;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
    {
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
            const auto *pointFrom = &cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom->intensity;
        }
        return cloudOut;
    }

    void loopFindNearKeyframesCloud(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
    {
        nearKeyframes->clear();
        for (int i = -submap_size; i <= submap_size; ++i) {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()))
                continue;

            mKF.lock();
            *nearKeyframes += *local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[root_idx]);
            mKF.unlock();
        }

        if (nearKeyframes->empty())
            return;

        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    std::optional<gtsam::Pose3> doICPVirtualRelative(int _loop_kf_idx, int _curr_kf_idx)
    {
        int historyKeyframeSearchNum = 25;
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
        loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx);
        loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx);

        sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
        pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
        cureKeyframeCloudMsg.header.frame_id = "camera_init";
        pubLoopScanLocal->publish(cureKeyframeCloudMsg);

        sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
        pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
        targetKeyframeCloudMsg.header.frame_id = "camera_init";
        pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);

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
            std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
            return std::nullopt;
        } else {
            std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
        }

        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

        return poseFrom.between(poseTo);
    }

    void process_pg()
    {
        while(rclcpp::ok())
        {
            while (!odometryBuf.empty() && !fullResBuf.empty())
            {
                mBuf.lock();
                while (!odometryBuf.empty() &&
                       rclcpp::Time(odometryBuf.front()->header.stamp).seconds() <
                       rclcpp::Time(fullResBuf.front()->header.stamp).seconds())
                    odometryBuf.pop();
                if (odometryBuf.empty())
                {
                    mBuf.unlock();
                    break;
                }

                timeLaserOdometry = rclcpp::Time(odometryBuf.front()->header.stamp).seconds();
                timeLaser = rclcpp::Time(fullResBuf.front()->header.stamp).seconds();

                laserCloudFullRes->clear();
                pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
                pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
                fullResBuf.pop();

                Pose6D pose_curr = getOdom(odometryBuf.front());
                odometryBuf.pop();
                mBuf.unlock();

                odom_pose_prev = odom_pose_curr;
                odom_pose_curr = pose_curr;
                Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr);

                double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z);
                translationAccumulated += delta_translation;
                rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw);

                if (translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap) {
                    isNowKeyFrame = true;
                    translationAccumulated = 0.0;
                    rotaionAccumulated = 0.0;
                } else {
                    isNowKeyFrame = false;
                }

                if (!isNowKeyFrame)
                    continue;

                pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
                downSizeFilterScancontext.setInputCloud(thisKeyFrame);
                downSizeFilterScancontext.filter(*thisKeyFrameDS);

                mKF.lock();
                keyframeLaserClouds.push_back(thisKeyFrameDS);
                keyframePoses.push_back(pose_curr);
                keyframePosesUpdated.push_back(pose_curr);
                keyframeTimes.push_back(timeLaserOdometry);

                scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

                laserCloudMapPGORedraw = true;
                mKF.unlock();

                const int prev_node_idx = keyframePoses.size() - 2;
                const int curr_node_idx = keyframePoses.size() - 1;

                if (!gtSAMgraphMade) {
                    const int init_node_idx = 0;
                    gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));

                    mtxPosegraph.lock();
                    {
                        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                        initialEstimate.insert(init_node_idx, poseOrigin);
                    }
                    mtxPosegraph.unlock();

                    gtSAMgraphMade = true;
                    cout << "posegraph prior node " << init_node_idx << " added" << endl;
                } else {
                    gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                    gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                    mtxPosegraph.lock();
                    {
                        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise));
                        initialEstimate.insert(curr_node_idx, poseTo);
                        aLoopIsClosed = true;  // New keyframe added to graph
                    }
                    mtxPosegraph.unlock();

                    if (curr_node_idx % 100 == 0)
                        cout << "posegraph odom node " << curr_node_idx << " added." << endl;
                }

                std::string curr_node_idx_str = padZeros(curr_node_idx);
                pcl::io::savePCDFileBinary(pgScansDirectory + curr_node_idx_str + ".pcd", *thisKeyFrame);
                pgTimeSaveStream << timeLaser << std::endl;
            }

            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }
    }

    void performSCLoopClosure(void)
    {
        if (int(keyframePoses.size()) < scManager.NUM_EXCLUDE_RECENT)
            return;

        auto detectResult = scManager.detectLoopClosureID();
        int SCclosestHistoryFrameID = detectResult.first;
        if (SCclosestHistoryFrameID != -1) {
            const int prev_node_idx = SCclosestHistoryFrameID;
            const int curr_node_idx = keyframePoses.size() - 1;

            // Create loop pair (always store with smaller index first for consistency)
            std::pair<int, int> loopPair = std::make_pair(
                std::min(prev_node_idx, curr_node_idx),
                std::max(prev_node_idx, curr_node_idx)
            );

            // Check if this loop has already been processed
            mBuf.lock();
            if (processedLoopPairs.find(loopPair) != processedLoopPairs.end()) {
                mBuf.unlock();
                return;  // Skip duplicate loop
            }

            cout << "[Loop found] Between " << prev_node_idx << " and " << curr_node_idx << endl;
            cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << endl;
            scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
            processedLoopPairs.insert(loopPair);  // Mark as processed
            mBuf.unlock();
        }
    }

    void process_lcd(void)
    {
        float loopClosureFrequency = 1.0;
        rclcpp::Rate rate(loopClosureFrequency);
        while (rclcpp::ok())
        {
            rate.sleep();
            performSCLoopClosure();
        }
    }

    void process_icp(void)
    {
        while(rclcpp::ok())
        {
            while (!scLoopICPBuf.empty())
            {
                if (scLoopICPBuf.size() > 30) {
                    RCLCPP_WARN(this->get_logger(), "Too many loop closure candidates to be ICPed is waiting...");
                }

                mBuf.lock();
                std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
                scLoopICPBuf.pop();
                mBuf.unlock();

                const int prev_node_idx = loop_idx_pair.first;
                const int curr_node_idx = loop_idx_pair.second;
                auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx);
                if (relative_pose_optional) {
                    gtsam::Pose3 relative_pose = relative_pose_optional.value();
                    mtxPosegraph.lock();
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                    aLoopIsClosed = true;  // Loop closure added to graph
                    mtxPosegraph.unlock();
                }
            }

            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }
    }

    void process_viz_path(void)
    {
        float hz = 10.0;
        rclcpp::Rate rate(hz);
        while (rclcpp::ok()) {
            rate.sleep();
            if (recentIdxUpdated > 1) {
                pubPath();
            }
        }
    }

    void process_isam(void)
    {
        float hz = 1;
        rclcpp::Rate rate(hz);
        while (rclcpp::ok()) {
            rate.sleep();
            if (gtSAMgraphMade && aLoopIsClosed) {
                mtxPosegraph.lock();
                runISAM2opt();
                cout << "running isam2 optimization ..." << endl;
                mtxPosegraph.unlock();
            }
        }
    }

    void pubMap(void)
    {
        int SKIP_FRAMES = 2;
        int counter = 0;

        laserCloudMapPGO->clear();

        mKF.lock();
        for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
            if (counter % SKIP_FRAMES == 0) {
                *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
            }
            counter++;
        }
        mKF.unlock();

        downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
        downSizeFilterMapPGO.filter(*laserCloudMapPGO);

        sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
        pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
        laserCloudMapPGOMsg.header.frame_id = "camera_init";
        pubMapAftPGO->publish(laserCloudMapPGOMsg);
    }

    void process_viz_map(void)
    {
        float vizmapFrequency = 0.1;
        rclcpp::Rate rate(vizmapFrequency);
        while (rclcpp::ok()) {
            rate.sleep();
            if (recentIdxUpdated > 1) {
                pubMap();
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserPGO>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
