#ifndef FAST_LIO_LOCALIZATION_H
#define FAST_LIO_LOCALIZATION_H

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <open3d/Open3D.h>
#include <faiss/IndexFlat.h>
#include <faiss/IndexIVFFlat.h>
#include <faiss/utils/Heap.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>

// 类型定义
using PointXYZ = pcl::PointXYZ;
using PointCloudXYZ = pcl::PointCloud<PointXYZ>;
using EigenMatrix4f = Eigen::Matrix4f;
using EigenVector3f = Eigen::Vector3f;
using Open3DPointCloud = open3d::geometry::PointCloud;
using FAISSIndexPtr = std::shared_ptr<faiss::IndexFlatL2>;

// 全局配置参数（复刻Python版）
struct Config {
    // 定位阈值（探维360适配）
    float LOCALIZATION_TH_init = 0.1f;
    float LOCALIZATION_TH_max = 0.7f;
    float LOCALIZATION_TH = 0.5f;
    
    // 下采样参数
    float MAP_VOXEL_SIZE = 0.05f;
    float SCAN_VOXEL_SIZE = 0.01f;
    
    // 频率/视野参数
    float FREQ_LOCALIZATION = 1.0f;
    float FOV = M_PI * 2;
    float FOV_FAR = 50.0f;
    float FOV_FAR_edge = 2.0f;
    float FOV_FAR_offset = 2.0f;
    
    // 配准/可视化半径
    float DISPLAY_RADIUS = 30.0f;
    float REGISTRATION_RADIUS = 10.0f;
    
    // 坐标系参数
    bool PUBLISH_TF = true;
    std::string FRAME_ID_MAP = "map";
    std::string FRAME_ID_BASE = "base_link";
    
    // 重试/超时参数
    int POLL_TIMEOUT = 10;
    int MAX_ATTEMPTS = 100;
    float POSE_WAIT_TIMEOUT = 5.0f;
    float ICP_TIMEOUT = 2.0f;
    
    // 坐标系修正
    float INIT_POSE_ROT_CORRECTION = 0.0f;
    
    // 缓存配置
    bool CACHE_ENABLED = true;
    std::string CACHE_DIR = "";
    std::string CACHE_POINT_FILE = "global_map_points.bin";
    std::string CACHE_INDEX_FILE = "global_map_index.faiss";
    int CACHE_POINT_COUNT = 49048115;
};

class FastLIOLocalization {
public:
    FastLIOLocalization(ros::NodeHandle& nh);
    ~FastLIOLocalization();
    
    // 主循环
    void run();

private:
    // ROS回调函数
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    // 地图相关
    bool loadMapFromCache();
    bool loadIndexFromCache();
    void saveMapToCache();
    void saveIndexToCache();
    void buildFAISSSIndexAsync();
    void buildOpen3DPointCloudAsync();
    void parseMapAsync(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    // 位姿转换
    EigenMatrix4f poseToMat(const geometry_msgs::Pose& pose);
    EigenMatrix4f initialPoseToMat(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void matToPose(const EigenMatrix4f& mat, geometry_msgs::Point& point, geometry_msgs::Quaternion& quat);
    EigenMatrix4f fixRotationMatrix(const EigenMatrix4f& mat);
    EigenMatrix4f inverseSE3(const EigenMatrix4f& mat);
    
    // FAISS查询
    std::vector<long long> queryFAISSBallPoint(const EigenVector3f& center, float radius);
    
    // ICP配准
    std::pair<EigenMatrix4f, float> registrationAtScale(
        const std::shared_ptr<Open3DPointCloud>& pc_scan,
        const std::shared_ptr<Open3DPointCloud>& pc_map,
        const EigenMatrix4f& initial,
        float scale);
    
    // 点云裁剪
    std::shared_ptr<Open3DPointCloud> cropScanInFOV(
        const std::shared_ptr<Open3DPointCloud>& cur_scan,
        const nav_msgs::Odometry::ConstPtr& odom);
    
    std::shared_ptr<Open3DPointCloud> getRegistrationSubmap(
        const EigenVector3f& current_pos,
        const EigenMatrix4f& pose_estimation,
        const nav_msgs::Odometry::ConstPtr& odom);
    
    // 定位核心
    bool globalLocalization(const EigenMatrix4f& pose_estimation);
    bool doLocalizationAttempt(const EigenMatrix4f& trans_map_to_odom);
    
    // 点云发布
    void publishPointCloud(
        ros::Publisher& pub,
        const std_msgs::Header& header,
        const std::shared_ptr<Open3DPointCloud>& pc);
    
    void quickPublishSubmap(const EigenVector3f& current_pos, const std_msgs::Header& header);
    
    // 重新初始化线程
    void reinitializationThread();
    void localizationThread();
    
    // 工具函数
    std::shared_ptr<Open3DPointCloud> pclToOpen3D(const PointCloudXYZ::Ptr& pcl_pc);
    PointCloudXYZ::Ptr open3DToPCL(const std::shared_ptr<Open3DPointCloud>& o3d_pc);
    void createCacheDir();
    
    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber sub_initial_pose_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_map_;
    
    ros::Publisher pub_pc_in_map_;
    ros::Publisher pub_submap_;
    ros::Publisher pub_map_to_odom_;
    
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 全局变量（带线程安全）
    std::mutex global_mutex_;
    std::condition_variable cv_;
    
    // 地图数据
    std::shared_ptr<Open3DPointCloud> global_map_;
    std::vector<float> global_map_np_;  // 扁平化存储的点云数据 (N*3)
    std::vector<float> global_map_normals_;
    std::vector<float> global_map_gravity_;
    FAISSIndexPtr global_map_index_;
    std::pair<EigenVector3f, EigenVector3f> map_bounds_;
    
    // 状态标记
    std::atomic<bool> initialized_ = false;
    std::atomic<bool> index_ready_ = false;
    std::atomic<bool> map_ready_ = false;
    std::atomic<bool> scan_valid_ = false;
    
    // 核心变换矩阵
    EigenMatrix4f T_map_to_odom_;
    
    // 缓存数据
    nav_msgs::Odometry::ConstPtr cur_odom_;
    std::shared_ptr<Open3DPointCloud> cur_scan_;
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr new_initial_pose_;
    
    // 配置参数
    Config config_;
    
    // 线程
    std::thread reinit_thread_;
    std::thread localization_thread_;
    std::atomic<bool> is_running_ = true;
};

#endif // FAST_LIO_LOCALIZATION_H
