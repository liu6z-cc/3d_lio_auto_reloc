/**
 * @file auto_relocalization_3d_node.cpp
 * @brief 3D自动重定位节点
 * 
 * 该节点用于3D SLAM系统的自动重定位功能：
 * 1. 监听3D里程计数据
 * 2. 当机器人靠近指定点时，发布初始位姿进行重定位
 * 3. 支持从文件加载上次保存的位姿进行初始化重定位
 * 4. 支持等待SLAM系统初始化完成后再进行重定位（解决点云加载慢的问题）
 * 5. 支持重试机制，确保重定位成功
 * 6. 重定位成功后发布状态，通知pose_recovery_3d_node开始保存位姿
 * 
 * 订阅话题：
 *   - /Odometry (nav_msgs/Odometry): 3D SLAM里程计数据
 * 
 * 发布话题：
 *   - /initialpose (geometry_msgs/PoseWithCovarianceStamped): 初始位姿
 *   - /pose_recovery_3d/reloc_status (std_msgs/Bool): 重定位完成状态
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <thread>
#include <atomic>
#include <mutex>

class AutoRelocalization3D {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    ros::Subscriber odom_sub_;
    ros::Subscriber global_pose_sub_;  // 订阅全局位姿用于验证重定位是否成功
    ros::Publisher reloc_status_pub_;
    ros::Publisher initialpose_pub_;      // 发布到 /initialpose
    ros::Publisher initialpose_2d_pub_;   // 发布到 /arc_pose_tool/initialpose_2d
    ros::ServiceServer trigger_reloc_server_;
    
    // 当前全局位姿（用于重定位验证）
    std::mutex pose_mutex_;
    double current_global_x_;
    double current_global_y_;
    double current_global_yaw_;  // 弧度
    std::atomic<bool> global_pose_received_;
    
    // 重定位验证参数
    double position_tolerance_;  // 位置容差（米）
    double yaw_tolerance_;       // 角度容差（弧度）
    std::string global_pose_topic_;
    
    // 重定位话题名称
    std::string initialpose_topic_;
    std::string initialpose_2d_topic_;
    
    // 目标点位姿参数（用于接近时触发重定位）
    double point_x_;
    double point_y_;
    double point_z_;
    double point_roll_;
    double point_pitch_;
    double point_yaw_;
    
    // 触发参数
    double trigger_distance_;
    bool has_relocated_;
    bool enabled_;
    bool auto_init_reloc_;
    
    // 等待和重试参数
    double init_wait_time_;      // 启动时等待时间（秒）
    double retry_interval_;      // 重试间隔（秒）
    int max_retries_;            // 最大重试次数
    bool check_slam_ready_;      // 是否检查SLAM就绪
    
    // 状态标志
    std::atomic<bool> slam_ready_;
    std::atomic<bool> reloc_success_;
    std::atomic<bool> odom_received_;
    
    // 角度单位设置
    bool yaw_in_degrees_;  // JSON文件中的yaw是否为度数
    
    // 文件路径（用于加载保存的位姿）
    std::string pose_file_path_;
    std::string pose_file_name_;
    int file_buffer_;
    
    // PCD地图路径
    std::string pcd_path_;
    
    // 初始化重定位线程
    std::thread init_reloc_thread_;

public:
    AutoRelocalization3D() : private_nh_("~"), has_relocated_(false), 
                              slam_ready_(false), reloc_success_(false), odom_received_(false),
                              global_pose_received_(false), current_global_x_(0.0), 
                              current_global_y_(0.0), current_global_yaw_(0.0) {
        // 读取参数
        private_nh_.param<double>("point_x", point_x_, 0.0);
        private_nh_.param<double>("point_y", point_y_, 0.0);
        private_nh_.param<double>("point_z", point_z_, 0.0);
        private_nh_.param<double>("point_roll", point_roll_, 0.0);
        private_nh_.param<double>("point_pitch", point_pitch_, 0.0);
        private_nh_.param<double>("point_yaw", point_yaw_, 0.0);
        private_nh_.param<double>("trigger_distance", trigger_distance_, 0.8);
        private_nh_.param<bool>("enabled", enabled_, true);
        private_nh_.param<bool>("auto_init_reloc", auto_init_reloc_, true);
        
        // 等待和重试参数
        private_nh_.param<double>("init_wait_time", init_wait_time_, 60.0);
        private_nh_.param<double>("retry_interval", retry_interval_, 5.0);
        private_nh_.param<int>("max_retries", max_retries_, 10);
        private_nh_.param<bool>("check_slam_ready", check_slam_ready_, true);
        
        // 重定位验证参数
        private_nh_.param<double>("position_tolerance", position_tolerance_, 1.0);  // 默认1米
        private_nh_.param<double>("yaw_tolerance", yaw_tolerance_, 0.35);  // 默认约20度(弧度)
        private_nh_.param<std::string>("global_pose_topic", global_pose_topic_, "/arc_pose_tool/base_pose2d");
        
        private_nh_.param<std::string>("pose_file_path", pose_file_path_, "");
        private_nh_.param<std::string>("pose_file_name", pose_file_name_, "robot_pose_3d");
        private_nh_.param<int>("file_buffer", file_buffer_, 2);
        private_nh_.param<std::string>("pcd_path", pcd_path_, "");
        private_nh_.param<bool>("yaw_in_degrees", yaw_in_degrees_, true);  // 默认JSON中的yaw是度数
        
        // 重定位话题参数
        private_nh_.param<std::string>("initialpose_topic", initialpose_topic_, "/initialpose");
        private_nh_.param<std::string>("initialpose_2d_topic", initialpose_2d_topic_, "/arc_pose_tool/initialpose_2d");
        
        // 订阅3D里程计
        odom_sub_ = nh_.subscribe("/Odometry", 1, &AutoRelocalization3D::odomCallback, this);
        
        // 订阅全局位姿（用于验证重定位是否成功）
        global_pose_sub_ = nh_.subscribe(global_pose_topic_, 1, &AutoRelocalization3D::globalPoseCallback, this);
        
        // 发布重定位状态（通知pose_recovery_3d_node可以开始保存位姿了）
        reloc_status_pub_ = nh_.advertise<std_msgs::Bool>("/pose_recovery_3d/reloc_status", 1, true);
        
        // 发布初始位姿（用于重定位）
        initialpose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(initialpose_topic_, 1);
        initialpose_2d_pub_ = nh_.advertise<std_msgs::String>(initialpose_2d_topic_, 1);
        
        // 提供手动触发重定位的服务
        trigger_reloc_server_ = nh_.advertiseService("trigger_relocalization", 
                                                      &AutoRelocalization3D::triggerRelocCallback, this);
        
        ROS_INFO("========================================");
        ROS_INFO("[auto_reloc_3d] Node Started");
        ROS_INFO("[auto_reloc_3d] Target Point: x=%.2f, y=%.2f, z=%.2f", point_x_, point_y_, point_z_);
        ROS_INFO("[auto_reloc_3d] Trigger Distance: %.2f meters", trigger_distance_);
        ROS_INFO("[auto_reloc_3d] Enabled: %s", enabled_ ? "true" : "false");
        ROS_INFO("[auto_reloc_3d] Auto Init Reloc: %s", auto_init_reloc_ ? "true" : "false");
        ROS_INFO("[auto_reloc_3d] Init Wait Time: %.1f seconds (max)", init_wait_time_);
        ROS_INFO("[auto_reloc_3d] Max Retries: %d, Retry Interval: %.1f seconds", max_retries_, retry_interval_);
        ROS_INFO("[auto_reloc_3d] Position Tolerance: %.2f m, Yaw Tolerance: %.2f rad (%.1f deg)", 
                 position_tolerance_, yaw_tolerance_, yaw_tolerance_ * 180.0 / M_PI);
        ROS_INFO("[auto_reloc_3d] Global Pose Topic: %s", global_pose_topic_.c_str());
        ROS_INFO("[auto_reloc_3d] Pose File Path: %s", pose_file_path_.c_str());
        ROS_INFO("[auto_reloc_3d] Pose File Name: %s", pose_file_name_.c_str());
        ROS_INFO("========================================");
        
        // 启动时自动重定位（在单独线程中执行，避免阻塞主循环）
        if (auto_init_reloc_) {
            init_reloc_thread_ = std::thread(&AutoRelocalization3D::performInitialRelocalization, this);
        }
    }
    
    ~AutoRelocalization3D() {
        // 等待初始化线程结束
        if (init_reloc_thread_.joinable()) {
            init_reloc_thread_.join();
        }
    }
    
    void globalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_global_x_ = msg->x;
        current_global_y_ = msg->y;
        // base_pose2d的theta是度数，转换为弧度存储
        current_global_yaw_ = msg->theta * M_PI / 180.0;
        global_pose_received_ = true;
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_received_ = true;  // 标记已收到里程计数据
        
        // 目标点触发重定位功能已禁用，因为会干扰自动初始化重定位
        // 如果需要此功能，请将 enabled 设为 true 并设置合适的目标点
        if (!enabled_) return;
        
        // 如果正在进行初始化重定位，不触发目标点重定位
        if (!reloc_success_) return;
        
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;
        double current_z = msg->pose.pose.position.z;
        
        // 计算到目标点的3D距离
        double distance = std::sqrt(
            std::pow(current_x - point_x_, 2) + 
            std::pow(current_y - point_y_, 2) +
            std::pow(current_z - point_z_, 2)
        );
        
        // 如果接近目标点且未重定位过
        if (distance < trigger_distance_ && !has_relocated_) {
            ROS_WARN("Approaching relocalization point (3D distance: %.2fm), triggering relocalization...", distance);
            callRelocService(point_x_, point_y_, point_z_, point_roll_, point_pitch_, point_yaw_);
            has_relocated_ = true;
        }
        // 离开目标点后重置标志（距离大于触发距离的2倍）
        else if (distance > trigger_distance_ * 2.0) {
            if (has_relocated_) {
                ROS_INFO("Left relocalization zone, resetting flag");
                has_relocated_ = false;
            }
        }
    }
    
    void publishRelocStatus(bool success) {
        std_msgs::Bool msg;
        msg.data = success;
        reloc_status_pub_.publish(msg);
        ROS_INFO("[auto_reloc_3d] Published reloc status: %s", success ? "SUCCESS" : "FAILED");
    }
    
    void performInitialRelocalization() {
        ROS_INFO("=== Starting Initial Relocalization Process ===");
        
        // 步骤1：尝试从文件加载上次保存的位姿
        double target_x, target_y, target_z, target_roll, target_pitch, target_yaw;
        if (!loadPoseFromFile(target_x, target_y, target_z, target_roll, target_pitch, target_yaw)) {
            ROS_WARN("No saved pose found, skipping initial relocalization");
            // 没有历史位姿，通知pose_recovery_3d_node可以开始保存
            publishRelocStatus(true);
            return;
        }
        
        ROS_INFO("Target pose loaded: x=%.2f, y=%.2f, yaw=%.2f rad (%.1f deg)", 
                 target_x, target_y, target_yaw, target_yaw * 180.0 / M_PI);
        
        // 步骤2：等待SLAM系统初始化，同时持续发布重定位位姿并检查是否匹配成功
        ROS_INFO("Waiting up to %.1f seconds for SLAM initialization and relocalization...", init_wait_time_);
        ROS_INFO("Will exit early if relocalization succeeds (position tolerance: %.2fm, yaw tolerance: %.2f rad)",
                 position_tolerance_, yaw_tolerance_);
        
        double waited = 0.0;
        double check_interval = 1.0;  // 每秒检查一次
        int publish_counter = 0;      // 发布计数器
        double publish_interval = 5.0; // 每5秒发布一次重定位位姿
        bool first_publish_done = false;
        
        while (waited < init_wait_time_ && ros::ok()) {
            // 每隔一定时间发布重定位位姿
            if (!first_publish_done || (waited > 10.0 && static_cast<int>(waited) % static_cast<int>(retry_interval_) == 0 && 
                static_cast<int>(waited) != publish_counter)) {
                publish_counter = static_cast<int>(waited);
                
                if (!first_publish_done) {
                    // 第一次发布，先等待几秒让系统启动
                    ROS_INFO("Waiting 5 seconds before first relocalization attempt...");
                    ros::Duration(5.0).sleep();
                    waited += 5.0;
                    first_publish_done = true;
                }
                
                ROS_INFO("Publishing relocalization pose (attempt at %.0fs)...", waited);
                callRelocService(target_x, target_y, target_z, target_roll, target_pitch, target_yaw);
            }
            
            // 检查当前位姿是否与目标位姿匹配
            if (checkPoseMatch(target_x, target_y, target_yaw)) {
                ROS_INFO("=== RELOCALIZATION SUCCESS ===");
                ROS_INFO("Current pose matches target pose within tolerance!");
                ROS_INFO("Waiting 3 seconds for pose to stabilize...");
                ros::Duration(3.0).sleep();
                
                // 再次确认位姿稳定
                if (checkPoseMatch(target_x, target_y, target_yaw)) {
                    ROS_INFO("Pose stable. Relocalization complete at %.0f seconds (early exit).", waited);
                    reloc_success_ = true;
                    publishRelocStatus(true);
                    return;
                } else {
                    ROS_WARN("Pose became unstable, continuing...");
                }
            }
            
            ros::Duration(check_interval).sleep();
            waited += check_interval;
            
            // 每10秒输出一次等待状态
            if (static_cast<int>(waited) % 10 == 0) {
                double cur_x, cur_y, cur_yaw;
                getCurrentPose(cur_x, cur_y, cur_yaw);
                double pos_error = std::sqrt(std::pow(cur_x - target_x, 2) + std::pow(cur_y - target_y, 2));
                double yaw_error = std::abs(normalizeAngle(cur_yaw - target_yaw));
                ROS_INFO("Status: %.0f/%.0f sec | Current: (%.2f, %.2f, %.1f deg) | Target: (%.2f, %.2f, %.1f deg) | Error: pos=%.2fm, yaw=%.1f deg",
                         waited, init_wait_time_, 
                         cur_x, cur_y, cur_yaw * 180.0 / M_PI,
                         target_x, target_y, target_yaw * 180.0 / M_PI,
                         pos_error, yaw_error * 180.0 / M_PI);
            }
        }
        
        // 超时，最后检查一次
        ROS_WARN("Timeout reached (%.0f seconds)", init_wait_time_);
        if (checkPoseMatch(target_x, target_y, target_yaw)) {
            ROS_INFO("=== RELOCALIZATION SUCCESS (at timeout) ===");
            reloc_success_ = true;
            publishRelocStatus(true);
        } else {
            double cur_x, cur_y, cur_yaw;
            getCurrentPose(cur_x, cur_y, cur_yaw);
            double pos_error = std::sqrt(std::pow(cur_x - target_x, 2) + std::pow(cur_y - target_y, 2));
            double yaw_error = std::abs(normalizeAngle(cur_yaw - target_yaw));
            ROS_ERROR("=== RELOCALIZATION FAILED ===");
            ROS_ERROR("Final error: position=%.2fm (tolerance: %.2fm), yaw=%.1f deg (tolerance: %.1f deg)",
                      pos_error, position_tolerance_, 
                      yaw_error * 180.0 / M_PI, yaw_tolerance_ * 180.0 / M_PI);
            ROS_ERROR("Please manually check relocalization or trigger it via service");
            // 即使失败，也通知pose_recovery_3d_node，让用户手动处理后可以保存
            publishRelocStatus(false);
        }
    }
    
    double normalizeAngle(double angle) {
        // 归一化角度到 [-pi, pi]
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
    void getCurrentPose(double& x, double& y, double& yaw) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        x = current_global_x_;
        y = current_global_y_;
        yaw = current_global_yaw_;
    }
    
    bool checkPoseMatch(double target_x, double target_y, double target_yaw) {
        if (!global_pose_received_) {
            ROS_DEBUG_THROTTLE(5.0, "No global pose received yet, cannot verify relocalization");
            return false;
        }
        
        double cur_x, cur_y, cur_yaw;
        getCurrentPose(cur_x, cur_y, cur_yaw);
        
        // 计算位置误差
        double pos_error = std::sqrt(std::pow(cur_x - target_x, 2) + std::pow(cur_y - target_y, 2));
        
        // 计算角度误差（归一化到[-pi, pi]）
        double yaw_error = std::abs(normalizeAngle(cur_yaw - target_yaw));
        
        ROS_DEBUG("Pose check: pos_error=%.2f (tol: %.2f), yaw_error=%.2f rad (tol: %.2f)", 
                 pos_error, position_tolerance_, yaw_error, yaw_tolerance_);
        
        return (pos_error < position_tolerance_) && (yaw_error < yaw_tolerance_);
    }
    
    bool callRelocServiceWithCheck(double x, double y, double z, double roll, double pitch, double yaw) {
        // 发布重定位位姿
        callRelocService(x, y, z, roll, pitch, yaw);
        
        // 等待并检查是否成功
        ROS_INFO("Waiting for relocalization to take effect...");
        
        for (int i = 0; i < 10 && ros::ok(); i++) {
            ros::Duration(0.5).sleep();
            if (checkPoseMatch(x, y, yaw)) {
                ROS_INFO("Relocalization verified: current pose matches target!");
                return true;
            }
        }
        
        ROS_WARN("Relocalization not verified within 5 seconds");
        return false;
    }
    
    bool loadPoseFromFile(double& x, double& y, double& z, double& roll, double& pitch, double& yaw) {
        double latest_timestamp = 0.0;
        bool found = false;
        
        ROS_INFO("[auto_reloc_3d] Searching for saved pose files...");
        ROS_INFO("[auto_reloc_3d] File path: %s", pose_file_path_.c_str());
        ROS_INFO("[auto_reloc_3d] File name pattern: %s_*.json", pose_file_name_.c_str());
        
        for (int i = 0; i < file_buffer_; i++) {
            std::string filename = pose_file_path_ + pose_file_name_ + "_" + std::to_string(i) + ".json";
            ROS_INFO("[auto_reloc_3d] Checking file: %s", filename.c_str());
            
            std::ifstream file(filename);
            
            if (!file.is_open()) {
                ROS_WARN("[auto_reloc_3d] File not found or cannot open: %s", filename.c_str());
                continue;
            }
            ROS_INFO("[auto_reloc_3d] File opened successfully: %s", filename.c_str());
            
            std::stringstream buffer;
            buffer << file.rdbuf();
            std::string content = buffer.str();
            file.close();
            
            double timestamp;
            if (parseJson(content, x, y, z, roll, pitch, yaw, timestamp)) {
                if (timestamp > latest_timestamp) {
                    latest_timestamp = timestamp;
                    found = true;
                }
            }
        }
        
        if (found) {
            // 如果yaw是度数，转换为弧度
            if (yaw_in_degrees_) {
                double yaw_deg = yaw;
                yaw = yaw * M_PI / 180.0;
                roll = roll * M_PI / 180.0;
                pitch = pitch * M_PI / 180.0;
                ROS_INFO("Converted yaw from degrees (%.2f) to radians (%.4f)", yaw_deg, yaw);
            }
            ROS_INFO("Loaded pose from file: x=%.2f, y=%.2f, z=%.2f, roll=%.4f, pitch=%.4f, yaw=%.4f (rad)",
                     x, y, z, roll, pitch, yaw);
        }
        
        return found;
    }
    
    bool parseJson(const std::string& content, double& x, double& y, double& z, 
                   double& roll, double& pitch, double& yaw, double& timestamp) {
        auto getValue = [&content](const std::string& key) -> double {
            size_t pos = content.find("\"" + key + "\"");
            if (pos == std::string::npos) return 0.0;
            pos = content.find(":", pos);
            if (pos == std::string::npos) return 0.0;
            pos++;
            while (pos < content.size() && (content[pos] == ' ' || content[pos] == '\t')) pos++;
            size_t end = pos;
            while (end < content.size() && (std::isdigit(content[end]) || content[end] == '.' || content[end] == '-' || content[end] == 'e' || content[end] == 'E' || content[end] == '+')) end++;
            return std::stod(content.substr(pos, end - pos));
        };
        
        try {
            x = getValue("x");
            y = getValue("y");
            z = getValue("z");
            roll = getValue("roll");
            pitch = getValue("pitch");
            yaw = getValue("yaw");
            timestamp = getValue("timestamp");
            return true;
        } catch (...) {
            return false;
        }
    }
    
    void callRelocService(double x, double y, double z, double roll, double pitch, double yaw) {
        ROS_INFO("Publishing relocalization pose: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f",
                 x, y, z, roll, pitch, yaw);
        
        // 等待订阅者连接
        ros::Rate wait_rate(10);
        int wait_count = 0;
        while (initialpose_pub_.getNumSubscribers() == 0 && wait_count < 50 && ros::ok()) {
            ROS_INFO_THROTTLE(1.0, "Waiting for subscribers to /initialpose...");
            wait_rate.sleep();
            wait_count++;
        }
        
        if (initialpose_pub_.getNumSubscribers() == 0) {
            ROS_WARN("No subscribers to /initialpose, publishing anyway...");
        } else {
            ROS_INFO("Found %d subscribers to /initialpose", initialpose_pub_.getNumSubscribers());
        }
        
        // 发布到 /initialpose (geometry_msgs/PoseWithCovarianceStamped)
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";
        
        pose_msg.pose.pose.position.x = x;
        pose_msg.pose.pose.position.y = y;
        pose_msg.pose.pose.position.z = z;
        
        // 欧拉角转四元数
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();
        
        // 设置协方差（对角线元素）- 与RViz格式一致
        pose_msg.pose.covariance[0] = 0.25;   // x
        pose_msg.pose.covariance[7] = 0.25;   // y
        pose_msg.pose.covariance[14] = 0.0;   // z
        pose_msg.pose.covariance[21] = 0.0;   // roll
        pose_msg.pose.covariance[28] = 0.0;   // pitch
        pose_msg.pose.covariance[35] = 0.07;  // yaw
        
        // 多次发布确保被接收
        for (int i = 0; i < 3; i++) {
            initialpose_pub_.publish(pose_msg);
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("Published to %s (3 times)", initialpose_topic_.c_str());
    }
    
    bool triggerRelocCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        double x, y, z, roll, pitch, yaw;
        if (loadPoseFromFile(x, y, z, roll, pitch, yaw)) {
            callRelocService(x, y, z, roll, pitch, yaw);
            res.success = true;
            res.message = "Relocalization triggered";
        } else {
            res.success = false;
            res.message = "No saved pose found";
        }
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_relocalization_3d_node");
    
    AutoRelocalization3D auto_reloc;
    
    ros::spin();
    
    return 0;
}
