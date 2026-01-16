/**
 * @file pose_recovery_3d_node.cpp
 * @brief 3D位姿恢复节点
 * 
 * 该节点用于记录机器人在地图坐标系下的位姿（包含x, y, z, roll, pitch, yaw），
 * 并在系统重启时自动恢复到上一次保存的位姿。
 * 
 * 订阅话题：
 *   - /arc_pose_tool/base_pose2d (geometry_msgs/Pose2D): 全局坐标位姿
 *   - /pose_recovery_3d/reloc_status (std_msgs/Bool): 重定位状态
 * 
 * 工作流程：
 *   1. 启动后等待重定位完成（订阅reloc_status话题）
 *   2. 重定位成功后才开始保存位姿
 *   3. 如果没有历史位姿文件，直接开始保存（首次运行）
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <cmath>
#include <mutex>

// JSON简易解析（内联实现）
#include <sstream>
#include <iomanip>

class PoseRecovery3D {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    ros::Subscriber pose_sub_;
    ros::Subscriber reloc_status_sub_;
    ros::Timer save_timer_;
    
    // 坐标系参数
    std::string map_frame_id_;
    std::string base_frame_id_;
    
    // 文件参数
    std::string file_name_;
    std::string file_path_;
    int file_buffer_;
    int current_buffer_index_;
    
    // 保存参数
    double period_;
    
    // 话题参数
    std::string pose_topic_;
    
    // 等待重定位参数
    bool wait_for_reloc_;          // 是否等待重定位完成
    bool reloc_completed_;         // 重定位是否完成
    bool has_saved_pose_;          // 是否有历史保存的位姿
    
    // 协方差参数（用于重定位时的不确定性）
    double covariance_x_;
    double covariance_y_;
    double covariance_z_;
    double covariance_roll_;
    double covariance_pitch_;
    double covariance_yaw_;
    
    // 当前位姿
    double current_x_;
    double current_y_;
    double current_z_;
    double current_roll_;
    double current_pitch_;
    double current_yaw_;
    
    bool pose_received_;
    std::mutex pose_mutex_;

public:
    PoseRecovery3D() : private_nh_("~"), pose_received_(false), current_buffer_index_(0),
                       reloc_completed_(false), has_saved_pose_(false) {
        // 读取参数
        private_nh_.param<std::string>("map_frame_id", map_frame_id_, "map");
        private_nh_.param<std::string>("base_frame_id", base_frame_id_, "body");
        private_nh_.param<std::string>("file_name", file_name_, "robot_pose_3d");
        private_nh_.param<std::string>("file_path", file_path_, "");
        private_nh_.param<int>("file_buffer", file_buffer_, 2);
        private_nh_.param<double>("period", period_, 0.5);
        private_nh_.param<std::string>("pose_topic", pose_topic_, "/arc_pose_tool/base_pose2d");
        private_nh_.param<bool>("wait_for_reloc", wait_for_reloc_, true);
        
        private_nh_.param<double>("covariance_x", covariance_x_, 0.1);
        private_nh_.param<double>("covariance_y", covariance_y_, 0.1);
        private_nh_.param<double>("covariance_z", covariance_z_, 0.1);
        private_nh_.param<double>("covariance_roll", covariance_roll_, 0.1);
        private_nh_.param<double>("covariance_pitch", covariance_pitch_, 0.1);
        private_nh_.param<double>("covariance_yaw", covariance_yaw_, 0.2);
        
        // 如果没有指定路径，使用当前目录
        if (file_path_.empty()) {
            file_path_ =  "/home/ch/catkin_ws/src/pose_recovery_3d/param/";
        }
        
        // 初始化位姿
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_z_ = 0.0;
        current_roll_ = 0.0;
        current_pitch_ = 0.0;
        current_yaw_ = 0.0;
        
        // 尝试加载之前保存的位姿
        has_saved_pose_ = loadPose();
        
        // 如果没有历史位姿，不需要等待重定位，直接开始保存
        if (!has_saved_pose_) {
            ROS_INFO("[pose_recovery_3d] No saved pose found, will start saving immediately (first run)");
            reloc_completed_ = true;
        }
        
        // 订阅全局坐标位姿（Pose2D: x, y, theta）
        pose_sub_ = nh_.subscribe(pose_topic_, 1, &PoseRecovery3D::poseCallback, this);
        
        // 订阅重定位状态话题
        if (wait_for_reloc_ && has_saved_pose_) {
            reloc_status_sub_ = nh_.subscribe("/pose_recovery_3d/reloc_status", 1, 
                                               &PoseRecovery3D::relocStatusCallback, this);
            ROS_INFO("[pose_recovery_3d] Waiting for relocalization to complete before saving...");
        }
        
        // 定时保存位姿
        save_timer_ = nh_.createTimer(ros::Duration(period_), &PoseRecovery3D::saveTimerCallback, this);
        
        ROS_INFO("========================================");
        ROS_INFO("[pose_recovery_3d] Node Started");
        ROS_INFO("[pose_recovery_3d] Subscribing to: %s (global pose)", pose_topic_.c_str());
        ROS_INFO("[pose_recovery_3d] Save period: %.2f seconds", period_);
        ROS_INFO("[pose_recovery_3d] File path: %s", file_path_.c_str());
        ROS_INFO("[pose_recovery_3d] File buffer count: %d", file_buffer_);
        ROS_INFO("[pose_recovery_3d] Wait for reloc: %s", (wait_for_reloc_ && has_saved_pose_) ? "YES" : "NO");
        ROS_INFO("========================================");
    }
    
    void relocStatusCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data && !reloc_completed_) {
            reloc_completed_ = true;
            ROS_INFO("[pose_recovery_3d] Relocalization completed! Starting pose saving...");
        }
    }
    
    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        // 第一次收到数据时打印日志
        if (!pose_received_) {
            ROS_INFO("[pose_recovery_3d] First global pose received! Starting pose saving...");
        }
        
        // 提取2D位姿（Pose2D只有x, y, theta）
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = 0.0;  // 2D位姿没有z
        
        // theta就是yaw角
        current_roll_ = 0.0;
        current_pitch_ = 0.0;
        current_yaw_ = msg->theta;
        
        pose_received_ = true;
    }
    
    void saveTimerCallback(const ros::TimerEvent& event) {
        // 如果还没收到位姿数据，不保存
        if (!pose_received_) {
            return;
        }
        
        // 如果需要等待重定位且重定位未完成，不保存
        if (!reloc_completed_) {
            ROS_INFO_THROTTLE(10.0, "[pose_recovery_3d] Waiting for relocalization before saving...");
            return;
        }
        
        savePose();
    }
    
    void savePose() {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        // 使用缓冲区交替写入文件
        std::string filename = file_path_ + file_name_ + "_" + std::to_string(current_buffer_index_) + ".json";
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            ROS_WARN("Failed to open file for writing: %s", filename.c_str());
            return;
        }
        
        // 写入JSON格式
        file << std::fixed << std::setprecision(6);
        file << "{\n";
        file << "    \"x\": " << current_x_ << ",\n";
        file << "    \"y\": " << current_y_ << ",\n";
        file << "    \"z\": " << current_z_ << ",\n";
        file << "    \"roll\": " << current_roll_ << ",\n";
        file << "    \"pitch\": " << current_pitch_ << ",\n";
        file << "    \"yaw\": " << current_yaw_ << ",\n";
        file << "    \"covariance_x\": " << covariance_x_ << ",\n";
        file << "    \"covariance_y\": " << covariance_y_ << ",\n";
        file << "    \"covariance_z\": " << covariance_z_ << ",\n";
        file << "    \"covariance_roll\": " << covariance_roll_ << ",\n";
        file << "    \"covariance_pitch\": " << covariance_pitch_ << ",\n";
        file << "    \"covariance_yaw\": " << covariance_yaw_ << ",\n";
        file << "    \"timestamp\": " << ros::Time::now().toSec() << "\n";
        file << "}\n";
        
        file.close();
        
        // 切换缓冲区
        current_buffer_index_ = (current_buffer_index_ + 1) % file_buffer_;
        
        // 使用INFO级别打印，方便调试（可改回DEBUG）
        ROS_INFO_THROTTLE(10.0, "[pose_recovery_3d] Pose saved to %s: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
                  filename.c_str(), current_x_, current_y_, current_z_, current_yaw_);
    }
    
    bool loadPose() {
        // 尝试加载最近保存的位姿文件
        double latest_timestamp = 0.0;
        bool found = false;
        
        ROS_INFO("[pose_recovery_3d] Searching for saved pose files...");
        
        for (int i = 0; i < file_buffer_; i++) {
            std::string filename = file_path_ + file_name_ + "_" + std::to_string(i) + ".json";
            ROS_INFO("[pose_recovery_3d] Checking file: %s", filename.c_str());
            
            std::ifstream file(filename);
            
            if (!file.is_open()) {
                ROS_WARN("[pose_recovery_3d] File not found or cannot open: %s", filename.c_str());
                continue;
            }
            ROS_INFO("[pose_recovery_3d] File opened successfully: %s", filename.c_str());
            
            std::stringstream buffer;
            buffer << file.rdbuf();
            std::string content = buffer.str();
            file.close();
            
            // 简单解析JSON
            double x, y, z, roll, pitch, yaw, timestamp;
            if (parseJson(content, x, y, z, roll, pitch, yaw, timestamp)) {
                if (timestamp > latest_timestamp) {
                    latest_timestamp = timestamp;
                    current_x_ = x;
                    current_y_ = y;
                    current_z_ = z;
                    current_roll_ = roll;
                    current_pitch_ = pitch;
                    current_yaw_ = yaw;
                    found = true;
                }
            }
        }
        
        if (found) {
            ROS_INFO("Loaded previous pose: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f",
                     current_x_, current_y_, current_z_, current_roll_, current_pitch_, current_yaw_);
        } else {
            ROS_WARN("No previous pose found, using default (0, 0, 0)");
        }
        
        return found;
    }
    
    bool parseJson(const std::string& content, double& x, double& y, double& z, 
                   double& roll, double& pitch, double& yaw, double& timestamp) {
        // 简单的JSON解析
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
    
    // 获取当前位姿（供外部调用重定位服务时使用）
    void getCurrentPose(double& x, double& y, double& z, double& roll, double& pitch, double& yaw) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        x = current_x_;
        y = current_y_;
        z = current_z_;
        roll = current_roll_;
        pitch = current_pitch_;
        yaw = current_yaw_;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_recovery_3d_node");
    
    PoseRecovery3D pose_recovery;
    
    ros::spin();
    
    return 0;
}
