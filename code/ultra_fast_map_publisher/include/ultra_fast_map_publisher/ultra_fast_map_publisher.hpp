#ifndef ULTRA_FAST_MAP_PUBLISHER_HPP
#define ULTRA_FAST_MAP_PUBLISHER_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <unordered_map>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <cstring>
#include <fstream>
#include <sstream>
#include <functional>
#include <iomanip>

namespace ultra_fast_map_publisher {

// ==================== 进度回调接口 ====================
class ProgressCallback {
public:
    virtual void on_progress(const std::string& phase, double percentage, 
                           size_t processed, size_t total) = 0;
    virtual void on_message(const std::string& message) = 0;
    virtual void on_phase_start(const std::string& phase) = 0;
    virtual void on_phase_end(const std::string& phase, double elapsed_seconds) = 0;
    virtual ~ProgressCallback() = default;
};

// 简单的控制台进度条
class ConsoleProgressBar : public ProgressCallback {
public:
    void on_progress(const std::string& phase, double percentage, 
                    size_t processed, size_t total) override;
    void on_message(const std::string& message) override;
    void on_phase_start(const std::string& phase) override;
    void on_phase_end(const std::string& phase, double elapsed_seconds) override;
};

// 自定义二进制格式头部
#pragma pack(push, 1)
struct FastMapHeader {
    char magic[4] = {'F', 'M', 'P', '1'};  // 'FMP1'
    uint32_t version = 1;
    uint64_t num_points = 0;
    uint64_t data_offset = sizeof(FastMapHeader);  // 数据起始位置
    float min_x = 0.0f, max_x = 0.0f;
    float min_y = 0.0f, max_y = 0.0f;
    float min_z = 0.0f, max_z = 0.0f;
    uint32_t point_size = sizeof(float) * 3;  // 每个点的大小（x, y, z）
    uint8_t compression = 0;  // 0: 无压缩, 1: zstd, 2: lz4
    uint8_t checksum_type = 0;  // 0: 无, 1: crc32
    uint32_t checksum = 0;
    uint64_t reserved[4] = {0};
    
    bool validate() const {
        return magic[0] == 'F' && magic[1] == 'M' && magic[2] == 'P' && magic[3] == '1';
    }
};
#pragma pack(pop)

class MemoryMappedFile {
public:
    MemoryMappedFile();
    ~MemoryMappedFile();
    
    bool open(const std::string& file_path);
    void close();
    
    bool is_open() const { return fd_ != -1 && data_ != MAP_FAILED; }
    size_t size() const { return size_; }
    
    template<typename T>
    const T* data_at(size_t offset = 0) const {
        return reinterpret_cast<const T*>(reinterpret_cast<const char*>(data_) + offset);
    }
    
    const void* data() const { return data_; }
    void* data() { return data_; }
    
private:
    int fd_ = -1;
    void* data_ = MAP_FAILED;
    size_t size_ = 0;
};

class ParallelProcessor {
public:
    ParallelProcessor(size_t num_threads = 0);
    ~ParallelProcessor() = default;
    
    // 并行处理点云数据
    template<typename Func>
    void parallel_for(size_t start, size_t end, Func func) {
        if (num_threads_ <= 1 || (end - start) < 10000) {
            for (size_t i = start; i < end; ++i) {
                func(i);
            }
            return;
        }
        
        std::vector<std::thread> workers;
        size_t chunk_size = (end - start + num_threads_ - 1) / num_threads_;
        
        for (size_t t = 0; t < num_threads_; ++t) {
            size_t thread_start = start + t * chunk_size;
            size_t thread_end = std::min(end, thread_start + chunk_size);
            
            if (thread_start < thread_end) {
                workers.emplace_back([=, &func]() {
                    for (size_t i = thread_start; i < thread_end; ++i) {
                        func(i);
                    }
                });
            }
        }
        
        for (auto& worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }
    
private:
    size_t num_threads_;
};

class UltraFastMapLoader {
public:
    struct CacheEntry {
        std::string original_path;
        std::string fast_path;
        std::time_t original_mtime = 0;
        std::time_t fast_mtime = 0;
    };
    
    UltraFastMapLoader();
    ~UltraFastMapLoader();
    
    // 加载点云地图
    bool load(const std::string& file_path, 
              bool use_fast_format = true,
              bool auto_convert = true);
    
    // 获取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud() const { return cloud_; }
    
    // 统计信息
    size_t get_num_points() const { return cloud_ ? cloud_->size() : 0; }
    double get_load_time() const { return load_time_; }
    bool is_loaded() const { return cloud_ && !cloud_->empty(); }
    
    // 转换为快速格式
    bool convert_to_fast_format(const std::string& input_path, 
                                const std::string& output_path,
                                bool force_reconvert = false);
    
    // 检查是否为快速格式
    static bool is_fast_format(const std::string& file_path);
    
    // 性能设置
    void set_num_threads(size_t threads) { num_threads_ = threads; }
    void set_use_memory_mapping(bool use) { use_memory_mapping_ = use; }
    void compute_bounds(pcl::PointXYZ& min_pt, pcl::PointXYZ& max_pt) const;
    
    // 进度回调
    void set_progress_callback(ProgressCallback* callback) { progress_callback_ = callback; }
    
private:
    // 内部加载方法
    bool load_fast_format(const std::string& file_path);
    bool load_pcd_format(const std::string& file_path);
    
    // 快速格式读写
    bool read_fast_format(const std::string& file_path);
    bool write_fast_format(const std::string& file_path, 
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // 工具函数
    static bool file_exists(const std::string& path);
    static bool is_file_newer(const std::string& file1, const std::string& file2);
    
    // 进度通知
    void notify_progress(const std::string& phase, double percentage, size_t processed, size_t total);
    void notify_message(const std::string& message);
    void notify_phase_start(const std::string& phase);
    void notify_phase_end(const std::string& phase, double elapsed_seconds);
    
    // 成员变量
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    double load_time_ = 0.0;
    std::string current_file_;
    
    // 配置
    size_t num_threads_ = 0;  // 0 = 自动检测
    bool use_memory_mapping_ = true;
    
    // 进度回调
    ProgressCallback* progress_callback_ = nullptr;
    
    // 缓存管理
    static std::unordered_map<std::string, CacheEntry> cache_;
    static std::mutex cache_mutex_;
};

class UltraFastMapPublisher {
public:
    UltraFastMapPublisher(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~UltraFastMapPublisher();
    
    bool initialize();
    void spin();
    
private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher cloud_pub_;
    ros::Publisher stats_pub_;
    ros::Timer publish_timer_;
    
    // 点云数据
    std::shared_ptr<UltraFastMapLoader> loader_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    std::unique_ptr<ConsoleProgressBar> progress_bar_;
    
    // 参数
    std::string map_path_;
    std::string output_topic_;
    std::string frame_id_;
    double publish_rate_;
    bool latch_mode_;
    bool use_fast_format_;
    bool auto_convert_;
    size_t num_threads_;
    
    // 状态
    bool initialized_ = false;
    std::atomic<bool> running_{true};
    
    // 回调函数
    void publish_callback(const ros::TimerEvent& e);
    void publish_pointcloud();
    void publish_statistics();
    
    // 工具函数
    bool load_parameters();
    bool load_map();
};

} // namespace ultra_fast_map_publisher

#endif // ULTRA_FAST_MAP_PUBLISHER_HPP