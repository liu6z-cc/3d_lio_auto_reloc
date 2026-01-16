#include "ultra_fast_map_publisher/ultra_fast_map_publisher.hpp"
#include <chrono>
#include <ctime>

using namespace ultra_fast_map_publisher;
using namespace std::chrono;

// ==================== 静态成员初始化 ====================

std::unordered_map<std::string, UltraFastMapLoader::CacheEntry> UltraFastMapLoader::cache_;
std::mutex UltraFastMapLoader::cache_mutex_;

// ==================== ConsoleProgressBar 实现 ====================

void ConsoleProgressBar::on_progress(const std::string& phase, double percentage, 
                                    size_t processed, size_t total) {
    int bar_width = 50;
    int pos = static_cast<int>(bar_width * percentage);
    
    // 获取当前时间
    auto now = system_clock::now();
    auto now_time_t = system_clock::to_time_t(now);
    auto now_ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << "\r" << std::put_time(std::localtime(&now_time_t), "%H:%M:%S") 
       << "." << std::setfill('0') << std::setw(3) << now_ms.count() 
       << " " << std::setw(15) << std::left << phase << ": [";
    
    for (int i = 0; i < bar_width; ++i) {
        if (i < pos) ss << "=";
        else if (i == pos) ss << ">";
        else ss << " ";
    }
    
    ss << "] " << std::setw(6) << std::right 
       << std::fixed << std::setprecision(1) << (percentage * 100.0) << "% "
       << processed << "/" << total << " points";
    
    if (percentage >= 1.0) {
        ss << " [DONE]";
    }
    
    std::cout << ss.str() << std::flush;
    
    if (percentage >= 1.0) {
        std::cout << std::endl;
    }
}

void ConsoleProgressBar::on_message(const std::string& message) {
    auto now = system_clock::now();
    auto now_time_t = system_clock::to_time_t(now);
    auto now_ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    
    std::cout << "\r" << std::put_time(std::localtime(&now_time_t), "%H:%M:%S") 
              << "." << std::setfill('0') << std::setw(3) << now_ms.count() 
              << " " << message << std::endl;
}

void ConsoleProgressBar::on_phase_start(const std::string& phase) {
    on_message("Starting: " + phase);
}

void ConsoleProgressBar::on_phase_end(const std::string& phase, double elapsed_seconds) {
    std::stringstream ss;
    ss << "Completed: " << phase << " in " 
       << std::fixed << std::setprecision(3) << elapsed_seconds << "s";
    on_message(ss.str());
}

// ==================== MemoryMappedFile 实现 ====================

MemoryMappedFile::MemoryMappedFile() : fd_(-1), data_(MAP_FAILED), size_(0) {}

MemoryMappedFile::~MemoryMappedFile() {
    close();
}

bool MemoryMappedFile::open(const std::string& file_path) {
    close();
    
    fd_ = ::open(file_path.c_str(), O_RDONLY);
    if (fd_ < 0) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return false;
    }
    
    struct stat st;
    if (fstat(fd_, &st) < 0) {
        ROS_ERROR("Failed to stat file: %s", file_path.c_str());
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    size_ = st.st_size;
    
    data_ = ::mmap(nullptr, size_, PROT_READ, MAP_PRIVATE, fd_, 0);
    if (data_ == MAP_FAILED) {
        ROS_ERROR("Failed to mmap file: %s", file_path.c_str());
        ::close(fd_);
        fd_ = -1;
        size_ = 0;
        return false;
    }
    
    return true;
}

void MemoryMappedFile::close() {
    if (data_ != MAP_FAILED) {
        ::munmap(data_, size_);
        data_ = MAP_FAILED;
    }
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
    size_ = 0;
}

// ==================== ParallelProcessor 实现 ====================

ParallelProcessor::ParallelProcessor(size_t num_threads) {
    if (num_threads == 0) {
        num_threads_ = std::thread::hardware_concurrency();
        if (num_threads_ == 0) num_threads_ = 4;
    } else {
        num_threads_ = num_threads;
    }
    ROS_INFO("Parallel processor initialized with %zu threads", num_threads_);
}

// ==================== UltraFastMapLoader 实现 ====================

UltraFastMapLoader::UltraFastMapLoader() 
    : cloud_(new pcl::PointCloud<pcl::PointXYZ>), progress_callback_(nullptr) {
    num_threads_ = std::thread::hardware_concurrency();
    if (num_threads_ == 0) num_threads_ = 4;
}

UltraFastMapLoader::~UltraFastMapLoader() {
    if (cloud_) {
        cloud_->clear();
    }
}

void UltraFastMapLoader::notify_progress(const std::string& phase, double percentage,
                                        size_t processed, size_t total) {
    if (progress_callback_) {
        progress_callback_->on_progress(phase, percentage, processed, total);
    }
}

void UltraFastMapLoader::notify_message(const std::string& message) {
    if (progress_callback_) {
        progress_callback_->on_message(message);
    }
}

void UltraFastMapLoader::notify_phase_start(const std::string& phase) {
    if (progress_callback_) {
        progress_callback_->on_phase_start(phase);
    }
}

void UltraFastMapLoader::notify_phase_end(const std::string& phase, double elapsed_seconds) {
    if (progress_callback_) {
        progress_callback_->on_phase_end(phase, elapsed_seconds);
    }
}

void UltraFastMapLoader::compute_bounds(pcl::PointXYZ& min_pt, pcl::PointXYZ& max_pt) const {
    if (cloud_->empty()) {
        min_pt = pcl::PointXYZ(0, 0, 0);
        max_pt = pcl::PointXYZ(0, 0, 0);
        return;
    }
    
    min_pt.x = max_pt.x = cloud_->points[0].x;
    min_pt.y = max_pt.y = cloud_->points[0].y;
    min_pt.z = max_pt.z = cloud_->points[0].z;
    
    for (const auto& p : cloud_->points) {
        if (p.x < min_pt.x) min_pt.x = p.x;
        if (p.x > max_pt.x) max_pt.x = p.x;
        if (p.y < min_pt.y) min_pt.y = p.y;
        if (p.y > max_pt.y) max_pt.y = p.y;
        if (p.z < min_pt.z) min_pt.z = p.z;
        if (p.z > max_pt.z) max_pt.z = p.z;
    }
}

bool UltraFastMapLoader::load(const std::string& file_path, 
                               bool use_fast_format,
                               bool auto_convert) {
    notify_phase_start("Map Loading");
    auto start_time = high_resolution_clock::now();
    
    if (!file_exists(file_path)) {
        ROS_ERROR("File does not exist: %s", file_path.c_str());
        notify_message("ERROR: File does not exist");
        return false;
    }
    
    notify_message("Checking file: " + file_path);
    
    current_file_ = file_path;
    cloud_->clear();
    
    bool success = false;
    bool is_fast = is_fast_format(file_path);
    
    if (use_fast_format && is_fast) {
        notify_message("Loading fast format map");
        success = load_fast_format(file_path);
    } else if (use_fast_format && auto_convert) {
        std::string fast_path = file_path + ".fast";
        
        notify_message("Checking cache for: " + file_path);
        
        bool need_convert = true;
        {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            auto it = cache_.find(file_path);
            if (it != cache_.end()) {
                CacheEntry& entry = it->second;
                if (file_exists(entry.fast_path) && 
                    !is_file_newer(file_path, entry.fast_path)) {
                    need_convert = false;
                    fast_path = entry.fast_path;
                    notify_message("Using cached fast format");
                }
            }
        }
        
        if (need_convert) {
            notify_phase_start("PCD to Fast Format Conversion");
            if (!convert_to_fast_format(file_path, fast_path, true)) {
                notify_message("WARN: Failed to convert, falling back to PCD");
                success = load_pcd_format(file_path);
            } else {
                success = load_fast_format(fast_path);
            }
        } else {
            success = load_fast_format(fast_path);
        }
    } else {
        notify_message("Loading PCD format map");
        success = load_pcd_format(file_path);
    }
    
    auto end_time = high_resolution_clock::now();
    load_time_ = duration_cast<duration<double>>(end_time - start_time).count();
    
    if (success) {
        std::stringstream ss;
        ss << "Successfully loaded " << cloud_->size() << " points in " 
           << std::fixed << std::setprecision(3) << load_time_ << " seconds";
        notify_message(ss.str());
        notify_phase_end("Map Loading", load_time_);
    } else {
        notify_message("ERROR: Failed to load map");
        notify_phase_end("Map Loading (Failed)", load_time_);
    }
    
    return success;
}

bool UltraFastMapLoader::load_fast_format(const std::string& file_path) {
    notify_phase_start("Loading Fast Format");
    auto start_time = high_resolution_clock::now();
    
    if (!use_memory_mapping_) {
        bool result = read_fast_format(file_path);
        auto end_time = high_resolution_clock::now();
        notify_phase_end("Loading Fast Format", 
                        duration_cast<duration<double>>(end_time - start_time).count());
        return result;
    }
    
    notify_message("Memory mapping file: " + file_path);
    
    MemoryMappedFile mmap;
    if (!mmap.open(file_path)) {
        notify_message("ERROR: Failed to memory map file");
        return false;
    }
    
    if (mmap.size() < sizeof(FastMapHeader)) {
        notify_message("ERROR: File too small to be a valid fast format");
        return false;
    }
    
    const FastMapHeader* header = mmap.data_at<FastMapHeader>(0);
    if (!header->validate()) {
        notify_message("ERROR: Invalid fast format header");
        return false;
    }
    
    size_t expected_size = header->data_offset + header->num_points * header->point_size;
    if (mmap.size() < expected_size) {
        notify_message("ERROR: File truncated");
        return false;
    }
    
    notify_message("Preparing point cloud with " + std::to_string(header->num_points) + " points");
    
    cloud_->resize(header->num_points);
    
    ParallelProcessor processor(num_threads_);
    
    const float* data_ptr = reinterpret_cast<const float*>(
        reinterpret_cast<const char*>(mmap.data()) + header->data_offset
    );
    
    const size_t batch_size = 100000;
    size_t total_batches = (header->num_points + batch_size - 1) / batch_size;
    
    for (size_t batch = 0; batch < total_batches; ++batch) {
        size_t start_idx = batch * batch_size;
        size_t end_idx = std::min(start_idx + batch_size, header->num_points);
        size_t batch_points = end_idx - start_idx;
        
        processor.parallel_for(0, batch_points, [&](size_t i) {
            size_t point_idx = start_idx + i;
            const float* point_data = data_ptr + point_idx * 3;
            auto& point = cloud_->points[point_idx];
            point.x = point_data[0];
            point.y = point_data[1];
            point.z = point_data[2];
        });
        
        double percentage = static_cast<double>(end_idx) / header->num_points;
        notify_progress("Loading Fast Format", percentage, end_idx, header->num_points);
    }
    
    cloud_->width = cloud_->size();
    cloud_->height = 1;
    cloud_->is_dense = true;
    
    auto end_time = high_resolution_clock::now();
    double elapsed = duration_cast<duration<double>>(end_time - start_time).count();
    
    notify_phase_end("Loading Fast Format", elapsed);
    return true;
}

bool UltraFastMapLoader::load_pcd_format(const std::string& file_path) {
    notify_phase_start("Loading PCD Format");
    auto start_time = high_resolution_clock::now();
    
    notify_message("Opening PCD file: " + file_path);
    
    std::ifstream in(file_path, std::ios::binary);
    if (!in) {
        notify_message("ERROR: Failed to open PCD file");
        return false;
    }
    
    std::string line;
    size_t data_start = 0;
    bool binary = false;
    size_t num_points = 0;
    int width = 0, height = 0;
    
    notify_message("Parsing PCD header...");
    
    while (std::getline(in, line)) {
        data_start += line.size() + 1;
        
        if (line.find("WIDTH") != std::string::npos) {
            sscanf(line.c_str(), "WIDTH %d", &width);
        } else if (line.find("HEIGHT") != std::string::npos) {
            sscanf(line.c_str(), "HEIGHT %d", &height);
        } else if (line.find("POINTS") != std::string::npos) {
            sscanf(line.c_str(), "POINTS %zu", &num_points);
        } else if (line.find("DATA binary") != std::string::npos) {
            binary = true;
            data_start += 1;
            break;
        }
    }
    
    if (!binary) {
        notify_message("ERROR: Only binary PCD format supported");
        return false;
    }
    
    if (num_points == 0) {
        notify_message("ERROR: Invalid point count");
        return false;
    }
    
    std::stringstream info_ss;
    info_ss << "PCD Info: " << width << "x" << height << " = " 
            << num_points << " points, data offset: " << data_start << " bytes";
    notify_message(info_ss.str());
    
    in.seekg(0, std::ios::end);
    size_t file_size = in.tellg();
    size_t data_size = file_size - data_start;
    
    size_t expected_size = num_points * sizeof(float) * 4;
    if (data_size != expected_size) {
        std::stringstream warn_ss;
        warn_ss << "WARN: Data size mismatch: expected " << expected_size 
                << ", got " << data_size;
        notify_message(warn_ss.str());
    }
    
    in.close();
    notify_message("Closing file, reopening with memory mapping...");
    
    MemoryMappedFile mmap;
    if (!mmap.open(file_path)) {
        notify_message("ERROR: Failed to memory map file");
        return false;
    }
    
    notify_message("Memory mapping successful, processing data...");
    
    cloud_->resize(num_points);
    
    const char* file_data = static_cast<const char*>(mmap.data());
    const float* point_data = reinterpret_cast<const float*>(file_data + data_start);
    
    const size_t batch_size = 100000;
    size_t total_batches = (num_points + batch_size - 1) / batch_size;
    
    ParallelProcessor processor(num_threads_);
    
    for (size_t batch = 0; batch < total_batches; ++batch) {
        size_t start_idx = batch * batch_size;
        size_t end_idx = std::min(start_idx + batch_size, num_points);
        size_t batch_points = end_idx - start_idx;
        
        processor.parallel_for(0, batch_points, [&](size_t i) {
            size_t point_idx = start_idx + i;
            const float* p = point_data + point_idx * 4;
            
            auto& point = cloud_->points[point_idx];
            point.x = p[0];
            point.y = p[1];
            point.z = p[2];
        });
        
        double percentage = static_cast<double>(end_idx) / num_points;
        notify_progress("Loading PCD", percentage, end_idx, num_points);
    }
    
    cloud_->width = cloud_->size();
    cloud_->height = 1;
    cloud_->is_dense = true;
    
    auto end_time = high_resolution_clock::now();
    double elapsed = duration_cast<duration<double>>(end_time - start_time).count();
    
    std::stringstream done_ss;
    done_ss << "Loaded " << cloud_->size() << " points in " 
            << std::fixed << std::setprecision(3) << elapsed << " seconds";
    notify_message(done_ss.str());
    
    notify_phase_end("Loading PCD Format", elapsed);
    return !cloud_->empty();
}

bool UltraFastMapLoader::convert_to_fast_format(const std::string& input_path,
                                                const std::string& output_path,
                                                bool force_reconvert) {
    notify_phase_start("Converting to Fast Format");
    auto start_time = high_resolution_clock::now();
    
    if (!force_reconvert && file_exists(output_path)) {
        if (!is_file_newer(input_path, output_path)) {
            notify_message("Fast format is up to date");
            
            std::lock_guard<std::mutex> lock(cache_mutex_);
            struct stat st_input, st_output;
            stat(input_path.c_str(), &st_input);
            stat(output_path.c_str(), &st_output);
            
            cache_[input_path] = {
                input_path,
                output_path,
                static_cast<std::time_t>(st_input.st_mtime),
                static_cast<std::time_t>(st_output.st_mtime)
            };
            
            notify_phase_end("Converting to Fast Format", 0);
            return true;
        }
    }
    
    notify_message("Starting conversion: " + input_path + " -> " + output_path);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    notify_message("Loading source PCD file...");
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_path, *temp_cloud) == -1) {
        notify_message("ERROR: Failed to load PCD for conversion");
        return false;
    }
    
    notify_message("Source loaded: " + std::to_string(temp_cloud->size()) + " points");
    
    FastMapHeader header;
    header.num_points = temp_cloud->size();
    
    if (temp_cloud->empty()) {
        notify_message("ERROR: Empty point cloud");
        return false;
    }
    
    pcl::PointXYZ min_pt, max_pt;
    compute_bounds(min_pt, max_pt);
    
    header.min_x = min_pt.x; header.max_x = max_pt.x;
    header.min_y = min_pt.y; header.max_y = max_pt.y;
    header.min_z = min_pt.z; header.max_z = max_pt.z;
    
    notify_message("Bounds computed");
    
    std::ofstream out(output_path, std::ios::binary);
    if (!out) {
        notify_message("ERROR: Failed to create output file");
        return false;
    }
    
    notify_message("Writing header...");
    
    out.write(reinterpret_cast<const char*>(&header), sizeof(header));
    
    const size_t batch_size = 100000;
    size_t total_points = temp_cloud->size();
    size_t total_batches = (total_points + batch_size - 1) / batch_size;
    
    notify_message("Writing point data in batches...");
    
    for (size_t batch = 0; batch < total_batches; ++batch) {
        size_t start_idx = batch * batch_size;
        size_t end_idx = std::min(start_idx + batch_size, total_points);
        
        for (size_t i = start_idx; i < end_idx; ++i) {
            const auto& p = temp_cloud->points[i];
            float data[3] = {p.x, p.y, p.z};
            out.write(reinterpret_cast<const char*>(data), sizeof(data));
        }
        
        double percentage = static_cast<double>(end_idx) / total_points;
        notify_progress("Converting", percentage, end_idx, total_points);
    }
    
    out.close();
    
    auto end_time = high_resolution_clock::now();
    double convert_time = duration_cast<duration<double>>(end_time - start_time).count();
    
    std::stringstream ss;
    ss << "Conversion completed in " << std::fixed << std::setprecision(3) 
       << convert_time << " seconds, output: " << output_path;
    notify_message(ss.str());
    
    std::lock_guard<std::mutex> lock(cache_mutex_);
    struct stat st_input, st_output;
    stat(input_path.c_str(), &st_input);
    stat(output_path.c_str(), &st_output);
    
    cache_[input_path] = {
        input_path,
        output_path,
        static_cast<std::time_t>(st_input.st_mtime),
        static_cast<std::time_t>(st_output.st_mtime)
    };
    
    notify_phase_end("Converting to Fast Format", convert_time);
    return true;
}

bool UltraFastMapLoader::is_fast_format(const std::string& file_path) {
    std::ifstream in(file_path, std::ios::binary);
    if (!in) return false;
    
    FastMapHeader header;
    in.read(reinterpret_cast<char*>(&header), sizeof(header));
    
    if (!in) return false;
    
    return header.validate();
}

bool UltraFastMapLoader::read_fast_format(const std::string& file_path) {
    notify_phase_start("Reading Fast Format (Stream)");
    auto start_time = high_resolution_clock::now();
    
    std::ifstream in(file_path, std::ios::binary);
    if (!in) {
        notify_message("ERROR: Failed to open file");
        return false;
    }
    
    FastMapHeader header;
    in.read(reinterpret_cast<char*>(&header), sizeof(header));
    
    if (!in || !header.validate()) {
        notify_message("ERROR: Invalid fast format file");
        return false;
    }
    
    notify_message("File validated, points: " + std::to_string(header.num_points));
    
    cloud_->resize(header.num_points);
    
    in.seekg(header.data_offset);
    
    notify_message("Reading point data...");
    
    const size_t batch_size = 100000;
    size_t total_batches = (header.num_points + batch_size - 1) / batch_size;
    
    ParallelProcessor processor(num_threads_);
    
    for (size_t batch = 0; batch < total_batches; ++batch) {
        size_t start_idx = batch * batch_size;
        size_t end_idx = std::min(start_idx + batch_size, header.num_points);
        size_t batch_points = end_idx - start_idx;
        
        std::vector<float> buffer(batch_points * 3);
        in.read(reinterpret_cast<char*>(buffer.data()), 
                batch_points * sizeof(float) * 3);
        
        if (!in) {
            notify_message("ERROR: Failed to read point data");
            return false;
        }
        
        processor.parallel_for(0, batch_points, [&](size_t i) {
            size_t point_idx = start_idx + i;
            auto& point = cloud_->points[point_idx];
            point.x = buffer[i * 3 + 0];
            point.y = buffer[i * 3 + 1];
            point.z = buffer[i * 3 + 2];
        });
        
        double percentage = static_cast<double>(end_idx) / header.num_points;
        notify_progress("Reading", percentage, end_idx, header.num_points);
    }
    
    cloud_->width = cloud_->size();
    cloud_->height = 1;
    cloud_->is_dense = true;
    
    auto end_time = high_resolution_clock::now();
    double elapsed = duration_cast<duration<double>>(end_time - start_time).count();
    
    notify_phase_end("Reading Fast Format (Stream)", elapsed);
    return true;
}

bool UltraFastMapLoader::write_fast_format(const std::string& file_path,
                                          const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::ofstream out(file_path, std::ios::binary);
    if (!out) {
        ROS_ERROR("Failed to create file: %s", file_path.c_str());
        return false;
    }
    
    FastMapHeader header;
    header.num_points = cloud->size();
    
    if (!cloud->empty()) {
        pcl::PointXYZ min_pt, max_pt;
        compute_bounds(min_pt, max_pt);
        
        header.min_x = min_pt.x; header.max_x = max_pt.x;
        header.min_y = min_pt.y; header.max_y = max_pt.y;
        header.min_z = min_pt.z; header.max_z = max_pt.z;
    }
    
    out.write(reinterpret_cast<const char*>(&header), sizeof(header));
    
    for (const auto& p : *cloud) {
        float data[3] = {p.x, p.y, p.z};
        out.write(reinterpret_cast<const char*>(data), sizeof(data));
    }
    
    out.close();
    return out.good();
}

bool UltraFastMapLoader::file_exists(const std::string& path) {
    std::ifstream f(path);
    return f.good();
}

bool UltraFastMapLoader::is_file_newer(const std::string& file1, const std::string& file2) {
    struct stat st1, st2;
    if (stat(file1.c_str(), &st1) != 0) return true;
    if (stat(file2.c_str(), &st2) != 0) return true;
    
    return st1.st_mtime > st2.st_mtime;
}

// ==================== UltraFastMapPublisher 实现 ====================

UltraFastMapPublisher::UltraFastMapPublisher(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), 
      loader_(new UltraFastMapLoader()),
      progress_bar_(new ConsoleProgressBar()) {
}

UltraFastMapPublisher::~UltraFastMapPublisher() {
    running_ = false;
}

bool UltraFastMapPublisher::load_parameters() {
    if (!private_nh_.getParam("map_path", map_path_)) {
        ROS_ERROR("Parameter 'map_path' is required");
        return false;
    }
    
    private_nh_.param("output_topic", output_topic_, std::string("/map_ultra_fast"));
    private_nh_.param("frame_id", frame_id_, std::string("map"));
    private_nh_.param("publish_rate", publish_rate_, 1.0);
    private_nh_.param("latch_mode", latch_mode_, true);
    private_nh_.param("use_fast_format", use_fast_format_, true);
    private_nh_.param("auto_convert", auto_convert_, true);
    
    int num_threads_temp = 0;
    private_nh_.param("num_threads", num_threads_temp, 0);
    num_threads_ = static_cast<size_t>(num_threads_temp);
    
    ROS_INFO("Parameters loaded:");
    ROS_INFO("  map_path: %s", map_path_.c_str());
    ROS_INFO("  output_topic: %s", output_topic_.c_str());
    ROS_INFO("  frame_id: %s", frame_id_.c_str());
    ROS_INFO("  publish_rate: %.1f Hz", publish_rate_);
    ROS_INFO("  latch_mode: %s", latch_mode_ ? "true" : "false");
    ROS_INFO("  use_fast_format: %s", use_fast_format_ ? "true" : "false");
    ROS_INFO("  auto_convert: %s", auto_convert_ ? "true" : "false");
    ROS_INFO("  num_threads: %zu", num_threads_);
    
    return true;
}

bool UltraFastMapPublisher::load_map() {
    ROS_INFO("Loading map...");
    
    loader_->set_progress_callback(progress_bar_.get());
    
    if (num_threads_ > 0) {
        loader_->set_num_threads(num_threads_);
    }
    
    if (!loader_->load(map_path_, use_fast_format_, auto_convert_)) {
        ROS_ERROR("Failed to load map");
        return false;
    }
    
    cloud_ = loader_->get_cloud();
    if (!cloud_ || cloud_->empty()) {
        ROS_ERROR("Loaded empty point cloud");
        return false;
    }
    
    ROS_INFO("Map loaded successfully: %zu points", cloud_->size());
    return true;
}

bool UltraFastMapPublisher::initialize() {
    if (!load_parameters()) {
        return false;
    }
    
    if (!load_map()) {
        return false;
    }
    
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        output_topic_, 1, latch_mode_
    );
    
    stats_pub_ = nh_.advertise<std_msgs::String>("/map_stats", 1, true);
    
    if (publish_rate_ > 0 && !latch_mode_) {
        publish_timer_ = nh_.createTimer(
            ros::Duration(1.0 / publish_rate_),
            &UltraFastMapPublisher::publish_callback, this
        );
    } else {
        publish_pointcloud();
    }
    
    publish_statistics();
    
    initialized_ = true;
    return true;
}

void UltraFastMapPublisher::publish_callback(const ros::TimerEvent& e) {
    if (!initialized_ || !running_) return;
    publish_pointcloud();
}

void UltraFastMapPublisher::publish_pointcloud() {
    if (!cloud_ || cloud_->empty()) {
        ROS_WARN_THROTTLE(5.0, "No point cloud data to publish");
        return;
    }
    
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_, msg);
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    
    cloud_pub_.publish(msg);
    
    ROS_DEBUG_THROTTLE(5.0, "Published point cloud with %zu points", cloud_->size());
}

void UltraFastMapPublisher::publish_statistics() {
    std_msgs::String stats_msg;
    std::stringstream ss;
    
    ss << "Map Statistics:\n";
    ss << "  Points: " << cloud_->size() << "\n";
    ss << "  Load time: " << loader_->get_load_time() << " seconds\n";
    ss << "  Frame: " << frame_id_ << "\n";
    ss << "  Topic: " << output_topic_ << "\n";
    ss << "  Latch mode: " << (latch_mode_ ? "true" : "false") << "\n";
    
    if (!cloud_->empty()) {
        pcl::PointXYZ min_pt, max_pt;
        loader_->compute_bounds(min_pt, max_pt);
        
        ss << "  Bounds:\n";
        ss << "    X: [" << min_pt.x << ", " << max_pt.x << "]\n";
        ss << "    Y: [" << min_pt.y << ", " << max_pt.y << "]\n";
        ss << "    Z: [" << min_pt.z << ", " << max_pt.z << "]\n";
        ss << "    Size: " << (max_pt.x - min_pt.x) << " x " 
           << (max_pt.y - min_pt.y) << " x " 
           << (max_pt.z - min_pt.z) << "\n";
    }
    
    stats_msg.data = ss.str();
    stats_pub_.publish(stats_msg);
}

void UltraFastMapPublisher::spin() {
    if (!initialized_) {
        ROS_ERROR("Publisher not initialized");
        return;
    }
    
    ROS_INFO("Ultra Fast Map Publisher started");
    ROS_INFO("Publishing map on topic: %s", output_topic_.c_str());
    
    if (latch_mode_) {
        ROS_INFO("Running in latch mode, map will be available to new subscribers");
    } else {
        ROS_INFO("Publishing at %.1f Hz", publish_rate_);
    }
    
    ros::spin();
}

// ==================== main函数 ====================

int main(int argc, char** argv) {
    ros::init(argc, argv, "ultra_fast_map_publisher");
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    UltraFastMapPublisher publisher(nh, private_nh);
    
    if (!publisher.initialize()) {
        ROS_ERROR("Failed to initialize Ultra Fast Map Publisher");
        return 1;
    }
    
    publisher.spin();
    
    return 0;
}