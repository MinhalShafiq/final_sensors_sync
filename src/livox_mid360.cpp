#include "livox_mid360.h"
#include <iostream>
#include <chrono>
#include <cstring>
#include <cmath>

LivoxMid360::LivoxMid360()
    : lidar_handle(0), initialized(false), is_running(false),
      accumulation_start_time(0.0), current_frame_id(0),
      enable_background_subtraction(false), voxel_size(0.15f),
      integration_time(0.2) {  // Default: 200ms for good density
}

LivoxMid360::~LivoxMid360() {
    cleanup();
}

// Static callback wrappers
void LivoxMid360::static_point_cloud_callback(uint32_t handle, const uint8_t dev_type,
                                              LivoxLidarEthernetPacket* data, void* client_data) {
    if (client_data) {
        LivoxMid360* livox = static_cast<LivoxMid360*>(client_data);
        livox->handle_point_cloud(handle, dev_type, data);
    }
}

void LivoxMid360::static_lidar_info_callback(const uint32_t handle, const LivoxLidarInfo* info,
                                            void* client_data) {
    if (client_data) {
        LivoxMid360* livox = static_cast<LivoxMid360*>(client_data);
        livox->handle_lidar_info(handle, info);
    }
}

void LivoxMid360::static_work_mode_callback(livox_status status, uint32_t handle,
                                           LivoxLidarAsyncControlResponse *response,
                                           void *client_data) {
    if (client_data) {
        LivoxMid360* livox = static_cast<LivoxMid360*>(client_data);
        livox->handle_work_mode(status, handle, response);
    }
}

// Instance callback handlers
void LivoxMid360::handle_point_cloud(uint32_t handle, const uint8_t dev_type,
                                     LivoxLidarEthernetPacket* data) {
    if (!data || !is_running) {
        return;
    }

    // Parse Cartesian High-Resolution data
    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        std::lock_guard<std::mutex> lock(frame_mutex);

        LivoxLidarCartesianHighRawPoint* raw_points =
            reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(data->data);
        uint32_t point_num = data->dot_num;

        // Simply accumulate ALL points
        for (uint32_t i = 0; i < point_num; i++) {
            LivoxPoint point;
            point.x = raw_points[i].x / 1000.0f;  // mm to m
            point.y = raw_points[i].y / 1000.0f;
            point.z = raw_points[i].z / 1000.0f;
            point.intensity = raw_points[i].reflectivity;
            point.tag = raw_points[i].tag;

            accumulation_buffer.push_back(point);
        }
    }
}

void LivoxMid360::handle_lidar_info(const uint32_t handle, const LivoxLidarInfo* info) {
    if (!info) {
        std::cerr << "Livox Mid360: Info change callback failed" << std::endl;
        return;
    }

    lidar_handle = handle;
    std::cout << "Livox Mid360: Connected (Handle: " << handle
              << ", SN: " << info->sn << ")" << std::endl;

    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, static_work_mode_callback, this);
}

void LivoxMid360::handle_work_mode(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response) {
    if (!response) {
        return;
    }

    if (status == kLivoxLidarStatusSuccess && response->ret_code == 0) {
        is_running = true;
        auto now = std::chrono::steady_clock::now();
        accumulation_start_time = std::chrono::duration<double>(now.time_since_epoch()).count();
        std::cout << "Livox Mid360: Started (integration_time: " << integration_time << "s)" << std::endl;
    } else {
        std::cerr << "Livox Mid360: Failed to start" << std::endl;
    }
}

bool LivoxMid360::initialize(const std::string& config_path) {
    if (initialized) {
        return true;
    }

    config_file_path = config_path;
    std::cout << "Livox Mid360: Initializing with config: " << config_path << std::endl;

    if (!LivoxLidarSdkInit(config_path.c_str())) {
        std::cerr << "Livox Mid360: SDK initialization failed!" << std::endl;
        return false;
    }

    SetLivoxLidarPointCloudCallBack(static_point_cloud_callback, this);
    SetLivoxLidarInfoChangeCallback(static_lidar_info_callback, this);

    initialized = true;

    // Wait for connection
    int wait_count = 0;
    while (!is_running && wait_count < 50) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_count++;
    }

    if (!is_running) {
        std::cerr << "Livox Mid360: Connection timeout" << std::endl;
        cleanup();
        return false;
    }

    std::cout << "Livox Mid360: Ready" << std::endl;
    return true;
}

LivoxMid360::VoxelKey LivoxMid360::point_to_voxel(const LivoxPoint& point) const {
    VoxelKey key;
    key.x = static_cast<int>(std::floor(point.x / voxel_size));
    key.y = static_cast<int>(std::floor(point.y / voxel_size));
    key.z = static_cast<int>(std::floor(point.z / voxel_size));
    return key;
}

std::vector<LivoxPoint> LivoxMid360::filter_dynamic_points(const std::vector<LivoxPoint>& points) {
    if (background_model.empty()) {
        return points;  // First frame, return all
    }

    std::vector<LivoxPoint> static_points;
    int dynamic_filtered = 0;

    for (const auto& point : points) {
        VoxelKey key = point_to_voxel(point);

        // Check if this voxel exists in background model
        if (background_model.find(key) != background_model.end()) {
            static_points.push_back(point);
        } else {
            dynamic_filtered++;
        }
    }

    if (dynamic_filtered > 0) {
        std::cout << "Livox Mid360: Filtered " << dynamic_filtered
                  << " dynamic points, kept " << static_points.size() << " static" << std::endl;
    }

    return static_points;
}

void LivoxMid360::update_background_model(const std::vector<LivoxPoint>& static_points) {
    // Simple approach: replace background with latest static points
    background_model.clear();

    for (const auto& point : static_points) {
        VoxelKey key = point_to_voxel(point);
        background_model[key].push_back(point);
    }
}

bool LivoxMid360::get_point_cloud(LivoxPointCloud& cloud) {
    if (!initialized || !is_running) {
        return false;
    }

    std::lock_guard<std::mutex> lock(frame_mutex);

    auto now = std::chrono::steady_clock::now();
    double current_time = std::chrono::duration<double>(now.time_since_epoch()).count();
    double elapsed = current_time - accumulation_start_time;

    // Check if integration time has elapsed
    if (elapsed < integration_time) {
        return false;  // Keep accumulating
    }

    if (accumulation_buffer.empty()) {
        return false;
    }

    std::cout << "Livox Mid360: Frame complete - " << accumulation_buffer.size()
              << " points accumulated over " << elapsed << "s" << std::endl;

    // Apply background subtraction if enabled
    std::vector<LivoxPoint> output_points;
    if (enable_background_subtraction) {
        output_points = filter_dynamic_points(accumulation_buffer);
        update_background_model(output_points);
    } else {
        output_points = accumulation_buffer;
    }

    // Return frame
    cloud.points = output_points;
    cloud.timestamp = current_time;
    cloud.frame_id = current_frame_id++;
    cloud.valid = true;

    // Reset for next frame
    accumulation_buffer.clear();
    accumulation_start_time = current_time;

    return true;
}

bool LivoxMid360::get_latest_frame(LivoxMid360Frame& frame) {
    LivoxPointCloud cloud;
    if (get_point_cloud(cloud)) {
        frame.timestamp = cloud.timestamp;
        frame.cloud_data = cloud;
        frame.frame_id = cloud.frame_id;
        frame.valid = true;
        return true;
    }
    return false;
}

void LivoxMid360::cleanup() {
    if (initialized) {
        std::cout << "Livox Mid360: Cleaning up..." << std::endl;
        is_running = false;

        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            accumulation_buffer.clear();
            background_model.clear();
        }

        LivoxLidarSdkUninit();
        initialized = false;
        std::cout << "Livox Mid360: Cleanup complete" << std::endl;
    }
}

void LivoxMid360::set_integration_time(double seconds) {
    std::lock_guard<std::mutex> lock(frame_mutex);
    integration_time = seconds;
    std::cout << "Livox Mid360: Integration time set to " << seconds << "s" << std::endl;
}

void LivoxMid360::set_background_subtraction(bool enable, float voxel_size_param) {
    std::lock_guard<std::mutex> lock(frame_mutex);
    enable_background_subtraction = enable;
    voxel_size = voxel_size_param;

    if (enable) {
        std::cout << "Livox Mid360: Background subtraction enabled (voxel: "
                  << voxel_size << "m)" << std::endl;
    } else {
        std::cout << "Livox Mid360: Background subtraction disabled" << std::endl;
        background_model.clear();
    }
}
