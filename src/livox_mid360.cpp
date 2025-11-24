#include "livox_mid360.h"
#include <iostream>
#include <chrono>
#include <cstring>

LivoxMid360::LivoxMid360()
    : lidar_handle(0), initialized(false), is_running(false),
      frame_start_time(0.0), current_frame_id(0) {
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

        for (uint32_t i = 0; i < point_num; i++) {
            LivoxPoint point;
            // Convert from mm to m
            point.x = raw_points[i].x / 1000.0f;
            point.y = raw_points[i].y / 1000.0f;
            point.z = raw_points[i].z / 1000.0f;
            point.intensity = raw_points[i].reflectivity;
            point.tag = raw_points[i].tag;

            current_frame_points.push_back(point);
        }
    }
}

void LivoxMid360::handle_lidar_info(const uint32_t handle, const LivoxLidarInfo* info) {
    if (!info) {
        std::cerr << "Livox Mid360: Info change callback failed, info is nullptr" << std::endl;
        return;
    }

    lidar_handle = handle;
    std::cout << "Livox Mid360: Connected to LiDAR (Handle: " << handle
              << ", SN: " << info->sn << ")" << std::endl;

    // Start the lidar by setting work mode to normal
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, static_work_mode_callback, this);
}

void LivoxMid360::handle_work_mode(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response) {
    if (!response) {
        return;
    }

    std::cout << "Livox Mid360: Work mode callback - status: " << status
              << ", handle: " << handle
              << ", ret_code: " << response->ret_code
              << ", error_key: " << response->error_key << std::endl;

    if (status == kLivoxLidarStatusSuccess && response->ret_code == 0) {
        is_running = true;
        auto now = std::chrono::steady_clock::now();
        frame_start_time = std::chrono::duration<double>(now.time_since_epoch()).count();
        std::cout << "Livox Mid360: Started successfully!" << std::endl;
    } else {
        std::cerr << "Livox Mid360: Failed to start (status: " << status
                  << ", ret_code: " << response->ret_code << ")" << std::endl;
    }
}

bool LivoxMid360::initialize(const std::string& config_path) {
    if (initialized) {
        std::cout << "Livox Mid360: Already initialized" << std::endl;
        return true;
    }

    config_file_path = config_path;

    std::cout << "Livox Mid360: Initializing SDK with config: " << config_path << std::endl;

    // Initialize Livox SDK
    if (!LivoxLidarSdkInit(config_path.c_str())) {
        std::cerr << "Livox Mid360: SDK initialization failed!" << std::endl;
        return false;
    }

    std::cout << "Livox Mid360: SDK initialized successfully" << std::endl;

    // Set callbacks with 'this' pointer so we can access instance data
    SetLivoxLidarPointCloudCallBack(static_point_cloud_callback, this);
    SetLivoxLidarInfoChangeCallback(static_lidar_info_callback, this);

    initialized = true;

    // Wait for the lidar to connect and start
    std::cout << "Livox Mid360: Waiting for LiDAR to connect..." << std::endl;
    int wait_count = 0;
    while (!is_running && wait_count < 50) {  // Wait up to 5 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_count++;
    }

    if (!is_running) {
        std::cerr << "Livox Mid360: LiDAR did not start within timeout" << std::endl;
        cleanup();
        return false;
    }

    std::cout << "Livox Mid360: Initialization complete and running" << std::endl;
    return true;
}

bool LivoxMid360::get_point_cloud(LivoxPointCloud& cloud) {
    if (!initialized || !is_running) {
        return false;
    }

    std::lock_guard<std::mutex> lock(frame_mutex);

    if (current_frame_points.empty()) {
        return false;
    }

    cloud.points = current_frame_points;
    auto now = std::chrono::steady_clock::now();
    cloud.timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
    cloud.frame_id = current_frame_id++;
    cloud.valid = true;

    // Clear for next frame
    current_frame_points.clear();

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
        LivoxLidarSdkUninit();
        initialized = false;
        std::cout << "Livox Mid360: Cleanup complete" << std::endl;
    }
}
