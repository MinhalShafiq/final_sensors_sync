#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <vector>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <sstream>
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#ifdef _WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h>
#endif

std::mutex data_mutex;
std::vector<LivoxLidarCartesianHighRawPoint> current_frame;
uint32_t frame_counter = 0;
std::string output_directory = "./pcd_output/";
bool save_enabled = false;

// Save point cloud to PCD file (Binary format)
void SaveToPCD(const std::vector<LivoxLidarCartesianHighRawPoint>& points, uint32_t frame_id) {
    if (points.empty()) {
        printf("Frame %u is empty, skipping...\n", frame_id);
        return;
    }

    std::stringstream filename;
    filename << output_directory << "frame_" << std::setfill('0') << std::setw(6) << frame_id << ".pcd";
    
    std::ofstream file(filename.str(), std::ios::binary);
    if (!file.is_open()) {
        printf("Failed to open file: %s\n", filename.str().c_str());
        return;
    }

    // Write PCD header (text part)
    std::stringstream header;
    header << "# .PCD v0.7 - Point Cloud Data file format\n";
    header << "VERSION 0.7\n";
    header << "FIELDS x y z intensity\n";
    header << "SIZE 4 4 4 4\n";
    header << "TYPE F F F U\n";
    header << "COUNT 1 1 1 1\n";
    header << "WIDTH " << points.size() << "\n";
    header << "HEIGHT 1\n";
    header << "VIEWPOINT 0 0 0 1 0 0 0\n";
    header << "POINTS " << points.size() << "\n";
    header << "DATA binary\n";
    
    file << header.str();

    // Write point data in binary format
    for (const auto& point : points) {
        // Convert from mm to m
        float x = point.x / 1000.0f;
        float y = point.y / 1000.0f;
        float z = point.z / 1000.0f;
        uint32_t intensity = (uint32_t)point.reflectivity;
        
        file.write(reinterpret_cast<const char*>(&x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&z), sizeof(float));
        file.write(reinterpret_cast<const char*>(&intensity), sizeof(uint32_t));
    }

    file.close();
    printf("Saved frame %u with %zu points to %s\n", frame_id, points.size(), filename.str().c_str());
}

// Point cloud callback function
void PointCloudCallback(uint32_t handle, const uint8_t dev_type,
                        LivoxLidarEthernetPacket* data, void* client_data) {
    if (data == nullptr || !save_enabled) {
        return;
    }

    // Parse point cloud data
    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        std::lock_guard<std::mutex> lock(data_mutex);
        LivoxLidarCartesianHighRawPoint* points = (LivoxLidarCartesianHighRawPoint*)data->data;
        uint32_t point_num = data->dot_num;

        for (uint32_t i = 0; i < point_num; i++) {
            current_frame.push_back(points[i]);
        }
    }
}

// Work mode callback
void WorkModeCallback(livox_status status, uint32_t handle, 
                      LivoxLidarAsyncControlResponse *response, void *client_data) {
    if (response == nullptr) {
        return;
    }
    printf("WorkModeCallback, status:%u, handle:%u, ret_code:%u, error_key:%u\n",
        status, handle, response->ret_code, response->error_key);
    
    if (status == kLivoxLidarStatusSuccess && response->ret_code == 0) {
        printf("Lidar started successfully! Beginning to save frames...\n");
        save_enabled = true;
    }
}

// Info change callback
void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
    if (info == nullptr) {
        printf("Lidar info change callback failed, the info is nullptr.\n");
        return;
    } 
    printf("LidarInfoChangeCallback - Lidar handle: %u SN: %s\n", handle, info->sn);
    
    // Start the lidar by setting work mode to normal
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
}

// Frame processing thread
void FrameProcessingThread() {
    const int frame_duration_ms = 100; // ~10Hz frame rate, adjust as needed
    
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(frame_duration_ms));
        
        if (!save_enabled) {
            continue;
        }
        
        std::vector<LivoxLidarCartesianHighRawPoint> frame_to_save;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (!current_frame.empty()) {
                frame_to_save = current_frame;
                current_frame.clear();
            }
        }
        
        if (!frame_to_save.empty()) {
            SaveToPCD(frame_to_save, frame_counter++);
        }
    }
}

int main(int argc, const char *argv[]) {
    if (argc != 2) {
        printf("Usage: %s <config_file_path>\n", argv[0]);
        printf("Example: %s ./mid360_config.json\n", argv[0]);
        return -1;
    }
    
    const std::string path = argv[1];

    // Create output directory
    system(("mkdir -p " + output_directory).c_str());

    // Initialize Livox SDK with config file
    if (!LivoxLidarSdkInit(path.c_str())) {
        printf("Livox SDK initialization failed!\n");
        LivoxLidarSdkUninit();
        return -1;
    }
    printf("Livox SDK initialized.\n");

    // Set callbacks
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

    // Start frame processing thread
    std::thread frame_thread(FrameProcessingThread);
    frame_thread.detach();

    printf("Starting to capture and save frames...\n");
    printf("Press Ctrl+C to stop.\n");

    // Keep running
#ifdef WIN32
    Sleep(300000); // 300 seconds
#else
    sleep(300);
#endif

    // Cleanup
    LivoxLidarSdkUninit();
    printf("Program ended.\n");
    return 0;
}