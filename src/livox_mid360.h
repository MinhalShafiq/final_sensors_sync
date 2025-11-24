#ifndef LIVOX_MID360_H
#define LIVOX_MID360_H

#include "sensor_data.h"
#include <vector>
#include <mutex>
#include <string>
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

class LivoxMid360 {
private:
    uint32_t lidar_handle;
    bool initialized;
    bool is_running;
    std::string config_file_path;

    // Current frame accumulation
    std::vector<LivoxPoint> current_frame_points;
    std::mutex frame_mutex;
    double frame_start_time;
    uint32_t current_frame_id;

    // Callbacks (static methods that call instance methods)
    static void static_point_cloud_callback(uint32_t handle, const uint8_t dev_type,
                                           LivoxLidarEthernetPacket* data, void* client_data);
    static void static_lidar_info_callback(const uint32_t handle, const LivoxLidarInfo* info,
                                          void* client_data);
    static void static_work_mode_callback(livox_status status, uint32_t handle,
                                         LivoxLidarAsyncControlResponse *response,
                                         void *client_data);

    // Instance callback handlers
    void handle_point_cloud(uint32_t handle, const uint8_t dev_type,
                           LivoxLidarEthernetPacket* data);
    void handle_lidar_info(const uint32_t handle, const LivoxLidarInfo* info);
    void handle_work_mode(livox_status status, uint32_t handle,
                         LivoxLidarAsyncControlResponse *response);

public:
    LivoxMid360();
    ~LivoxMid360();

    bool initialize(const std::string& config_path);
    bool get_point_cloud(LivoxPointCloud& cloud);
    void cleanup();
    bool is_initialized() const { return initialized; }
    bool is_active() const { return is_running; }

    // For synchronization - get accumulated frame
    bool get_latest_frame(LivoxMid360Frame& frame);
};

#endif // LIVOX_MID360_H
