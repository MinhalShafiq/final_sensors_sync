#ifndef LIVOX_MID360_H
#define LIVOX_MID360_H

#include "sensor_data.h"
#include <vector>
#include <deque>
#include <map>
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

    // Simple time-based accumulation (Livox needs 200-500ms for density)
    std::vector<LivoxPoint> accumulation_buffer;
    std::mutex frame_mutex;
    double accumulation_start_time;
    uint32_t current_frame_id;

    // Background model for motion filtering (optional)
    struct VoxelKey {
        int x, y, z;
        bool operator<(const VoxelKey& other) const {
            if (x != other.x) return x < other.x;
            if (y != other.y) return y < other.y;
            return z < other.z;
        }
    };
    std::map<VoxelKey, std::vector<LivoxPoint>> background_model;
    bool enable_background_subtraction;
    float voxel_size;
    double integration_time;  // seconds (0.2-0.5s for high density)

    // Helper methods
    VoxelKey point_to_voxel(const LivoxPoint& point) const;
    std::vector<LivoxPoint> filter_dynamic_points(const std::vector<LivoxPoint>& points);
    void update_background_model(const std::vector<LivoxPoint>& static_points);

    // Callbacks
    static void static_point_cloud_callback(uint32_t handle, const uint8_t dev_type,
                                           LivoxLidarEthernetPacket* data, void* client_data);
    static void static_lidar_info_callback(const uint32_t handle, const LivoxLidarInfo* info,
                                          void* client_data);
    static void static_work_mode_callback(livox_status status, uint32_t handle,
                                         LivoxLidarAsyncControlResponse *response,
                                         void *client_data);

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

    // For synchronization
    bool get_latest_frame(LivoxMid360Frame& frame);

    // Configuration: integration_time in seconds (0.2-0.5 recommended)
    void set_integration_time(double seconds);
    void set_background_subtraction(bool enable, float voxel_size = 0.15f);
};

#endif // LIVOX_MID360_H
