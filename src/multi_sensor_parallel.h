#ifndef MULTI_SENSOR_PARALLEL_H
#define MULTI_SENSOR_PARALLEL_H

#include "common.h"
#include "sensor_data.h"
#include "theta_camera.h"
#include "intel_d405.h"
#include "livox_mid360.h"
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

class MultiSensorParallel {
public:
    enum class SaveMode {
        NORMAL,
        NUMPY,
        BOTH
    };

private:
    std::string base_dir;
    std::string trigger_pipe_path;

    // Mutexes for thread-safe operations
    std::mutex save_mutex;
    std::mutex timestamp_csv_mutex;
    std::mutex d415_mutex;
    std::condition_variable shutdown_cv;
    std::atomic<bool> shutdown{false};

    // THETA cameras
    uvc_context_t* uvc_ctx;
    ThetaCamera theta_cameras[MAX_THETA_CAMERAS];
    int active_theta_cameras = 0;

    // Intel D415 pipeline (keep alive for continuous streaming)
    rs2::pipeline* d415_pipe;
    cv::Mat latest_d415_color;
    cv::Mat latest_d415_depth;
    double latest_d415_timestamp;

    // Output files
    std::ofstream data_csv;
    std::ofstream timestamp_csv;

    // Configuration
    int save_count = 0;

    // Timing
    std::chrono::steady_clock::time_point start_time;

    // Threads
    std::thread trigger_listener_thread;
    std::thread d415_stream_thread;

    // Save control
    struct SaveRequest {
        bool active = false;
        SaveMode mode = SaveMode::BOTH;
    };
    SaveRequest pending_save;
    std::mutex save_request_mutex;

    // Private methods
    double get_current_time();
    int find_theta_devices();
    void trigger_listener_loop();
    void d415_streaming_loop();
    void save_parallel_set(SaveMode mode);
    void record_save_timestamp();

    // Camera and LiDAR instances
    IntelD405Camera d405_camera;
    LivoxMid360 livox_lidar;

    // Initialization methods
    bool initialize_d405_camera(int w = 640, int h = 480, int fps = 30);
    bool initialize_livox_lidar(const std::string& config_path = "mid360_config.json");

public:
    MultiSensorParallel(const std::string& base_name = "multi_sensor_parallel");
    ~MultiSensorParallel();
    bool initialize_theta_cameras();
    void start_recording(int duration = 0); // 0 = run until stopped
    void stop_recording();
    int get_save_count() const;
    int get_active_theta_cameras() const;
};

#endif // MULTI_SENSOR_PARALLEL_H
