#ifndef MULTI_SENSOR_SYNCHRONIZER_H
#define MULTI_SENSOR_SYNCHRONIZER_H

#include "common.h"
#include "sensor_data.h"
#include "theta_camera.h"
#include "intel_d405.h"
#include "livox_mid360.h"
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <limits>
#include <algorithm>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

class MultiSensorSynchronizer {
public:
    enum class SaveMode {
        NORMAL,
        NUMPY,
        BOTH
    };

private:
    std::string base_dir;
    std::string trigger_pipe_path;

    // Sensor data buffers
    std::deque<TimestampedFrame> intel_d415_buffer;
    std::deque<ThetaFrame> theta_buffer[MAX_THETA_CAMERAS];
    std::deque<IntelD405Frame> d405_buffer;
    std::deque<LivoxMid360Frame> livox_buffer;

    // Mutexes
    std::mutex intel_d415_mutex;
    std::mutex theta_mutex[MAX_THETA_CAMERAS];
    std::mutex d405_mutex;
    std::mutex livox_mutex;
    std::mutex sync_mutex;
    std::mutex save_request_mutex;

    std::condition_variable shutdown_cv;
    std::atomic<bool> shutdown{false};

    // THETA cameras
    uvc_context_t* uvc_ctx;
    ThetaCamera theta_cameras[MAX_THETA_CAMERAS];
    int active_theta_cameras = 0;

    // Output files
    std::ofstream sync_csv;
    std::ofstream lidar_pointcloud_csv;
    std::ofstream lidar_imu_csv;

    // Configuration
    double sync_window = 0.05; // 50ms
    double buffer_duration = 0.5;
    std::atomic<int> sync_frequency{5}; // Hz
    int sync_count = 0;

    // Timing
    std::chrono::steady_clock::time_point start_time;

    // Threads
    std::thread sync_thread;
    std::thread intel_d415_thread_handle;
    std::thread d405_thread_handle;
    std::thread livox_thread_handle;
    std::thread trigger_listener_thread;

    // Save control
    struct SaveRequest {
        bool active = false;
        SaveMode mode = SaveMode::BOTH;
    };
    SaveRequest pending_save;

    // Private methods
    double get_current_time();
    void cleanup_old_frames();
    int find_theta_devices();
    bool synchronization_callback();
    void synchronization_loop();
    void remove_processed_frames(const LivoxMid360Frame* livox_frame,
                               const TimestampedFrame* intel_d415_frame,
                               const ThetaFrame* theta0_frame,
                               const ThetaFrame* theta1_frame,
                               const IntelD405Frame* d405_frame);
    void save_synchronized_set(const LivoxMid360Frame* livox_frame,
                             const TimestampedFrame* intel_d415_frame,
                             const ThetaFrame* theta0_frame,
                             const ThetaFrame* theta1_frame,
                             const IntelD405Frame* d405_frame,
                             double rs_diff, double theta0_diff, double theta1_diff,
                             double d405_diff,
                             SaveMode mode);

    // Thread functions
    void intel_d415_thread();
    void theta_collection_thread();
    void d405_thread();
    void livox_thread();
    void trigger_listener_loop();

    // Camera and LiDAR instances
    IntelD405Camera d405_camera;
    LivoxMid360 livox_lidar;

    // Initialization methods
    bool initialize_d405_camera(int w = 640, int h = 480, int fps = 30);
    bool initialize_livox_lidar(const std::string& config_path = "mid360_config.json");

public:
    MultiSensorSynchronizer(const std::string& base_name = "multi_sensor_data");
    ~MultiSensorSynchronizer();
    bool initialize_theta_cameras();
    void start_recording(int duration = 0); // 0 = run until stopped
    void stop_recording();
    void set_sync_frequency(int frequency);
    void set_sync_window(double window_ms);
    int get_sync_count() const;
    int get_active_theta_cameras() const;
};

#endif // MULTI_SENSOR_SYNCHRONIZER_H