#ifndef MULTI_SENSOR_PARALLEL_ARMS_H
#define MULTI_SENSOR_PARALLEL_ARMS_H

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

// Arm payload that travels in each trigger message.
// Master fields are commanded; slave fields are feedback.
struct ArmPayload {
    int episode_id = -1;
    double master_joints[6] = {0,0,0,0,0,0};
    double master_grip_angle = 0.0;
    double master_grip_effort = 0.0;
    double slave_joints[6] = {0,0,0,0,0,0};
    double slave_grip_angle = 0.0;
    double slave_grip_effort = 0.0;
    bool valid = false;
};

class MultiSensorParallelArms {
public:
    enum class SaveMode {
        NORMAL,
        NUMPY,
        BOTH
    };

private:
    std::string base_dir;
    std::string trigger_pipe_path;

    std::mutex save_mutex;
    std::mutex timestamp_csv_mutex;
    std::mutex d415_mutex;
    std::condition_variable shutdown_cv;
    std::atomic<bool> shutdown{false};

    uvc_context_t* uvc_ctx;
    ThetaCamera theta_cameras[MAX_THETA_CAMERAS];
    int active_theta_cameras = 0;

    rs2::pipeline* d415_pipe;
    cv::Mat latest_d415_color;
    cv::Mat latest_d415_depth;
    double latest_d415_timestamp;

    std::ofstream data_csv;
    std::ofstream timestamp_csv;

    int save_count = 0;

    std::chrono::steady_clock::time_point start_time;

    std::thread trigger_listener_thread;
    std::thread d415_stream_thread;

    struct SaveRequest {
        bool active = false;
        SaveMode mode = SaveMode::BOTH;
        ArmPayload arms;
    };
    SaveRequest pending_save;
    std::mutex save_request_mutex;

    double get_current_time();
    int find_theta_devices();
    void trigger_listener_loop();
    void d415_streaming_loop();
    void save_parallel_set(SaveMode mode, const ArmPayload& arms);
    void record_save_timestamp();
    bool parse_trigger_payload(const std::string& cmd, SaveMode& mode, ArmPayload& arms);

    IntelD405Camera d405_camera;
    LivoxMid360 livox_lidar;

    bool initialize_d405_camera(int w = 640, int h = 480, int fps = 30);
    bool initialize_livox_lidar(const std::string& config_path = "mid360_config.json");

public:
    // If `exact_dir` is non-empty, it is used verbatim as the output directory.
    // Otherwise the legacy behavior (base_name + "_" + timestamp) is used.
    MultiSensorParallelArms(const std::string& base_name = "multi_sensor_parallel_arms",
                            const std::string& exact_dir = "");
    ~MultiSensorParallelArms();
    bool initialize_theta_cameras();
    void start_recording(int duration = 0);
    void stop_recording();
    int get_save_count() const;
    int get_active_theta_cameras() const;
    const std::string& get_base_dir() const { return base_dir; }
    const std::string& get_trigger_pipe_path() const { return trigger_pipe_path; }
};

#endif // MULTI_SENSOR_PARALLEL_ARMS_H
