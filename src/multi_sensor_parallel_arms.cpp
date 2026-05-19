#include "multi_sensor_parallel_arms.h"
#include "npy_writer.h"
#include "pcd_writer.h"
#include <librealsense2/rs.hpp>
#include <sstream>
#include <vector>

MultiSensorParallelArms::MultiSensorParallelArms(const std::string& base_name,
                                                 const std::string& exact_dir)
    : d415_pipe(nullptr), latest_d415_timestamp(0.0) {
    start_time = std::chrono::steady_clock::now();

    if (!exact_dir.empty()) {
        base_dir = exact_dir;
    } else {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        base_dir = base_name + "_" + ss.str();
    }
    trigger_pipe_path = "/tmp/multi_sensor_parallel_arms_trigger_" + base_name;

    std::filesystem::create_directories(base_dir);
    std::filesystem::create_directories(base_dir + "/intel_d415/color");
    std::filesystem::create_directories(base_dir + "/intel_d415/depth");
    std::filesystem::create_directories(base_dir + "/theta_cam0");
    std::filesystem::create_directories(base_dir + "/theta_cam1");
    std::filesystem::create_directories(base_dir + "/livox_lidar");
    std::filesystem::create_directories(base_dir + "/intel_d405/color");
    std::filesystem::create_directories(base_dir + "/intel_d405/depth");

    data_csv.open(base_dir + "/parallel_data.csv");
    data_csv << "save_id,episode_id,livox_timestamp,intel_d415_timestamp,theta0_timestamp,theta1_timestamp,d405_timestamp,"
             << "intel_d415_color,intel_d415_depth,theta0_image,theta1_image,livox_num_points,"
             << "d405_color,d405_depth,"
             << "master_j1,master_j2,master_j3,master_j4,master_j5,master_j6,"
             << "master_grip_angle,master_grip_effort,"
             << "slave_j1,slave_j2,slave_j3,slave_j4,slave_j5,slave_j6,"
             << "slave_grip_angle,slave_grip_effort\n";

    timestamp_csv.open(base_dir + "/save_timestamps.csv");
    timestamp_csv << "save_id,episode_id,save_start_time,save_end_time,save_duration_ms\n";

    uvc_error_t res = uvc_init(&uvc_ctx, NULL);
    if (res < 0) {
        std::cerr << "Failed to initialize UVC context" << std::endl;
        uvc_perror(res, "uvc_init");
        uvc_ctx = nullptr;
    }
    std::cout << "Recording to directory: " << base_dir << std::endl;
    std::cout << "Trigger pipe: " << trigger_pipe_path << std::endl;
}

MultiSensorParallelArms::~MultiSensorParallelArms() {
    if (d415_pipe) {
        delete d415_pipe;
        d415_pipe = nullptr;
    }
    if (uvc_ctx) {
        uvc_exit(uvc_ctx);
    }
    unlink(trigger_pipe_path.c_str());
}

double MultiSensorParallelArms::get_current_time() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now - start_time;
    return std::chrono::duration<double>(duration).count();
}

int MultiSensorParallelArms::find_theta_devices() {
    uvc_device_t** device_list;
    uvc_error_t res;
    int found = 0;
    res = uvc_get_device_list(uvc_ctx, &device_list);
    if (res != UVC_SUCCESS) {
        std::cerr << "Failed to get UVC device list" << std::endl;
        return 0;
    }

    std::cout << "\n=== THETA Camera Detection (Arms Variant) ===" << std::endl;
    std::cout << "Scanning for RICOH THETA X cameras (VID:PID = 0x05ca:0x2717)..." << std::endl;

    int total_devices = 0;
    for (int i = 0; device_list[i] != NULL; i++) {
        total_devices++;
    }
    std::cout << "Total UVC devices found: " << total_devices << std::endl;

    for (int i = 0; device_list[i] != NULL && found < MAX_THETA_CAMERAS; i++) {
        uvc_device_descriptor_t* desc;
        if (uvc_get_device_descriptor(device_list[i], &desc) == UVC_SUCCESS) {
            if (desc->idVendor == 0x05ca && desc->idProduct == 0x2717) {
                std::cout << "Found THETA X device #" << i << " - attempting to initialize as cam" << found << "..." << std::endl;
                uvc_ref_device(device_list[i]);
                if (theta_cameras[found].initialize(uvc_ctx, device_list[i], found)) {
                    found++;
                    std::cout << "Successfully initialized as cam" << (found-1) << std::endl;
                } else {
                    std::cout << "Failed to initialize device" << std::endl;
                    uvc_unref_device(device_list[i]);
                }
            }
            uvc_free_device_descriptor(desc);
        }
    }
    uvc_free_device_list(device_list, 0);

    std::cout << "=== Detection Complete: " << found << " THETA camera(s) initialized ===\n" << std::endl;
    return found;
}

bool MultiSensorParallelArms::initialize_theta_cameras() {
    if (!uvc_ctx) {
        std::cerr << "UVC context not initialized" << std::endl;
        return false;
    }
    active_theta_cameras = find_theta_devices();
    if (active_theta_cameras == 0) {
        std::cerr << "No RICOH THETA X cameras found" << std::endl;
        return false;
    }
    std::cout << "Found " << active_theta_cameras << " THETA X camera(s)" << std::endl;
    return true;
}

bool MultiSensorParallelArms::initialize_d405_camera(int w, int h, int fps) {
    if (d405_camera.is_initialized()) return true;
    std::cout << "Initializing Intel D405 camera..." << std::endl;
    bool ok = d405_camera.initialize(w, h, fps);
    if (!ok) std::cerr << "Warning: Failed to initialize Intel D405 camera" << std::endl;
    else     std::cout << "Intel D405 camera initialized successfully" << std::endl;
    return ok;
}

bool MultiSensorParallelArms::initialize_livox_lidar(const std::string& config_path) {
    if (livox_lidar.is_initialized()) return true;
    std::cout << "Initializing Livox Mid360 LiDAR..." << std::endl;
    bool ok = livox_lidar.initialize(config_path);
    if (!ok) std::cerr << "Warning: Failed to initialize Livox Mid360 LiDAR" << std::endl;
    else     std::cout << "Livox Mid360 LiDAR initialized successfully" << std::endl;
    return ok;
}

// Trigger message format (newline terminated):
//   SAVE_<MODE>|<episode_id>|m_j1,m_j2,m_j3,m_j4,m_j5,m_j6,m_grip_angle,m_grip_effort|s_j1,...,s_grip_effort
// MODE in {NORMAL, NUMPY, BOTH}. The arm sections are optional; absent fields default to 0
// and `arms.valid` is set true only when both arm sections parsed successfully.
bool MultiSensorParallelArms::parse_trigger_payload(const std::string& cmd, SaveMode& mode, ArmPayload& arms) {
    std::string trimmed = cmd;
    auto pos = trimmed.find('\n');
    if (pos != std::string::npos) trimmed.resize(pos);

    std::vector<std::string> parts;
    std::stringstream ss(trimmed);
    std::string item;
    while (std::getline(ss, item, '|')) parts.push_back(item);
    if (parts.empty()) return false;

    if (parts[0].find("NORMAL") != std::string::npos) mode = SaveMode::NORMAL;
    else if (parts[0].find("NUMPY") != std::string::npos) mode = SaveMode::NUMPY;
    else mode = SaveMode::BOTH;

    if (parts.size() < 2) return true;
    try { arms.episode_id = std::stoi(parts[1]); } catch (...) { arms.episode_id = -1; }

    auto split_doubles = [](const std::string& s, std::vector<double>& out) {
        std::stringstream ds(s);
        std::string tok;
        while (std::getline(ds, tok, ',')) {
            try { out.push_back(std::stod(tok)); } catch (...) { out.push_back(0.0); }
        }
    };

    if (parts.size() >= 3) {
        std::vector<double> m;
        split_doubles(parts[2], m);
        for (int i = 0; i < 6 && i < (int)m.size(); ++i) arms.master_joints[i] = m[i];
        if (m.size() > 6) arms.master_grip_angle = m[6];
        if (m.size() > 7) arms.master_grip_effort = m[7];
    }
    if (parts.size() >= 4) {
        std::vector<double> s;
        split_doubles(parts[3], s);
        for (int i = 0; i < 6 && i < (int)s.size(); ++i) arms.slave_joints[i] = s[i];
        if (s.size() > 6) arms.slave_grip_angle = s[6];
        if (s.size() > 7) arms.slave_grip_effort = s[7];
    }
    arms.valid = parts.size() >= 4;
    return true;
}

void MultiSensorParallelArms::trigger_listener_loop() {
    std::cout << "Trigger listener started. Listening on: " << trigger_pipe_path << std::endl;

    if (mkfifo(trigger_pipe_path.c_str(), 0666) != 0 && errno != EEXIST) {
        std::cerr << "Failed to create trigger pipe: " << strerror(errno) << std::endl;
        return;
    }

    while (!shutdown) {
        int fd = open(trigger_pipe_path.c_str(), O_RDONLY);
        if (fd == -1) {
            std::cerr << "Failed to open trigger pipe" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        // Bigger buffer: extended payload with arm data is well under 512 bytes
        char buffer[1024] = {0};
        ssize_t bytes = read(fd, buffer, sizeof(buffer) - 1);
        close(fd);

        if (bytes > 0) {
            std::string cmd(buffer, bytes);
            SaveMode mode = SaveMode::BOTH;
            ArmPayload arms;
            parse_trigger_payload(cmd, mode, arms);

            {
                std::lock_guard<std::mutex> lock(save_request_mutex);
                pending_save = {true, mode, arms};
            }
            std::cout << "Received SAVE trigger (mode: "
                      << (mode == SaveMode::NORMAL ? "NORMAL" :
                          mode == SaveMode::NUMPY ? "NUMPY" : "BOTH")
                      << ", episode: " << arms.episode_id
                      << ", arms: " << (arms.valid ? "yes" : "no") << ")" << std::endl;

            save_parallel_set(mode, arms);
        }
    }

    unlink(trigger_pipe_path.c_str());
}

void MultiSensorParallelArms::d415_streaming_loop() {
    try {
        std::cout << "Intel D415 streaming thread started" << std::endl;
        d415_pipe = new rs2::pipeline();
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        d415_pipe->start(cfg);

        for (int i = 0; i < 30; i++) {
            d415_pipe->wait_for_frames(1000);
        }
        std::cout << "Intel D415 warmed up and ready" << std::endl;

        while (!shutdown) {
            try {
                rs2::frameset frames = d415_pipe->wait_for_frames(100);
                rs2::depth_frame depth = frames.get_depth_frame();
                rs2::video_frame color = frames.get_color_frame();
                if (!depth || !color) continue;

                cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)depth.get_data());
                cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color.get_data());

                {
                    std::lock_guard<std::mutex> lock(d415_mutex);
                    latest_d415_color = color_image.clone();
                    latest_d415_depth = depth_image.clone();
                    latest_d415_timestamp = get_current_time();
                }
            } catch (const rs2::error& e) {
                if (!shutdown) {
                    std::cerr << "Intel D415 streaming error: " << e.what() << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }

        d415_pipe->stop();
        std::cout << "Intel D415 streaming thread stopped" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Intel D415 thread failed: " << e.what() << std::endl;
    }
}

void MultiSensorParallelArms::record_save_timestamp() {}

void MultiSensorParallelArms::save_parallel_set(SaveMode mode, const ArmPayload& arms) {
    auto save_start = std::chrono::steady_clock::now();

    try {
        std::lock_guard<std::mutex> lock(save_mutex);
        save_count++;

        std::stringstream prefix_ss;
        prefix_ss << "parallel_" << std::setfill('0') << std::setw(6) << save_count;
        std::string prefix = prefix_ss.str();

        bool save_normal = (mode == SaveMode::NORMAL || mode == SaveMode::BOTH);
        bool save_numpy = (mode == SaveMode::NUMPY || mode == SaveMode::BOTH);

        double livox_timestamp = 0.0;
        double d415_timestamp = 0.0;
        double theta0_timestamp = 0.0;
        double theta1_timestamp = 0.0;
        double d405_timestamp = 0.0;

        std::string d415_color_path = "", d415_depth_path = "";
        std::string theta0_path = "", theta1_path = "";
        std::string d405_color_path = "", d405_depth_path = "";
        int livox_num_points = 0;

        std::vector<std::thread> capture_threads;

        capture_threads.emplace_back([&]() {
            try {
                cv::Mat color_image, depth_image;
                {
                    std::lock_guard<std::mutex> lock(d415_mutex);
                    if (!latest_d415_color.empty() && !latest_d415_depth.empty()) {
                        color_image = latest_d415_color.clone();
                        depth_image = latest_d415_depth.clone();
                        d415_timestamp = latest_d415_timestamp;
                    }
                }

                if (!color_image.empty() && !depth_image.empty()) {
                    if (save_normal) {
                        d415_color_path = base_dir + "/intel_d415/color/" + prefix + "_color.png";
                        d415_depth_path = base_dir + "/intel_d415/depth/" + prefix + "_depth.png";
                        cv::imwrite(d415_color_path, color_image);

                        cv::Mat depth_8bit, depth_colormap;
                        depth_image.convertTo(depth_8bit, CV_8UC1, 0.03);
                        cv::applyColorMap(depth_8bit, depth_colormap, cv::COLORMAP_JET);
                        cv::imwrite(d415_depth_path, depth_colormap);
                    }

                    if (save_numpy) {
                        npy::save(base_dir + "/intel_d415/color/" + prefix + "_color.npy", color_image);
                        npy::save(base_dir + "/intel_d415/depth/" + prefix + "_depth.npy", depth_image);
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "D415 capture error: " << e.what() << std::endl;
            }
        });

        if (active_theta_cameras > 0) {
            capture_threads.emplace_back([&]() {
                try {
                    ThetaFrame frame = theta_cameras[0].get_latest_frame();
                    if (frame.valid && !frame.image.empty()) {
                        theta0_timestamp = get_current_time();
                        if (save_normal) {
                            std::string theta0_image_path = base_dir + "/theta_cam0/" + prefix + "_theta0.jpg";
                            std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
                            cv::imwrite(theta0_image_path, frame.image, compression_params);
                            theta0_path = std::filesystem::path(theta0_image_path).filename().string();
                        }
                        if (save_numpy) {
                            npy::save(base_dir + "/theta_cam0/" + prefix + "_theta0.npy", frame.image);
                        }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "THETA 0 capture error: " << e.what() << std::endl;
                }
            });
        }

        if (active_theta_cameras > 1) {
            capture_threads.emplace_back([&]() {
                try {
                    ThetaFrame frame = theta_cameras[1].get_latest_frame();
                    if (frame.valid && !frame.image.empty()) {
                        theta1_timestamp = get_current_time();
                        if (save_normal) {
                            std::string theta1_image_path = base_dir + "/theta_cam1/" + prefix + "_theta1.jpg";
                            std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
                            cv::imwrite(theta1_image_path, frame.image, compression_params);
                            theta1_path = std::filesystem::path(theta1_image_path).filename().string();
                        }
                        if (save_numpy) {
                            npy::save(base_dir + "/theta_cam1/" + prefix + "_theta1.npy", frame.image);
                        }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "THETA 1 capture error: " << e.what() << std::endl;
                }
            });
        }

        if (d405_camera.is_initialized()) {
            capture_threads.emplace_back([&]() {
                try {
                    cv::Mat color_frame, depth_frame;
                    if (d405_camera.capture_frame(color_frame, depth_frame)) {
                        d405_timestamp = get_current_time();
                        if (save_normal) {
                            if (!color_frame.empty()) {
                                d405_color_path = base_dir + "/intel_d405/color/" + prefix + "_color.png";
                                cv::imwrite(d405_color_path, color_frame);
                            }
                            if (!depth_frame.empty()) {
                                cv::Mat depth_8bit, depth_colormap;
                                depth_frame.convertTo(depth_8bit, CV_8UC1, 0.03);
                                cv::applyColorMap(depth_8bit, depth_colormap, cv::COLORMAP_JET);
                                d405_depth_path = base_dir + "/intel_d405/depth/" + prefix + "_depth.png";
                                cv::imwrite(d405_depth_path, depth_colormap);
                            }
                        }
                        if (save_numpy) {
                            if (!color_frame.empty())
                                npy::save(base_dir + "/intel_d405/color/" + prefix + "_color.npy", color_frame);
                            if (!depth_frame.empty())
                                npy::save(base_dir + "/intel_d405/depth/" + prefix + "_depth.npy", depth_frame);
                        }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "D405 capture error: " << e.what() << std::endl;
                }
            });
        }

        if (livox_lidar.is_initialized()) {
            capture_threads.emplace_back([&]() {
                try {
                    LivoxMid360Frame frame;
                    if (livox_lidar.get_latest_frame(frame) && frame.valid) {
                        livox_timestamp = get_current_time();
                        livox_num_points = frame.cloud_data.points.size();

                        if (save_numpy && !frame.cloud_data.points.empty()) {
                            int N = frame.cloud_data.points.size();
                            cv::Mat livox_data(N, 4, CV_32F);
                            for (int i = 0; i < N; ++i) {
                                livox_data.at<float>(i, 0) = frame.cloud_data.points[i].x;
                                livox_data.at<float>(i, 1) = frame.cloud_data.points[i].y;
                                livox_data.at<float>(i, 2) = frame.cloud_data.points[i].z;
                                livox_data.at<float>(i, 3) = frame.cloud_data.points[i].intensity / 255.0f;
                            }
                            npy::save(base_dir + "/livox_lidar/" + prefix + "_pointcloud.npy", livox_data);
                        }
                        if (!frame.cloud_data.points.empty()) {
                            std::string pcd_path = base_dir + "/livox_lidar/" + prefix + "_pointcloud.pcd";
                            if (!pcd::save_pcd(pcd_path, frame.cloud_data.points, true)) {
                                std::cerr << "Failed to save PCD file: " << pcd_path << std::endl;
                            }
                        }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Livox capture error: " << e.what() << std::endl;
                }
            });
        }

        for (auto& t : capture_threads) {
            if (t.joinable()) t.join();
        }

        auto save_end = std::chrono::steady_clock::now();
        auto save_duration = std::chrono::duration_cast<std::chrono::milliseconds>(save_end - save_start);

        if (save_normal) {
            data_csv << save_count << ","
                     << arms.episode_id << ","
                     << std::fixed << std::setprecision(6)
                     << (livox_timestamp > 0 ? std::to_string(livox_timestamp) : "") << ","
                     << (d415_timestamp > 0 ? std::to_string(d415_timestamp) : "") << ","
                     << (theta0_timestamp > 0 ? std::to_string(theta0_timestamp) : "") << ","
                     << (theta1_timestamp > 0 ? std::to_string(theta1_timestamp) : "") << ","
                     << (d405_timestamp > 0 ? std::to_string(d405_timestamp) : "") << ","
                     << std::filesystem::path(d415_color_path).filename().string() << ","
                     << std::filesystem::path(d415_depth_path).filename().string() << ","
                     << theta0_path << ","
                     << theta1_path << ","
                     << livox_num_points << ","
                     << std::filesystem::path(d405_color_path).filename().string() << ","
                     << std::filesystem::path(d405_depth_path).filename().string() << ","
                     << arms.master_joints[0] << "," << arms.master_joints[1] << ","
                     << arms.master_joints[2] << "," << arms.master_joints[3] << ","
                     << arms.master_joints[4] << "," << arms.master_joints[5] << ","
                     << arms.master_grip_angle << "," << arms.master_grip_effort << ","
                     << arms.slave_joints[0] << "," << arms.slave_joints[1] << ","
                     << arms.slave_joints[2] << "," << arms.slave_joints[3] << ","
                     << arms.slave_joints[4] << "," << arms.slave_joints[5] << ","
                     << arms.slave_grip_angle << "," << arms.slave_grip_effort << "\n";
            data_csv.flush();
        }

        {
            std::lock_guard<std::mutex> ts_lock(timestamp_csv_mutex);
            double save_start_time = std::chrono::duration<double>(save_start - start_time).count();
            double save_end_time = std::chrono::duration<double>(save_end - start_time).count();

            timestamp_csv << save_count << ","
                          << arms.episode_id << ","
                          << std::fixed << std::setprecision(6)
                          << save_start_time << ","
                          << save_end_time << ","
                          << save_duration.count() << "\n";
            timestamp_csv.flush();
        }

        std::cout << "Saved parallel set #" << save_count
                  << " (episode " << arms.episode_id
                  << ", mode: " << (mode == SaveMode::NORMAL ? "NORMAL" :
                                    mode == SaveMode::NUMPY ? "NUMPY" : "BOTH")
                  << ", duration: " << save_duration.count() << " ms)" << std::endl;

    } catch (const std::exception& e) {
        std::cout << "Error saving parallel set: " << e.what() << std::endl;
    }
}

void MultiSensorParallelArms::start_recording(int duration) {
    std::cout << "Starting multi-sensor parallel arms recording (duration=" << duration << "s, 0=infinite)..." << std::endl;

    gst_init(nullptr, nullptr);

    std::cout << "Starting Intel D415 continuous streaming..." << std::endl;
    d415_stream_thread = std::thread(&MultiSensorParallelArms::d415_streaming_loop, this);

    if (!initialize_theta_cameras()) {
        std::cout << "Warning: No THETA cameras found, continuing with other sensors" << std::endl;
    }
    for (int i = 0; i < active_theta_cameras; i++) {
        theta_cameras[i].start_streaming();
    }

    std::cout << "Initializing Intel D405 camera..." << std::endl;
    initialize_d405_camera();

    std::cout << "Initializing Livox Mid360 LiDAR..." << std::endl;
    initialize_livox_lidar();

    std::cout << "Waiting for sensors to warm up..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "All sensors ready. Waiting for triggers..." << std::endl;

    trigger_listener_thread = std::thread(&MultiSensorParallelArms::trigger_listener_loop, this);

    if (duration > 0) {
        std::unique_lock<std::mutex> lock(save_mutex);
        shutdown_cv.wait_for(lock, std::chrono::seconds(duration), [this] { return shutdown.load(); });
        stop_recording();
    }
}

void MultiSensorParallelArms::stop_recording() {
    std::cout << "Stopping recording..." << std::endl;
    shutdown = true;
    shutdown_cv.notify_all();

    for (int i = 0; i < active_theta_cameras; i++) {
        theta_cameras[i].cleanup();
    }
    d405_camera.cleanup();
    livox_lidar.cleanup();

    if (d415_stream_thread.joinable()) d415_stream_thread.join();
    if (trigger_listener_thread.joinable()) trigger_listener_thread.join();

    if (data_csv.is_open()) data_csv.close();
    if (timestamp_csv.is_open()) timestamp_csv.close();

    std::cout << "Recording stopped. Data saved to: " << base_dir << std::endl;
    std::cout << "Total saved sets: " << save_count << std::endl;
}

int MultiSensorParallelArms::get_save_count() const { return save_count; }
int MultiSensorParallelArms::get_active_theta_cameras() const { return active_theta_cameras; }
