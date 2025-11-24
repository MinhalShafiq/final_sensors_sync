#include "multi_sensor_synchronizer.h"
#include "npy_writer.h" // for saving .npy files
#include "pcd_writer.h" // for saving .pcd files

MultiSensorSynchronizer::MultiSensorSynchronizer(const std::string& base_name) {
    start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    std::string timestamp = ss.str();
    base_dir = base_name + "_" + timestamp;
    trigger_pipe_path = "/tmp/multi_sensor_trigger_" + base_name;

    std::filesystem::create_directories(base_dir);
    std::filesystem::create_directories(base_dir + "/intel_d415/color");
    std::filesystem::create_directories(base_dir + "/intel_d415/depth");
    std::filesystem::create_directories(base_dir + "/theta_cam0");
    std::filesystem::create_directories(base_dir + "/theta_cam1");
    std::filesystem::create_directories(base_dir + "/livox_lidar");
    std::filesystem::create_directories(base_dir + "/intel_d405/color");
    std::filesystem::create_directories(base_dir + "/intel_d405/depth");

    sync_csv.open(base_dir + "/synchronized_data.csv");
    sync_csv << "sync_id,livox_timestamp,intel_d415_timestamp,theta0_timestamp,theta1_timestamp,d405_timestamp,"
             << "livox_intel_d415_diff_ms,livox_theta0_diff_ms,livox_theta1_diff_ms,livox_d405_diff_ms,"
             << "intel_d415_color,intel_d415_depth,theta0_image,theta1_image,livox_num_points,"
             << "d405_color,d405_depth\n";

    uvc_error_t res = uvc_init(&uvc_ctx, NULL);
    if (res < 0) {
        std::cerr << "Failed to initialize UVC context" << std::endl;
        uvc_perror(res, "uvc_init");
        uvc_ctx = nullptr;
    }
    std::cout << "Recording to directory: " << base_dir << std::endl;
}

MultiSensorSynchronizer::~MultiSensorSynchronizer() {
    if (uvc_ctx) {
        uvc_exit(uvc_ctx);
    }
    unlink(trigger_pipe_path.c_str());
}

double MultiSensorSynchronizer::get_current_time() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now - start_time;
    return std::chrono::duration<double>(duration).count();
}

void MultiSensorSynchronizer::cleanup_old_frames() {
    double current_time = get_current_time();
    double cutoff_time = current_time - buffer_duration;

    {
        std::lock_guard<std::mutex> lock(intel_d415_mutex);
        intel_d415_buffer.erase(
            std::remove_if(intel_d415_buffer.begin(), intel_d415_buffer.end(),
                [cutoff_time](const TimestampedFrame& frame) {
                    return frame.timestamp < cutoff_time;
                }),
            intel_d415_buffer.end()
        );
    }

    for (int i = 0; i < active_theta_cameras; i++) {
        std::lock_guard<std::mutex> lock(theta_mutex[i]);
        theta_buffer[i].erase(
            std::remove_if(theta_buffer[i].begin(), theta_buffer[i].end(),
                [cutoff_time](const ThetaFrame& frame) {
                    return frame.timestamp < cutoff_time;
                }),
            theta_buffer[i].end()
        );
    }

    {
        std::lock_guard<std::mutex> lock(d405_mutex);
        d405_buffer.erase(
            std::remove_if(d405_buffer.begin(), d405_buffer.end(),
                [cutoff_time](const IntelD405Frame& frame) {
                    return frame.timestamp < cutoff_time;
                }),
            d405_buffer.end()
        );
    }

    {
        std::lock_guard<std::mutex> lock(livox_mutex);
        livox_buffer.erase(
            std::remove_if(livox_buffer.begin(), livox_buffer.end(),
                [cutoff_time](const LivoxMid360Frame& frame) {
                    return frame.timestamp < cutoff_time;
                }),
            livox_buffer.end()
        );
    }
}

int MultiSensorSynchronizer::find_theta_devices() {
    uvc_device_t** device_list;
    uvc_error_t res;
    int found = 0;
    res = uvc_get_device_list(uvc_ctx, &device_list);
    if (res != UVC_SUCCESS) {
        std::cerr << "Failed to get UVC device list" << std::endl;
        return 0;
    }

    std::cout << "\n=== THETA Camera Detection ===" << std::endl;
    std::cout << "Scanning for RICOH THETA X cameras (VID:PID = 0x05ca:0x2717)..." << std::endl;

    // First pass: count all UVC devices
    int total_devices = 0;
    for (int i = 0; device_list[i] != NULL; i++) {
        total_devices++;
    }
    std::cout << "Total UVC devices found: " << total_devices << std::endl;

    // Second pass: find THETA cameras
    for (int i = 0; device_list[i] != NULL && found < MAX_THETA_CAMERAS; i++) {
        uvc_device_descriptor_t* desc;
        if (uvc_get_device_descriptor(device_list[i], &desc) == UVC_SUCCESS) {
            if (desc->idVendor == 0x05ca && desc->idProduct == 0x2717) {
                std::cout << "Found THETA X device #" << i << " - attempting to initialize as cam" << found << "..." << std::endl;
                uvc_ref_device(device_list[i]);
                if (theta_cameras[found].initialize(uvc_ctx, device_list[i], found)) {
                    found++;
                    std::cout << "✓ Successfully initialized as cam" << (found-1) << std::endl;
                } else {
                    std::cout << "✗ Failed to initialize device" << std::endl;
                    uvc_unref_device(device_list[i]);
                }
            }
            uvc_free_device_descriptor(desc);
        }
    }
    uvc_free_device_list(device_list, 0);

    std::cout << "=== Detection Complete: " << found << " THETA camera(s) initialized ===" << std::endl;
    if (found > 0) {
        std::cout << "Note: Physical camera order depends on USB enumeration" << std::endl;
        std::cout << "      Check serial numbers above to identify each camera" << std::endl;
    }
    std::cout << std::endl;

    return found;
}

bool MultiSensorSynchronizer::initialize_theta_cameras() {
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

void MultiSensorSynchronizer::intel_d415_thread() {
    try {
        rs2::pipeline pipe;
        rs2::config cfg;
        rs2::context ctx;
        auto devices = ctx.query_devices();
        if (devices.size() == 0) {
            std::cout << "No Intel D415 devices found!" << std::endl;
            return;
        }
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        pipe.start(cfg);
        std::cout << "Intel D415 thread started" << std::endl;
        while (!shutdown) {
            try {
                rs2::frameset frames = pipe.wait_for_frames(100);
                rs2::depth_frame depth = frames.get_depth_frame();
                rs2::video_frame color = frames.get_color_frame();
                if (!depth || !color) continue;
                double capture_time = get_current_time();
                cv::Mat depth_image(cv::Size(640, 480), CV_16UC1, (void*)depth.get_data());
                cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color.get_data());
                TimestampedFrame frame(capture_time, color_image, depth_image, "intel_d415");
                {
                    std::lock_guard<std::mutex> lock(intel_d415_mutex);
                    intel_d415_buffer.push_back(std::move(frame));
                }
            } catch (const rs2::error& e) {
                if (!shutdown) {
                    std::cout << "Intel D415 error: " << e.what() << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
        pipe.stop();
    } catch (const std::exception& e) {
        std::cout << "Intel D415 thread failed: " << e.what() << std::endl;
    }
    std::cout << "Intel D415 thread stopped" << std::endl;
}

void MultiSensorSynchronizer::theta_collection_thread() {
    std::cout << "THETA collection thread started" << std::endl;
    for (int i = 0; i < active_theta_cameras; i++) {
        theta_cameras[i].start_streaming();
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Frame collection monitoring
    int cam0_frames = 0, cam1_frames = 0;
    auto last_report = std::chrono::steady_clock::now();

    while (!shutdown) {
        try {
            for (int i = 0; i < active_theta_cameras; i++) {
                ThetaFrame frame = theta_cameras[i].get_latest_frame();
                if (frame.valid) {
                    ThetaFrame sync_frame = frame;
                    sync_frame.timestamp = get_current_time();
                    {
                        std::lock_guard<std::mutex> lock(theta_mutex[i]);
                        theta_buffer[i].push_back(std::move(sync_frame));
                    }

                    // Track frames per camera
                    if (i == 0) cam0_frames++;
                    else if (i == 1) cam1_frames++;
                }
            }

            // Report frame collection stats every 10 seconds
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_report);
            if (elapsed.count() >= 10) {
                std::cout << "THETA frame collection stats (last 10s):" << std::endl;
                if (active_theta_cameras > 0) {
                    std::cout << "  - cam0: " << cam0_frames << " frames (" << (cam0_frames/10.0) << " fps)" << std::endl;
                }
                if (active_theta_cameras > 1) {
                    std::cout << "  - cam1: " << cam1_frames << " frames (" << (cam1_frames/10.0) << " fps)" << std::endl;
                }
                cam0_frames = cam1_frames = 0;
                last_report = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } catch (const std::exception& e) {
            if (!shutdown) {
                std::cout << "THETA collection error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    std::cout << "THETA collection thread stopped" << std::endl;
}

void MultiSensorSynchronizer::d405_thread() {
    std::cout << "Intel D405 capture thread started" << std::endl;
    if (!d405_camera.is_initialized()) {
        if (!initialize_d405_camera()) {
            std::cout << "Intel D405 camera not available, thread exiting" << std::endl;
            return;
        }
    }

    int capture_count = 0;
    int failed_captures = 0;
    auto last_report = std::chrono::steady_clock::now();

    while (!shutdown) {
        try {
            cv::Mat color_frame, depth_frame;
            if (d405_camera.capture_frame(color_frame, depth_frame)) {
                double timestamp = get_current_time();
                IntelD405Frame frame(timestamp, color_frame, depth_frame);
                {
                    std::lock_guard<std::mutex> lock(d405_mutex);
                    d405_buffer.push_back(std::move(frame));
                }
                capture_count++;
            } else {
                failed_captures++;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // Report stats every 10 seconds
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_report);
            if (elapsed.count() >= 10) {
                size_t buffer_size = 0;
                {
                    std::lock_guard<std::mutex> lock(d405_mutex);
                    buffer_size = d405_buffer.size();
                }
                std::cout << "D405 stats (last 10s): " << capture_count << " captures, "
                          << failed_captures << " failures, buffer size: " << buffer_size << std::endl;
                capture_count = 0;
                failed_captures = 0;
                last_report = now;
            }
        } catch (const std::exception& e) {
            if (!shutdown) {
                std::cout << "Intel D405 thread error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    d405_camera.cleanup();
    std::cout << "Intel D405 capture thread stopped" << std::endl;
}

void MultiSensorSynchronizer::livox_thread() {
    std::cout << "Livox Mid360 capture thread started" << std::endl;
    if (!livox_lidar.is_initialized()) {
        if (!initialize_livox_lidar()) {
            std::cout << "Livox Mid360 LiDAR not available, thread exiting" << std::endl;
            return;
        }
    }

    // Wait for Livox to fully initialize and start streaming
    std::this_thread::sleep_for(std::chrono::seconds(2));

    while (!shutdown) {
        try {
            LivoxMid360Frame frame;
            if (livox_lidar.get_latest_frame(frame)) {
                frame.timestamp = get_current_time();
                {
                    std::lock_guard<std::mutex> lock(livox_mutex);
                    livox_buffer.push_back(std::move(frame));
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } catch (const std::exception& e) {
            if (!shutdown) {
                std::cout << "Livox Mid360 thread error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    livox_lidar.cleanup();
    std::cout << "Livox Mid360 capture thread stopped" << std::endl;
}

void MultiSensorSynchronizer::trigger_listener_loop() {
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

        char buffer[32] = {0};
        ssize_t bytes = read(fd, buffer, sizeof(buffer) - 1);
        close(fd);

        if (bytes > 0) {
            std::string cmd(buffer);
            SaveMode mode = SaveMode::BOTH;

            if (cmd.find("NORMAL") != std::string::npos) {
                mode = SaveMode::NORMAL;
            } else if (cmd.find("NUMPY") != std::string::npos) {
                mode = SaveMode::NUMPY;
            }

            {
                std::lock_guard<std::mutex> lock(save_request_mutex);
                pending_save = {true, mode};
            }
            std::cout << "Received SAVE trigger (mode: "
                      << (mode == SaveMode::NORMAL ? "NORMAL" :
                          mode == SaveMode::NUMPY ? "NUMPY" : "BOTH")
                      << ")" << std::endl;
        }
    }

    unlink(trigger_pipe_path.c_str());
}

bool MultiSensorSynchronizer::synchronization_callback() {
    if (shutdown) return false;
    cleanup_old_frames();

    // Copy all sensor buffers
    std::vector<LivoxMid360Frame> livox_frames;
    std::vector<TimestampedFrame> intel_d415_frames;
    std::vector<ThetaFrame> theta0_frames, theta1_frames;
    std::vector<IntelD405Frame> d405_frames;

    {
        std::lock_guard<std::mutex> lock(livox_mutex);
        livox_frames = std::vector<LivoxMid360Frame>(livox_buffer.begin(), livox_buffer.end());
    }
    {
        std::lock_guard<std::mutex> lock(intel_d415_mutex);
        intel_d415_frames = std::vector<TimestampedFrame>(intel_d415_buffer.begin(), intel_d415_buffer.end());
    }
    {
        std::lock_guard<std::mutex> lock(d405_mutex);
        d405_frames = std::vector<IntelD405Frame>(d405_buffer.begin(), d405_buffer.end());
    }
    if (active_theta_cameras > 0) {
        std::lock_guard<std::mutex> lock(theta_mutex[0]);
        theta0_frames = std::vector<ThetaFrame>(theta_buffer[0].begin(), theta_buffer[0].end());
    }
    if (active_theta_cameras > 1) {
        std::lock_guard<std::mutex> lock(theta_mutex[1]);
        theta1_frames = std::vector<ThetaFrame>(theta_buffer[1].begin(), theta_buffer[1].end());
    }

    // Use Livox as master LiDAR for synchronization
    if (livox_frames.empty()) return true;

    auto latest_livox = std::max_element(livox_frames.begin(), livox_frames.end(),
        [](const LivoxMid360Frame& a, const LivoxMid360Frame& b) {
            return a.timestamp < b.timestamp;
        });

    // Find best matches for all sensors
    TimestampedFrame* best_intel_d415 = nullptr;
    ThetaFrame* best_theta0 = nullptr;
    ThetaFrame* best_theta1 = nullptr;
    IntelD405Frame* best_d405 = nullptr;

    double d415_time_diff = std::numeric_limits<double>::max();
    double theta0_time_diff = std::numeric_limits<double>::max();
    double theta1_time_diff = std::numeric_limits<double>::max();
    double d405_time_diff = std::numeric_limits<double>::max();

    // Match Intel D415
    for (auto& frame : intel_d415_frames) {
        double diff = std::abs(latest_livox->timestamp - frame.timestamp);
        if (diff <= sync_window && diff < d415_time_diff) {
            d415_time_diff = diff;
            best_intel_d415 = &frame;
        }
    }

    // Match Intel D405
    double d405_closest_diff = std::numeric_limits<double>::max();
    for (auto& frame : d405_frames) {
        if (!frame.valid) continue;
        double diff = std::abs(latest_livox->timestamp - frame.timestamp);

        // Track closest frame even if outside sync window (for diagnostics)
        if (diff < d405_closest_diff) {
            d405_closest_diff = diff;
        }

        if (diff <= sync_window && diff < d405_time_diff) {
            d405_time_diff = diff;
            best_d405 = &frame;
        }
    }

    // Log when D405 misses sync (closest frame was outside window)
    static int d405_miss_count = 0;
    static auto last_d405_report = std::chrono::steady_clock::now();
    if (!best_d405 && !d405_frames.empty() && d405_closest_diff < std::numeric_limits<double>::max()) {
        d405_miss_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_d405_report);
        if (elapsed.count() >= 10) {
            std::cout << "⚠ D405 sync misses in last 10s: " << d405_miss_count
                      << " (closest was " << (d405_closest_diff * 1000) << "ms away, window is ±"
                      << (sync_window * 1000) << "ms)" << std::endl;
            d405_miss_count = 0;
            last_d405_report = now;
        }
    }

    // Match THETA cameras
    for (auto& frame : theta0_frames) {
        if (!frame.valid) continue;
        double diff = std::abs(latest_livox->timestamp - frame.timestamp);
        if (diff <= sync_window && diff < theta0_time_diff) {
            theta0_time_diff = diff;
            best_theta0 = &frame;
        }
    }

    for (auto& frame : theta1_frames) {
        if (!frame.valid) continue;
        double diff = std::abs(latest_livox->timestamp - frame.timestamp);
        if (diff <= sync_window && diff < theta1_time_diff) {
            theta1_time_diff = diff;
            best_theta1 = &frame;
        }
    }

    bool has_any = (best_intel_d415 || best_theta0 || best_theta1 || best_d405);
    if (!has_any) return true;

    // Check for save trigger
    SaveRequest req;
    {
        std::lock_guard<std::mutex> lock(save_request_mutex);
        if (pending_save.active) {
            req = pending_save;
            pending_save.active = false;
        }
    }

    if (req.active) {
        save_synchronized_set(&(*latest_livox), best_intel_d415, best_theta0, best_theta1, best_d405,
                            d415_time_diff, theta0_time_diff, theta1_time_diff, d405_time_diff,
                            req.mode);
    }

    remove_processed_frames(&(*latest_livox), best_intel_d415, best_theta0, best_theta1, best_d405);

    return true;
}

void MultiSensorSynchronizer::remove_processed_frames(const LivoxMid360Frame* livox_frame,
                           const TimestampedFrame* intel_d415_frame,
                           const ThetaFrame* theta0_frame,
                           const ThetaFrame* theta1_frame,
                           const IntelD405Frame* d405_frame) {
    if (livox_frame) {
        std::lock_guard<std::mutex> lock(livox_mutex);
        livox_buffer.erase(
            std::remove_if(livox_buffer.begin(), livox_buffer.end(),
                [livox_frame](const LivoxMid360Frame& frame) {
                    return frame.timestamp <= livox_frame->timestamp;
                }),
            livox_buffer.end()
        );
    }
    if (intel_d415_frame) {
        std::lock_guard<std::mutex> lock(intel_d415_mutex);
        intel_d415_buffer.erase(
            std::remove_if(intel_d415_buffer.begin(), intel_d415_buffer.end(),
                [intel_d415_frame](const TimestampedFrame& frame) {
                    return frame.timestamp <= intel_d415_frame->timestamp;
                }),
            intel_d415_buffer.end()
        );
    }
    if (theta0_frame && active_theta_cameras > 0) {
        std::lock_guard<std::mutex> lock(theta_mutex[0]);
        theta_buffer[0].erase(
            std::remove_if(theta_buffer[0].begin(), theta_buffer[0].end(),
                [theta0_frame](const ThetaFrame& frame) {
                    return frame.timestamp <= theta0_frame->timestamp;
                }),
            theta_buffer[0].end()
        );
    }
    if (theta1_frame && active_theta_cameras > 1) {
        std::lock_guard<std::mutex> lock(theta_mutex[1]);
        theta_buffer[1].erase(
            std::remove_if(theta_buffer[1].begin(), theta_buffer[1].end(),
                [theta1_frame](const ThetaFrame& frame) {
                    return frame.timestamp <= theta1_frame->timestamp;
                }),
            theta_buffer[1].end()
        );
    }
    if (d405_frame) {
        std::lock_guard<std::mutex> lock(d405_mutex);
        d405_buffer.erase(
            std::remove_if(d405_buffer.begin(), d405_buffer.end(),
                [d405_frame](const IntelD405Frame& frame) {
                    return frame.timestamp <= d405_frame->timestamp;
                }),
            d405_buffer.end()
        );
    }
}

void MultiSensorSynchronizer::save_synchronized_set(const LivoxMid360Frame* livox_frame,
                         const TimestampedFrame* intel_d415_frame,
                         const ThetaFrame* theta0_frame,
                         const ThetaFrame* theta1_frame,
                         const IntelD405Frame* d405_frame,
                         double d415_diff, double theta0_diff, double theta1_diff,
                         double d405_diff,
                         SaveMode mode) {
    try {
        sync_count++;
        std::stringstream prefix_ss;
        prefix_ss << "sync_" << std::setfill('0') << std::setw(6) << sync_count;
        std::string prefix = prefix_ss.str();

        bool save_normal = (mode == SaveMode::NORMAL || mode == SaveMode::BOTH);
        bool save_numpy = (mode == SaveMode::NUMPY || mode == SaveMode::BOTH);

        std::string d415_color_path = "", d415_depth_path = "";
        std::string theta0_path = "", theta1_path = "";

        // Save Intel D415 data
        if (save_normal && intel_d415_frame) {
            d415_color_path = base_dir + "/intel_d415/color/" + prefix + "_color.png";
            d415_depth_path = base_dir + "/intel_d415/depth/" + prefix + "_depth.png";
            if (!intel_d415_frame->color_image.empty()) {
                cv::imwrite(d415_color_path, intel_d415_frame->color_image);
            }
            if (!intel_d415_frame->depth_image.empty()) {
                cv::Mat depth_8bit, depth_colormap;
                intel_d415_frame->depth_image.convertTo(depth_8bit, CV_8UC1, 0.03);
                cv::applyColorMap(depth_8bit, depth_colormap, cv::COLORMAP_JET);
                cv::imwrite(d415_depth_path, depth_colormap);
            }
        }

        if (save_numpy && intel_d415_frame) {
            if (!intel_d415_frame->color_image.empty())
                npy::save(base_dir + "/intel_d415/color/" + prefix + "_color.npy", intel_d415_frame->color_image);
            if (!intel_d415_frame->depth_image.empty())
                npy::save(base_dir + "/intel_d415/depth/" + prefix + "_depth.npy", intel_d415_frame->depth_image);
        }

        // Save THETA camera 0 equirectangular image (unchanged stitched image)
        if (theta0_frame && theta0_frame->valid && !theta0_frame->image.empty()) {
            if (save_normal) {
                std::string theta0_image_path = base_dir + "/theta_cam0/" + prefix + "_theta0.jpg";
                std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
                cv::imwrite(theta0_image_path, theta0_frame->image, compression_params);
                theta0_path = std::filesystem::path(theta0_image_path).filename().string();
            }
            if (save_numpy) {
                npy::save(base_dir + "/theta_cam0/" + prefix + "_theta0.npy", theta0_frame->image);
            }
        }

        // Save THETA camera 1 equirectangular image (unchanged stitched image)
        if (theta1_frame && theta1_frame->valid && !theta1_frame->image.empty()) {
            if (save_normal) {
                std::string theta1_image_path = base_dir + "/theta_cam1/" + prefix + "_theta1.jpg";
                std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
                cv::imwrite(theta1_image_path, theta1_frame->image, compression_params);
                theta1_path = std::filesystem::path(theta1_image_path).filename().string();
            }
            if (save_numpy) {
                npy::save(base_dir + "/theta_cam1/" + prefix + "_theta1.npy", theta1_frame->image);
            }
        }

        // Save Intel D405 data
        if (d405_frame && d405_frame->valid) {
            if (save_normal) {
                if (!d405_frame->color_image.empty()) {
                    std::string d405_color_path = base_dir + "/intel_d405/color/" + prefix + "_color.png";
                    cv::imwrite(d405_color_path, d405_frame->color_image);
                }
                if (!d405_frame->depth_image.empty()) {
                    cv::Mat depth_8bit, depth_colormap;
                    d405_frame->depth_image.convertTo(depth_8bit, CV_8UC1, 0.03);
                    cv::applyColorMap(depth_8bit, depth_colormap, cv::COLORMAP_JET);
                    std::string d405_depth_path = base_dir + "/intel_d405/depth/" + prefix + "_depth.png";
                    cv::imwrite(d405_depth_path, depth_colormap);
                }
            }
            if (save_numpy) {
                if (!d405_frame->color_image.empty())
                    npy::save(base_dir + "/intel_d405/color/" + prefix + "_color.npy", d405_frame->color_image);
                if (!d405_frame->depth_image.empty())
                    npy::save(base_dir + "/intel_d405/depth/" + prefix + "_depth.npy", d405_frame->depth_image);
            }
        }

        // Save Livox Mid360 point cloud
        if (livox_frame && livox_frame->valid && !livox_frame->cloud_data.points.empty()) {
            if (save_numpy) {
                // Save as Nx4 array (x, y, z, intensity)
                int N = livox_frame->cloud_data.points.size();
                cv::Mat livox_data(N, 4, CV_32F);
                for (int i = 0; i < N; ++i) {
                    livox_data.at<float>(i, 0) = livox_frame->cloud_data.points[i].x;
                    livox_data.at<float>(i, 1) = livox_frame->cloud_data.points[i].y;
                    livox_data.at<float>(i, 2) = livox_frame->cloud_data.points[i].z;
                    livox_data.at<float>(i, 3) = livox_frame->cloud_data.points[i].intensity / 255.0f;
                }
                npy::save(base_dir + "/livox_lidar/" + prefix + "_pointcloud.npy", livox_data);
            }

            // Always save PCD format (binary format for efficiency)
            std::string pcd_path = base_dir + "/livox_lidar/" + prefix + "_pointcloud.pcd";
            if (!pcd::save_pcd(pcd_path, livox_frame->cloud_data.points, true)) {
                std::cerr << "Failed to save PCD file: " << pcd_path << std::endl;
            }
        }

        // Write to synchronized CSV
        if (save_normal) {
            sync_csv << sync_count << ","
                     << std::fixed << std::setprecision(6)
                     << (livox_frame ? std::to_string(livox_frame->timestamp) : "") << ","
                     << (intel_d415_frame ? std::to_string(intel_d415_frame->timestamp) : "") << ","
                     << (theta0_frame ? std::to_string(theta0_frame->timestamp) : "") << ","
                     << (theta1_frame ? std::to_string(theta1_frame->timestamp) : "") << ","
                     << (d405_frame ? std::to_string(d405_frame->timestamp) : "") << ","
                     << std::setprecision(2)
                     << (intel_d415_frame ? d415_diff * 1000 : -1) << ","
                     << (theta0_frame ? theta0_diff * 1000 : -1) << ","
                     << (theta1_frame ? theta1_diff * 1000 : -1) << ","
                     << (d405_frame ? d405_diff * 1000 : -1) << ","
                     << std::filesystem::path(d415_color_path).filename().string() << ","
                     << std::filesystem::path(d415_depth_path).filename().string() << ","
                     << theta0_path << ","
                     << theta1_path << ","
                     << (livox_frame ? livox_frame->cloud_data.points.size() : 0) << ","
                     << (d405_frame ? "d405_color" : "") << ","
                     << (d405_frame ? "d405_depth" : "") << "\n";
            sync_csv.flush();
        }

        std::cout << "Saved set #" << sync_count << " (mode: "
                  << (mode == SaveMode::NORMAL ? "NORMAL" :
                      mode == SaveMode::NUMPY ? "NUMPY" : "BOTH")
                  << ")" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Error saving synchronized set: " << e.what() << std::endl;
    }
}

void MultiSensorSynchronizer::synchronization_loop() {
    std::cout << "Starting synchronization loop at " << sync_frequency.load() << " Hz..." << std::endl;
    auto frequency_check_start = std::chrono::steady_clock::now();
    int actual_sync_count = 0;
    while (!shutdown) {
        auto callback_start = std::chrono::steady_clock::now();
        if (!synchronization_callback()) break;

        actual_sync_count++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed_since_check = std::chrono::duration_cast<std::chrono::seconds>(now - frequency_check_start);
        if (elapsed_since_check.count() >= 10) {
            double actual_freq = actual_sync_count / 10.0;
            std::cout << "Actual sync frequency: " << std::fixed << std::setprecision(2) 
                     << actual_freq << " Hz (target: " << sync_frequency.load() << " Hz)" << std::endl;
            actual_sync_count = 0;
            frequency_check_start = now;
        }

        auto callback_end = std::chrono::steady_clock::now();
        auto callback_duration = std::chrono::duration_cast<std::chrono::microseconds>(callback_end - callback_start);
        auto target_period_us = std::chrono::microseconds(1000000 / sync_frequency.load());
        if (callback_duration < target_period_us) {
            auto sleep_duration = target_period_us - callback_duration;
            std::unique_lock<std::mutex> lock(sync_mutex);
            shutdown_cv.wait_for(lock, sleep_duration, [this] { return shutdown.load(); });
        }
    }
    std::cout << "Synchronization loop stopped" << std::endl;
}

void MultiSensorSynchronizer::start_recording(int duration) {
    std::cout << "Starting multi-sensor recording (duration=" << duration << "s, 0=infinite)..." << std::endl;
    std::cout << "Sync frequency: " << sync_frequency.load() << " Hz" << std::endl;
    std::cout << "Sync window: ±" << (sync_window * 1000) << " ms" << std::endl;

    gst_init(nullptr, nullptr);

    if (!initialize_theta_cameras()) {
        std::cout << "Warning: No THETA cameras found, continuing with other sensors" << std::endl;
    }

    // Start all sensor threads
    std::cout << "Starting Intel D415 camera thread..." << std::endl;
    intel_d415_thread_handle = std::thread(&MultiSensorSynchronizer::intel_d415_thread, this);

    std::cout << "Starting Intel D405 camera thread..." << std::endl;
    d405_thread_handle = std::thread(&MultiSensorSynchronizer::d405_thread, this);

    std::cout << "Starting Livox Mid360 LiDAR thread..." << std::endl;
    livox_thread_handle = std::thread(&MultiSensorSynchronizer::livox_thread, this);

    std::thread theta_thread_handle;
    if (active_theta_cameras > 0) {
        theta_thread_handle = std::thread(&MultiSensorSynchronizer::theta_collection_thread, this);
    }

    trigger_listener_thread = std::thread(&MultiSensorSynchronizer::trigger_listener_loop, this);
    sync_thread = std::thread(&MultiSensorSynchronizer::synchronization_loop, this);

    if (duration > 0) {
        std::unique_lock<std::mutex> lock(sync_mutex);
        shutdown_cv.wait_for(lock, std::chrono::seconds(duration), [this] { return shutdown.load(); });
        stop_recording();
    }

    if (theta_thread_handle.joinable()) theta_thread_handle.join();
}

void MultiSensorSynchronizer::stop_recording() {
    std::cout << "Stopping recording..." << std::endl;
    shutdown = true;
    shutdown_cv.notify_all();

    for (int i = 0; i < active_theta_cameras; i++) {
        theta_cameras[i].cleanup();
    }

    if (sync_thread.joinable()) sync_thread.join();
    if (intel_d415_thread_handle.joinable()) intel_d415_thread_handle.join();
    if (d405_thread_handle.joinable()) d405_thread_handle.join();
    if (livox_thread_handle.joinable()) livox_thread_handle.join();
    if (trigger_listener_thread.joinable()) trigger_listener_thread.join();

    if (sync_csv.is_open()) sync_csv.close();

    std::cout << "Recording stopped. Data saved to: " << base_dir << std::endl;
    std::cout << "Total synchronized sets: " << sync_count << std::endl;
}

void MultiSensorSynchronizer::set_sync_frequency(int frequency) {
    if (frequency >= 1 && frequency <= 50) {
        sync_frequency = frequency;
        std::cout << "Sync frequency set to " << frequency << " Hz" << std::endl;
    }
}

void MultiSensorSynchronizer::set_sync_window(double window_ms) {
    sync_window = window_ms / 1000.0;
    std::cout << "Sync window set to ±" << window_ms << " ms" << std::endl;
}

int MultiSensorSynchronizer::get_sync_count() const {
    return sync_count;
}

int MultiSensorSynchronizer::get_active_theta_cameras() const {
    return active_theta_cameras;
}

bool MultiSensorSynchronizer::initialize_d405_camera(int w, int h, int fps) {
    if (d405_camera.is_initialized()) return true;
    std::cout << "Initializing Intel D405 camera..." << std::endl;
    bool ok = d405_camera.initialize(w, h, fps);
    if (!ok) {
        std::cerr << "Warning: Failed to initialize Intel D405 camera" << std::endl;
    } else {
        std::cout << "Intel D405 camera initialized successfully" << std::endl;
    }
    return ok;
}

bool MultiSensorSynchronizer::initialize_livox_lidar(const std::string& config_path) {
    if (livox_lidar.is_initialized()) return true;
    std::cout << "Initializing Livox Mid360 LiDAR..." << std::endl;
    bool ok = livox_lidar.initialize(config_path);
    if (!ok) {
        std::cerr << "Warning: Failed to initialize Livox Mid360 LiDAR" << std::endl;
    } else {
        std::cout << "Livox Mid360 LiDAR initialized successfully" << std::endl;
    }
    return ok;
}