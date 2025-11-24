#include "intel_d405.h"
#include <iostream>

IntelD405Camera::IntelD405Camera()
    : initialized(false), pipeline_started(false), width(640), height(480), fps(30) {
}

IntelD405Camera::~IntelD405Camera() {
    cleanup();
}

bool IntelD405Camera::find_d405_device() {
    rs2::context ctx;
    auto devices = ctx.query_devices();

    if (devices.size() == 0) {
        std::cerr << "No RealSense devices found" << std::endl;
        return false;
    }

    // Search for D405 camera
    for (auto&& dev : devices) {
        std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
        if (name.find("D405") != std::string::npos) {
            serial_number = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            std::cout << "Found Intel D405 camera with serial: " << serial_number << std::endl;
            return true;
        }
    }

    std::cerr << "Intel D405 camera not found among " << devices.size() << " device(s)" << std::endl;
    return false;
}

bool IntelD405Camera::initialize(int w, int h, int framerate) {
    if (initialized) {
        return true;
    }

    width = w;
    height = h;
    fps = framerate;

    // Find the D405 device
    if (!find_d405_device()) {
        return false;
    }

    try {
        // Configure the pipeline
        config.enable_device(serial_number);
        config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
        config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);

        // Start the pipeline
        pipeline.start(config);
        pipeline_started = true;

        // Wait for a few frames to stabilize
        for (int i = 0; i < 30; i++) {
            pipeline.wait_for_frames();
        }

        initialized = true;
        std::cout << "Intel D405 camera initialized successfully at "
                  << width << "x" << height << "@" << fps << "fps" << std::endl;
        return true;

    } catch (const rs2::error& e) {
        std::cerr << "RealSense error in D405 initialization: " << e.what() << std::endl;
        cleanup();
        return false;
    }
}

bool IntelD405Camera::capture_frame(cv::Mat& color_frame, cv::Mat& depth_frame) {
    if (!initialized || !pipeline_started) {
        return false;
    }

    try {
        // Wait for frames with timeout
        rs2::frameset frames = pipeline.wait_for_frames(5000);

        rs2::frame color = frames.get_color_frame();
        rs2::frame depth = frames.get_depth_frame();

        if (!color || !depth) {
            std::cerr << "Incomplete frames from D405" << std::endl;
            return false;
        }

        // Convert to OpenCV format
        cv::Mat color_mat(cv::Size(width, height), CV_8UC3,
                         (void*)color.get_data(), cv::Mat::AUTO_STEP);
        color_frame = color_mat.clone();

        cv::Mat depth_mat(cv::Size(width, height), CV_16UC1,
                         (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        depth_frame = depth_mat.clone();

        return !color_frame.empty() && !depth_frame.empty();

    } catch (const rs2::error& e) {
        std::cerr << "RealSense error during D405 frame capture: " << e.what() << std::endl;
        return false;
    }
}

void IntelD405Camera::cleanup() {
    if (pipeline_started) {
        try {
            pipeline.stop();
        } catch (const rs2::error& e) {
            std::cerr << "Error stopping D405 pipeline: " << e.what() << std::endl;
        }
        pipeline_started = false;
    }
    initialized = false;
}
