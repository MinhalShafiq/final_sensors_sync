#ifndef INTEL_D405_H
#define INTEL_D405_H

#include "sensor_data.h"
#include <librealsense2/rs.hpp>

class IntelD405Camera {
private:
    rs2::pipeline pipeline;
    rs2::config config;
    std::string serial_number;
    int width;
    int height;
    int fps;
    bool initialized;
    bool pipeline_started;

public:
    IntelD405Camera();
    ~IntelD405Camera();

    bool initialize(int w = 640, int h = 480, int framerate = 30);
    bool capture_frame(cv::Mat& color_frame, cv::Mat& depth_frame);
    void cleanup();
    bool is_initialized() const { return initialized; }
    std::string get_serial_number() const { return serial_number; }

private:
    bool find_d405_device();
};

#endif // INTEL_D405_H
