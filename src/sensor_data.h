#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include "common.h"

// Sensor data structures
struct TimestampedFrame {
    double timestamp;
    cv::Mat color_image;
    cv::Mat depth_image;
    std::string source_id;

    TimestampedFrame() = default;
    TimestampedFrame(double ts, const cv::Mat& color, const cv::Mat& depth, const std::string& id)
        : timestamp(ts), color_image(color.clone()), depth_image(depth.clone()), source_id(id) {}
};

struct ThetaFrame {
    double timestamp;
    cv::Mat image;
    std::string camera_id;
    bool valid;

    ThetaFrame() : valid(false) {}
    ThetaFrame(double ts, const cv::Mat& img, const std::string& id)
        : timestamp(ts), image(img.clone()), camera_id(id), valid(true) {}
};

// Intel D405 camera frame
struct IntelD405Frame {
    double timestamp;
    cv::Mat color_image;
    cv::Mat depth_image;
    bool valid;

    IntelD405Frame() : valid(false) {}
    IntelD405Frame(double ts, const cv::Mat& color, const cv::Mat& depth)
        : timestamp(ts), color_image(color.clone()), depth_image(depth.clone()), valid(true) {}
};

// Livox Mid360 point structure
struct LivoxPoint {
    float x;
    float y;
    float z;
    uint8_t intensity;
    uint8_t tag;
};

// Livox Mid360 point cloud
struct LivoxPointCloud {
    std::vector<LivoxPoint> points;
    double timestamp;
    uint32_t frame_id;
    bool valid;

    LivoxPointCloud() : timestamp(0.0), frame_id(0), valid(false) {}
};

// Livox Mid360 LiDAR frame
struct LivoxMid360Frame {
    double timestamp;
    LivoxPointCloud cloud_data;
    int frame_id;
    bool valid;

    LivoxMid360Frame() : timestamp(0.0), frame_id(0), valid(false) {}
};

#endif // SENSOR_DATA_H