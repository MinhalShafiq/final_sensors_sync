#ifndef NPY_WRITER_H
#define NPY_WRITER_H

#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <cstdint>
#include <stdexcept>
#include <opencv2/opencv.hpp>

namespace npy {

// Write OpenCV Mat to .npy file (supports CV_8U, CV_16U, CV_32F)
void save(const std::string& filename, const cv::Mat& mat);

} // namespace npy

#endif