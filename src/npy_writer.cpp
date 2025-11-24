#include "npy_writer.h"
#include <endian.h>
#include <stdexcept>
#include <string>

namespace npy {

namespace {
    std::string mat_type_to_dtype(int mat_type) {
        int depth = mat_type & CV_MAT_DEPTH_MASK;
        int channels = 1 + (mat_type >> CV_CN_SHIFT);

        switch (depth) {
            case CV_8U:  return "u1";
            case CV_8S:  return "i1";
            case CV_16U: return "u2";
            case CV_16S: return "i2";
            case CV_32S: return "i4";
            case CV_32F: return "f4";
            case CV_64F: return "f8";
            default: throw std::runtime_error("Unsupported OpenCV depth for .npy");
        }
    }

    size_t mat_type_to_elem_size(int mat_type) {
        int depth = mat_type & CV_MAT_DEPTH_MASK;
        int channels = 1 + (mat_type >> CV_CN_SHIFT);

        size_t base_size;
        switch (depth) {
            case CV_8U: case CV_8S:  base_size = 1; break;
            case CV_16U: case CV_16S: base_size = 2; break;
            case CV_32S: case CV_32F: base_size = 4; break;
            case CV_64F:              base_size = 8; break;
            default: throw std::runtime_error("Unsupported OpenCV depth for .npy");
        }
        return base_size * channels;
    }
} // anonymous namespace

void save(const std::string& filename, const cv::Mat& mat) {
    if (mat.empty()) return;

    cv::Mat continuous_mat = mat;
    if (!mat.isContinuous()) {
        continuous_mat = mat.clone();
    }

    std::string dtype = mat_type_to_dtype(continuous_mat.type());
    bool fortran_order = false;

    std::vector<size_t> shape;
    if (continuous_mat.channels() == 1) {
        shape = { (size_t)continuous_mat.rows, (size_t)continuous_mat.cols };
    } else {
        shape = { (size_t)continuous_mat.rows, (size_t)continuous_mat.cols, (size_t)continuous_mat.channels() };
    }

    // Build header dict
    std::string header = "{'descr': '<" + dtype + "', 'fortran_order': " +
                        (fortran_order ? "True" : "False") + ", 'shape': (";
    for (size_t i = 0; i < shape.size(); ++i) {
        header += std::to_string(shape[i]);
        if (i < shape.size() - 1) header += ", ";
    }
    header += "), }";

    // Finalize header with padding
    header += '\n';
    size_t header_len = 10 + 2 + header.size(); // "NUMPY\x01\x00" + uint16 + header
    size_t padding = (64 - (header_len % 64)) % 64;
    header.append(padding, ' ');

    // Write file
    std::ofstream file(filename, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot open file for writing: " + filename);

    file.write("\x93NUMPY\x01\x00", 8);

    uint16_t hlen = static_cast<uint16_t>(header.size());
    file.write(reinterpret_cast<char*>(&hlen), sizeof(hlen));

    file.write(header.data(), header.size());

    size_t data_size = continuous_mat.total() * mat_type_to_elem_size(continuous_mat.type());
    file.write(reinterpret_cast<const char*>(continuous_mat.data), data_size);
}

} // namespace npy