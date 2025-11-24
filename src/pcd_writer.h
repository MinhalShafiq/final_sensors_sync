#ifndef PCD_WRITER_H
#define PCD_WRITER_H

#include <string>
#include <vector>
#include <fstream>
#include "sensor_data.h"

namespace pcd {

// Write point cloud data to PCD file format
// Supports both ASCII and binary formats
bool save_pcd(const std::string& filename,
              const std::vector<LivoxPoint>& points,
              bool binary = true);

// Write point cloud from cv::Mat (Nx4 format: x, y, z, intensity)
bool save_pcd_from_mat(const std::string& filename,
                       const std::vector<LivoxPoint>& points,
                       bool binary = true);

} // namespace pcd

#endif // PCD_WRITER_H
