#include "pcd_writer.h"
#include <iostream>
#include <iomanip>
#include <sstream>

namespace pcd {

bool save_pcd(const std::string& filename,
              const std::vector<LivoxPoint>& points,
              bool binary) {
    if (points.empty()) {
        std::cerr << "PCD Writer: No points to save" << std::endl;
        return false;
    }

    std::ofstream file;
    if (binary) {
        file.open(filename, std::ios::out | std::ios::binary);
    } else {
        file.open(filename, std::ios::out);
    }

    if (!file.is_open()) {
        std::cerr << "PCD Writer: Failed to open file: " << filename << std::endl;
        return false;
    }

    // Write PCD header
    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z intensity\n";
    file << "SIZE 4 4 4 4\n";  // intensity is 4 bytes for proper color/intensity representation
    file << "TYPE F F F U\n";
    file << "COUNT 1 1 1 1\n";
    file << "WIDTH " << points.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << points.size() << "\n";
    file << "DATA " << (binary ? "binary" : "ascii") << "\n";

    if (binary) {
        // Write binary data
        for (const auto& point : points) {
            file.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
            file.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
            file.write(reinterpret_cast<const char*>(&point.z), sizeof(float));
            // Write intensity as uint32_t for proper color/intensity information
            uint32_t intensity = static_cast<uint32_t>(point.intensity);
            file.write(reinterpret_cast<const char*>(&intensity), sizeof(uint32_t));
        }
    } else {
        // Write ASCII data
        file << std::fixed << std::setprecision(6);
        for (const auto& point : points) {
            file << point.x << " "
                 << point.y << " "
                 << point.z << " "
                 << static_cast<uint32_t>(point.intensity) << "\n";
        }
    }

    file.close();
    return true;
}

bool save_pcd_from_mat(const std::string& filename,
                       const std::vector<LivoxPoint>& points,
                       bool binary) {
    return save_pcd(filename, points, binary);
}

} // namespace pcd
