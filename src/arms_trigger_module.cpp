#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>

namespace py = pybind11;

// Build extended SAVE message and write to named pipe non-blocking.
// arm vectors must be length 8 each: [j1..j6, grip_angle, grip_effort].
void send_arms_save_trigger(const std::string& mode,
                            int episode_id,
                            const std::vector<double>& master_arm,
                            const std::vector<double>& slave_arm,
                            const std::string& pipe_path) {
    if (mode != "normal" && mode != "numpy" && mode != "both") {
        throw std::invalid_argument("Mode must be 'normal', 'numpy', or 'both'");
    }
    if (master_arm.size() != 8 || slave_arm.size() != 8) {
        throw std::invalid_argument("master_arm and slave_arm must be length 8 (j1..j6, grip_angle, grip_effort)");
    }

    std::ostringstream oss;
    oss << "SAVE_";
    for (char c : mode) oss << (char)std::toupper(c);
    oss << "|" << episode_id << "|";
    oss << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < master_arm.size(); ++i) {
        if (i) oss << ",";
        oss << master_arm[i];
    }
    oss << "|";
    for (size_t i = 0; i < slave_arm.size(); ++i) {
        if (i) oss << ",";
        oss << slave_arm[i];
    }
    oss << "\n";

    int fd = open(pipe_path.c_str(), O_WRONLY | O_NONBLOCK);
    if (fd == -1) {
        throw std::runtime_error("parallel_arms_recorder not running or pipe not found at: " + pipe_path);
    }

    std::string message = oss.str();
    ssize_t written = write(fd, message.c_str(), message.size());
    close(fd);

    if (written == -1) {
        throw std::runtime_error("Failed to send trigger to parallel_arms_recorder");
    }
}

PYBIND11_MODULE(arms_trigger_module, m) {
    m.doc() = "Python bindings to trigger parallel_arms_recorder with arm payload";
    m.def("save", &send_arms_save_trigger,
          py::arg("mode") = "both",
          py::arg("episode_id") = -1,
          py::arg("master_arm") = std::vector<double>(8, 0.0),
          py::arg("slave_arm") = std::vector<double>(8, 0.0),
          py::arg("pipe_path") = "/tmp/multi_sensor_parallel_arms_trigger_multi_sensor_parallel_arms",
          "Send a save trigger with arm payload. master_arm/slave_arm are length-8: j1..j6, grip_angle, grip_effort.");
}
