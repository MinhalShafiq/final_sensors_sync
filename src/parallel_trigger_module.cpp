#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <string>

namespace py = pybind11;

void send_parallel_save_trigger(const std::string& mode = "both", const std::string& pipe_path = "/tmp/multi_sensor_parallel_trigger_multi_sensor_parallel") {
    if (mode != "normal" && mode != "numpy" && mode != "both") {
        throw std::invalid_argument("Mode must be 'normal', 'numpy', or 'both'");
    }

    int fd = open(pipe_path.c_str(), O_WRONLY | O_NONBLOCK);
    if (fd == -1) {
        throw std::runtime_error("Parallel recorder not running or pipe not found at: " + pipe_path);
    }

    std::string message = "SAVE_" + mode + "\n";
    ssize_t written = write(fd, message.c_str(), message.size());
    close(fd);

    if (written == -1) {
        throw std::runtime_error("Failed to send trigger command to parallel recorder");
    }
}

PYBIND11_MODULE(parallel_trigger_module, m) {
    m.doc() = "Python bindings to trigger parallel sensor saving";
    m.def("save", &send_parallel_save_trigger,
          py::arg("mode") = "both",
          py::arg("pipe_path") = "/tmp/multi_sensor_parallel_trigger_multi_sensor_parallel",
          "Send a save trigger to the parallel_recorder. Mode: 'normal', 'numpy', or 'both'");
}
