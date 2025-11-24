#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <cstring>
#include <string>

int main(int argc, char* argv[]) {
    std::string pipe_path = "/tmp/multi_sensor_parallel_trigger_multi_sensor_parallel";
    std::string mode = "both"; // default

    // Parse arguments: [pipe_path] [mode]  OR  [mode]
    if (argc == 2) {
        std::string arg = argv[1];
        if (arg == "normal" || arg == "numpy" || arg == "both") {
            mode = arg;
        } else {
            pipe_path = arg; // assume it's the pipe path
        }
    } else if (argc >= 3) {
        pipe_path = argv[1];
        mode = argv[2];
        if (mode != "normal" && mode != "numpy" && mode != "both") {
            std::cerr << "Invalid mode. Use: normal, numpy, or both" << std::endl;
            return 1;
        }
    }

    int fd = open(pipe_path.c_str(), O_WRONLY | O_NONBLOCK);
    if (fd == -1) {
        std::cerr << "Error: Parallel recorder not running or pipe not found at: " << pipe_path << std::endl;
        std::cerr << "Make sure ./parallel_recorder is running first." << std::endl;
        return 1;
    }

    std::string message = "SAVE_" + mode + "\n";
    ssize_t written = write(fd, message.c_str(), message.size());
    close(fd);

    if (written == -1) {
        std::cerr << "Failed to send trigger." << std::endl;
        return 1;
    }

    std::cout << "Parallel trigger sent (mode: " << mode << ")" << std::endl;
    return 0;
}
