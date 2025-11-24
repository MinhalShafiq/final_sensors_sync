#include "multi_sensor_parallel.h"
#include <iostream>
#include <csignal>
#include <string>

MultiSensorParallel* recorder = nullptr;

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nReceived interrupt signal, stopping..." << std::endl;
        if (recorder) {
            recorder->stop_recording();
        }
        exit(0);
    }
}

void print_usage() {
    std::cout << "Usage: parallel_recorder [options]\n";
    std::cout << "Options:\n";
    std::cout << "  -d <seconds>   Duration in seconds (0 for infinite, default: 0)\n";
    std::cout << "  -n <name>      Base name for output directory (default: multi_sensor_parallel)\n";
    std::cout << "  -h             Show this help message\n";
    std::cout << "\nTo trigger a save, use:\n";
    std::cout << "  echo 'SAVE' > /tmp/multi_sensor_parallel_trigger_<name>\n";
    std::cout << "  echo 'SAVE_NORMAL' > /tmp/multi_sensor_parallel_trigger_<name>\n";
    std::cout << "  echo 'SAVE_NUMPY' > /tmp/multi_sensor_parallel_trigger_<name>\n";
}

int main(int argc, char* argv[]) {
    int duration = 0;
    std::string base_name = "multi_sensor_parallel";

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage();
            return 0;
        } else if (arg == "-d" && i + 1 < argc) {
            duration = std::atoi(argv[++i]);
        } else if (arg == "-n" && i + 1 < argc) {
            base_name = argv[++i];
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            print_usage();
            return 1;
        }
    }

    // Register signal handler
    signal(SIGINT, signal_handler);

    std::cout << "=== Multi-Sensor Parallel Recorder ===" << std::endl;
    std::cout << "Base name: " << base_name << std::endl;
    std::cout << "Duration: " << (duration == 0 ? "infinite" : std::to_string(duration) + "s") << std::endl;
    std::cout << "\nThis recorder captures from all sensors in parallel (no synchronization)" << std::endl;
    std::cout << "Data is saved when triggered via named pipe" << std::endl;
    std::cout << "Pipe path: /tmp/multi_sensor_parallel_trigger_" << base_name << std::endl;
    std::cout << "\n";

    try {
        recorder = new MultiSensorParallel(base_name);
        recorder->start_recording(duration);

        if (duration == 0) {
            // Keep running until interrupted
            std::cout << "Press Ctrl+C to stop recording..." << std::endl;
            while (true) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        delete recorder;
        recorder = nullptr;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        if (recorder) {
            delete recorder;
        }
        return 1;
    }

    std::cout << "Recording completed successfully." << std::endl;
    return 0;
}
