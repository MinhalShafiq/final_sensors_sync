#include "multi_sensor_parallel_arms.h"
#include <iostream>
#include <csignal>
#include <string>

MultiSensorParallelArms* recorder = nullptr;

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nReceived shutdown signal, stopping..." << std::endl;
        if (recorder) recorder->stop_recording();
        exit(0);
    }
}

void print_usage() {
    std::cout << "Usage: parallel_arms_recorder [options]\n";
    std::cout << "Options:\n";
    std::cout << "  -d <seconds>   Duration in seconds (0 for infinite, default: 0)\n";
    std::cout << "  -n <name>      Base name (used in pipe path and dir if -D not given)\n";
    std::cout << "  -D <dir>       Exact output directory (overrides timestamp suffix)\n";
    std::cout << "  -h             Show this help message\n";
    std::cout << "\nTrigger pipe path: /tmp/multi_sensor_parallel_arms_trigger_<name>\n";
    std::cout << "Trigger payload format:\n";
    std::cout << "  SAVE_<MODE>|<episode_id>|<m_j1,..,m_j6,m_grip_angle,m_grip_effort>|<s_j1,...,s_grip_effort>\n";
    std::cout << "  MODE in {NORMAL, NUMPY, BOTH}\n";
}

int main(int argc, char* argv[]) {
    int duration = 0;
    std::string base_name = "multi_sensor_parallel_arms";
    std::string exact_dir = "";

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") { print_usage(); return 0; }
        else if (arg == "-d" && i + 1 < argc) duration = std::atoi(argv[++i]);
        else if (arg == "-n" && i + 1 < argc) base_name = argv[++i];
        else if (arg == "-D" && i + 1 < argc) exact_dir = argv[++i];
        else { std::cerr << "Unknown argument: " << arg << std::endl; print_usage(); return 1; }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    std::cout << "=== Multi-Sensor Parallel Arms Recorder ===" << std::endl;
    std::cout << "Base name: " << base_name << std::endl;
    if (!exact_dir.empty()) std::cout << "Exact dir: " << exact_dir << std::endl;
    std::cout << "Duration: " << (duration == 0 ? "infinite" : std::to_string(duration) + "s") << std::endl;

    try {
        recorder = new MultiSensorParallelArms(base_name, exact_dir);
        recorder->start_recording(duration);

        if (duration == 0) {
            std::cout << "Press Ctrl+C to stop recording..." << std::endl;
            while (true) std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        delete recorder;
        recorder = nullptr;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        if (recorder) delete recorder;
        return 1;
    }

    std::cout << "Recording completed successfully." << std::endl;
    return 0;
}
