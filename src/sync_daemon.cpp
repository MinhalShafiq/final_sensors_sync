#include "multi_sensor_synchronizer.h"
#include <csignal>
#include <iostream>

std::unique_ptr<MultiSensorSynchronizer> g_sync;

void signal_handler(int sig) {
    std::cout << "\nCaught signal " << sig << ". Shutting down..." << std::endl;
    if (g_sync) g_sync->stop_recording();
    exit(0);
}

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_sync = std::make_unique<MultiSensorSynchronizer>("session");
    g_sync->set_sync_frequency(10);
    g_sync->set_sync_window(50.0); // ±50ms (increased from 30ms for better D405 matching)
    g_sync->start_recording(0); // Run until stopped

    return 0;
}