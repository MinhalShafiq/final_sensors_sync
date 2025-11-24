#ifndef COMMON_H
#define COMMON_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <thread>
#include <deque>
#include <mutex>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <filesystem>
#include <signal.h>
#include <condition_variable>
#include <algorithm>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <libuvc/libuvc.h>

// Constants
#define MAX_PIPELINE_LEN 1024
#define MAX_THETA_CAMERAS 2

#endif // COMMON_H