#include "theta_camera.h"

ThetaCamera::ThetaCamera() : camera_id(-1), dev(nullptr), devh(nullptr), 
                            pipeline(nullptr), appsrc(nullptr), appsink(nullptr),
                            bus_watch_id(0), dwFrameInterval(0) {
    memset(serial, 0, sizeof(serial));
}

ThetaCamera::~ThetaCamera() {
    cleanup();
}

double ThetaCamera::get_current_time() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

// GStreamer bus callback
gboolean ThetaCamera::gst_bus_cb(GstBus *bus, GstMessage *message, gpointer data) {
    ThetaCamera *cam = (ThetaCamera *)data;
    GError *err = NULL;
    gchar *dbg = NULL;
    
    switch (GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_ERROR:
            gst_message_parse_error(message, &err, &dbg);
            std::cerr << "THETA " << cam->camera_id << " Pipeline Error: " << err->message << std::endl;
            if (dbg) std::cerr << "Debug: " << dbg << std::endl;
            g_error_free(err);
            g_free(dbg);
            cam->is_active = false;
            break;
        case GST_MESSAGE_WARNING:
            gst_message_parse_warning(message, &err, &dbg);
            std::cerr << "THETA " << cam->camera_id << " Pipeline Warning: " << err->message << std::endl;
            g_error_free(err);
            g_free(dbg);
            break;
        case GST_MESSAGE_STATE_CHANGED:
            if (GST_MESSAGE_SRC(message) == GST_OBJECT(cam->pipeline)) {
                GstState old_state, new_state;
                gst_message_parse_state_changed(message, &old_state, &new_state, NULL);
                if (new_state == GST_STATE_PLAYING) {
                    std::cout << "THETA " << cam->camera_id << ": Pipeline is now PLAYING" << std::endl;
                }
            }
            break;
        default:
            break;
    }
    return TRUE;
}

// Appsink callback
GstFlowReturn ThetaCamera::on_new_sample(GstAppSink *appsink, gpointer user_data) {
    ThetaCamera *cam = (ThetaCamera *)user_data;
    GstSample *sample;
    GstBuffer *buffer;
    GstMapInfo map;
    
    sample = gst_app_sink_pull_sample(appsink);
    if (!sample) {
        return GST_FLOW_OK;
    }
    
    buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    
    try {
        // Convert JPEG data to cv::Mat
        std::vector<uchar> jpeg_data(map.data, map.data + map.size);
        cv::Mat image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
        
        if (!image.empty()) {
            double timestamp = cam->get_current_time();
            ThetaFrame frame(timestamp, image, "theta_" + std::to_string(cam->camera_id));

            std::lock_guard<std::mutex> lock(cam->capture_mutex);
            cam->latest_frame = std::move(frame);
            cam->samples_received++;

            // Log first few successful decodes
            if (cam->samples_received <= 3) {
                std::cout << "THETA " << cam->camera_id << ": Sample #" << cam->samples_received
                          << " decoded successfully (size: " << image.cols << "x" << image.rows << ")" << std::endl;
            }

            // Debug output every 30 frames
            if (cam->samples_received % 30 == 0) {
                std::cout << "THETA " << cam->camera_id << ": Captured "
                          << cam->samples_received << " samples (size: "
                          << image.cols << "x" << image.rows << ")" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "THETA " << cam->camera_id << " decode error: " << e.what() << std::endl;
    }
    
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    
    return GST_FLOW_OK;
}

// UVC frame callback
void ThetaCamera::uvc_callback(uvc_frame_t *frame, void *ptr) {
    ThetaCamera *cam = (ThetaCamera *)ptr;

    if (!cam->is_active || frame->data_bytes == 0) {
        if (frame->data_bytes == 0 && cam->frames_received % 100 == 0) {
            std::cerr << "THETA " << cam->camera_id << ": Received empty UVC frame!" << std::endl;
        }
        return;
    }

    cam->frames_received++;

    // Log first few frames for debugging
    if (cam->frames_received <= 3) {
        std::cout << "THETA " << cam->camera_id << ": UVC frame #" << cam->frames_received
                  << " received (" << frame->data_bytes << " bytes, seq=" << frame->sequence << ")" << std::endl;
    }
    
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, frame->data_bytes, NULL);
    if (!buffer) {
        std::cerr << "THETA " << cam->camera_id << ": Failed to allocate buffer" << std::endl;
        return;
    }
    
    GstClockTime ts = GST_CLOCK_TIME_NONE;
    if (cam->dwFrameInterval > 0) {
        ts = frame->sequence * (GstClockTime)cam->dwFrameInterval * 100;
    }
    GST_BUFFER_PTS(buffer) = ts;
    GST_BUFFER_DTS(buffer) = ts;
    GST_BUFFER_DURATION(buffer) = (GstClockTime)cam->dwFrameInterval * 100;
    
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        memcpy(map.data, frame->data, frame->data_bytes);
        gst_buffer_unmap(buffer, &map);
        
        GstFlowReturn ret;
        g_signal_emit_by_name(cam->appsrc, "push-buffer", buffer, &ret);
        
        if (ret != GST_FLOW_OK && ret != GST_FLOW_FLUSHING) {
            if (cam->frames_received % 100 == 0) {
                std::cerr << "THETA " << cam->camera_id << ": Push buffer returned " << ret << std::endl;
            }
        }
    }
    
    gst_buffer_unref(buffer);
}

bool ThetaCamera::init_gstreamer() {
    char pipeline_str[MAX_PIPELINE_LEN];
    char appsrc_name[32], appsink_name[32];
    
    snprintf(appsrc_name, sizeof(appsrc_name), "appsrc_%d", camera_id);
    snprintf(appsink_name, sizeof(appsink_name), "appsink_%d", camera_id);
    
    snprintf(pipeline_str, sizeof(pipeline_str),
             "appsrc name=%s is-live=true do-timestamp=true format=time ! "
             "queue max-size-buffers=0 max-size-time=0 max-size-bytes=0 ! "
             "h264parse ! "
             "queue max-size-buffers=0 max-size-time=0 max-size-bytes=0 ! "
             "avdec_h264 ! "
             "videoconvert ! "
             "jpegenc quality=90 ! "
             "appsink name=%s emit-signals=true sync=false max-buffers=2 drop=true",
             appsrc_name, appsink_name);
    
    GError *error = NULL;
    pipeline = gst_parse_launch(pipeline_str, &error);
    if (!pipeline) {
        std::cerr << "THETA " << camera_id << ": Failed to create pipeline: " 
                  << (error ? error->message : "Unknown error") << std::endl;
        if (error) g_error_free(error);
        return false;
    }
    
    appsrc = gst_bin_get_by_name(GST_BIN(pipeline), appsrc_name);
    appsink = gst_bin_get_by_name(GST_BIN(pipeline), appsink_name);
    
    if (!appsrc || !appsink) {
        std::cerr << "THETA " << camera_id << ": Failed to get elements" << std::endl;
        return false;
    }
    
    // Set appsrc caps
    GstCaps *caps = gst_caps_new_simple("video/x-h264",
        "stream-format", G_TYPE_STRING, "byte-stream",
        "alignment", G_TYPE_STRING, "au",
        "width", G_TYPE_INT, 1920,
        "height", G_TYPE_INT, 960,
        "framerate", GST_TYPE_FRACTION, 29, 1,
        NULL);
    gst_app_src_set_caps(GST_APP_SRC(appsrc), caps);
    gst_caps_unref(caps);
    
    g_object_set(G_OBJECT(appsrc),
                 "stream-type", 0,
                 "is-live", TRUE,
                 "format", GST_FORMAT_TIME,
                 NULL);
    
    // Set appsink caps
    GstCaps *sink_caps = gst_caps_new_simple("image/jpeg", NULL);
    gst_app_sink_set_caps(GST_APP_SINK(appsink), sink_caps);
    gst_caps_unref(sink_caps);
    
    g_object_set(appsink, 
                 "max-buffers", 2,
                 "drop", TRUE,
                 "sync", FALSE,
                 NULL);
    
    // Set callback
    GstAppSinkCallbacks callbacks = {NULL};
    callbacks.new_sample = on_new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &callbacks, this, NULL);
    
    // Set up bus
    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    bus_watch_id = gst_bus_add_watch(bus, gst_bus_cb, this);
    gst_object_unref(bus);
    
    std::cout << "THETA " << camera_id << ": GStreamer pipeline initialized" << std::endl;
    return true;
}

bool ThetaCamera::initialize(uvc_context_t *uvc_ctx, uvc_device_t *device, int cam_id) {
    camera_id = cam_id;
    dev = device;
    
    // Get device descriptor for serial
    uvc_device_descriptor_t *desc;
    if (uvc_get_device_descriptor(dev, &desc) == UVC_SUCCESS) {
        if (desc->serialNumber) {
            strncpy(serial, desc->serialNumber, sizeof(serial) - 1);
        } else {
            snprintf(serial, sizeof(serial), "THETA_UNKNOWN_%d", camera_id);
        }
        uvc_free_device_descriptor(desc);
    }
    
    // Open device
    uvc_error_t res = uvc_open(dev, &devh);
    if (res < 0) {
        std::cerr << "THETA " << camera_id << ": Failed to open device" << std::endl;
        uvc_perror(res, "uvc_open");
        return false;
    }
    
    std::cout << "THETA " << camera_id << ": Device opened successfully" << std::endl;
    
    // Set format
    res = uvc_get_stream_ctrl_format_size(devh, &ctrl, UVC_FRAME_FORMAT_H264, 1920, 960, 29);
    if (res < 0) {
        std::cerr << "THETA " << camera_id << ": Failed to set format" << std::endl;
        uvc_perror(res, "uvc_get_stream_ctrl_format_size");
        uvc_close(devh);
        return false;
    }
    
    dwFrameInterval = ctrl.dwFrameInterval;
    
    // Initialize GStreamer
    if (!init_gstreamer()) {
        std::cerr << "THETA " << camera_id << ": Failed to init GStreamer" << std::endl;
        uvc_close(devh);
        return false;
    }
    
    // Start pipeline
    GstStateChangeReturn state_ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (state_ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "THETA " << camera_id << ": Failed to start pipeline" << std::endl;
        gst_element_set_state(pipeline, GST_STATE_NULL);
        uvc_close(devh);
        return false;
    }
    
    std::cout << "THETA " << camera_id << ": Initialized successfully" << std::endl;
    std::cout << "  - Serial Number: " << serial << std::endl;
    std::cout << "  - Resolution: 1920x960 @ 29fps" << std::endl;
    std::cout << "  - Camera ID: " << camera_id << " (cam" << camera_id << ")" << std::endl;
    return true;
}

void ThetaCamera::start_streaming() {
    if (!devh || is_streaming) return;

    is_active = true;
    stream_thread = std::thread([this]() {
        std::cout << "THETA " << camera_id << ": Starting UVC streaming..." << std::endl;

        // Stagger start to avoid USB bandwidth issues
        int delay_ms = camera_id * 1000;
        if (delay_ms > 0) {
            std::cout << "THETA " << camera_id << ": Waiting " << delay_ms << "ms before starting..." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

        uvc_error_t res = uvc_start_streaming(devh, &ctrl, uvc_callback, this, 0);
        if (res < 0) {
            std::cerr << "THETA " << camera_id << ": Failed to start streaming" << std::endl;
            uvc_perror(res, "uvc_start_streaming");
            is_active = false;
            return;
        }

        is_streaming = true;
        std::cout << "THETA " << camera_id << ": UVC streaming started successfully (Serial: " << serial << ")" << std::endl;

        // Monitor streaming health
        uint32_t last_frames = 0, last_samples = 0;
        auto last_check = std::chrono::steady_clock::now();

        while (is_active) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Check every 5 seconds
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_check);
            if (elapsed.count() >= 5) {
                uint32_t current_frames = frames_received.load();
                uint32_t current_samples = samples_received.load();

                std::cout << "THETA " << camera_id << " health check:" << std::endl;
                std::cout << "  - UVC frames received: " << current_frames
                          << " (+" << (current_frames - last_frames) << " in last 5s)" << std::endl;
                std::cout << "  - Decoded samples: " << current_samples
                          << " (+" << (current_samples - last_samples) << " in last 5s)" << std::endl;

                if (current_frames == last_frames && current_frames > 0) {
                    std::cerr << "  ⚠ WARNING: No new UVC frames! Camera may have stopped streaming." << std::endl;
                } else if (current_frames > last_frames && current_samples == last_samples) {
                    std::cerr << "  ⚠ WARNING: Receiving UVC frames but GStreamer decode failing!" << std::endl;
                } else if (current_frames == 0) {
                    std::cerr << "  ⚠ WARNING: No UVC frames at all! Check USB connection." << std::endl;
                }

                last_frames = current_frames;
                last_samples = current_samples;
                last_check = now;
            }
        }
        
        std::cout << "THETA " << camera_id << ": Stopping UVC streaming..." << std::endl;
        uvc_stop_streaming(devh);
        is_streaming = false;
    });
}

ThetaFrame ThetaCamera::get_latest_frame() {
    std::lock_guard<std::mutex> lock(capture_mutex);
    ThetaFrame frame = latest_frame;
    // Mark as invalid after reading to prevent re-use
    // This ensures each frame is used only once in synchronization
    latest_frame.valid = false;
    return frame;
}

void ThetaCamera::cleanup() {
    is_active = false;
    
    if (stream_thread.joinable()) {
        stream_thread.join();
    }
    
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        if (bus_watch_id > 0) {
            g_source_remove(bus_watch_id);
        }
        gst_object_unref(pipeline);
        pipeline = nullptr;
    }
    
    if (devh) {
        uvc_close(devh);
        devh = nullptr;
    }
    
    std::cout << "THETA " << camera_id << ": Cleaned up" << std::endl;
}