#ifndef THETA_CAMERA_H
#define THETA_CAMERA_H

#include "common.h"
#include "sensor_data.h"

class ThetaCamera {
public:
    int camera_id;
    char serial[64];
    
    // UVC components
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;
    
    // GStreamer components
    GstElement *pipeline;
    GstElement *appsrc;
    GstElement *appsink;
    guint bus_watch_id;
    uint32_t dwFrameInterval;
    
    // Synchronization
    std::mutex capture_mutex;
    ThetaFrame latest_frame;
    std::atomic<bool> is_active{false};
    std::atomic<bool> is_streaming{false};
    std::thread stream_thread;
    
    // Stats
    std::atomic<uint32_t> frames_received{0};
    std::atomic<uint32_t> samples_received{0};
    
    ThetaCamera();
    ~ThetaCamera();
    
    double get_current_time();
    bool initialize(uvc_context_t *uvc_ctx, uvc_device_t *device, int cam_id);
    void start_streaming();
    ThetaFrame get_latest_frame();
    void cleanup();
    
private:
    bool init_gstreamer();
    
    // Static callbacks
    static gboolean gst_bus_cb(GstBus *bus, GstMessage *message, gpointer data);
    static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data);
    static void uvc_callback(uvc_frame_t *frame, void *ptr);
};

#endif // THETA_CAMERA_H