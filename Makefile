CXX = g++
CXXFLAGS = -std=c++17 -pthread -O2

# GStreamer / LibUVC flags
GST_CFLAGS  = $(shell pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0 libuvc)
GST_LIBS    = $(shell pkg-config --libs gstreamer-1.0 gstreamer-app-1.0 libuvc)

# OpenCV (try opencv4 first, fallback to opencv)
OPENCV_CFLAGS = $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv)
OPENCV_LIBS   = $(shell pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv)

# RealSense (optional)
REALSENSE_CFLAGS = $(shell pkg-config --cflags realsense2 2>/dev/null || echo "")
REALSENSE_LIBS   = $(shell pkg-config --libs realsense2 2>/dev/null || echo "-lrealsense2")

# Livox SDK
LIVOX_INCLUDE = -I/usr/local/include
LIVOX_LIB = -L/usr/local/lib -llivox_lidar_sdk_shared

# Pybind11 (for Python bindings)
PYBIND11_CFLAGS = $(shell python3.10 -m pybind11 --includes)
PYBIND11_LDFLAGS = -shared

# Python config
PYTHON_CFLAGS = $(shell python3.10-config --includes)
PYTHON_LIBS = $(shell python3.10-config --libs)

# Combined flags
ALL_CFLAGS = $(CXXFLAGS) $(GST_CFLAGS) $(OPENCV_CFLAGS) $(REALSENSE_CFLAGS) $(LIVOX_INCLUDE)
ALL_LIBS   = $(GST_LIBS) $(OPENCV_LIBS) $(REALSENSE_LIBS) $(LIVOX_LIB) -lpthread

# Source and build structure
SRC_DIR = src
OBJ_DIR = build
INCLUDES = -I$(SRC_DIR)

# Core source files (shared)
CORE_SOURCES = $(SRC_DIR)/multi_sensor_synchronizer.cpp \
               $(SRC_DIR)/theta_camera.cpp \
               $(SRC_DIR)/intel_d405.cpp \
               $(SRC_DIR)/livox_mid360.cpp \
               $(SRC_DIR)/npy_writer.cpp \
               $(SRC_DIR)/pcd_writer.cpp

CORE_OBJECTS = $(CORE_SOURCES:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

# Parallel recorder source files
PARALLEL_SOURCES = $(SRC_DIR)/multi_sensor_parallel.cpp \
                   $(SRC_DIR)/theta_camera.cpp \
                   $(SRC_DIR)/intel_d405.cpp \
                   $(SRC_DIR)/livox_mid360.cpp \
                   $(SRC_DIR)/npy_writer.cpp \
                   $(SRC_DIR)/pcd_writer.cpp

PARALLEL_OBJECTS = $(PARALLEL_SOURCES:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/parallel_%.o)

# Executables
DAEMON_TARGET = sync_daemon
TRIGGER_TARGET = trigger_save
PARALLEL_TARGET = parallel_recorder
PARALLEL_TRIGGER_TARGET = parallel_save
PYTHON_MODULE = trigger_module$(shell python3.10-config --extension-suffix)
PARALLEL_PYTHON_MODULE = parallel_trigger_module$(shell python3.10-config --extension-suffix)

DAEMON_MAIN = $(SRC_DIR)/sync_daemon.cpp
TRIGGER_MAIN = $(SRC_DIR)/trigger_save.cpp
PARALLEL_MAIN = $(SRC_DIR)/parallel_main.cpp
PARALLEL_TRIGGER_MAIN = $(SRC_DIR)/parallel_save.cpp
PYTHON_MAIN = $(SRC_DIR)/trigger_module.cpp
PARALLEL_PYTHON_MAIN = $(SRC_DIR)/parallel_trigger_module.cpp

DAEMON_OBJ = $(OBJ_DIR)/sync_daemon.o
TRIGGER_OBJ = $(OBJ_DIR)/trigger_save.o
PARALLEL_OBJ = $(OBJ_DIR)/parallel_main.o
PARALLEL_TRIGGER_OBJ = $(OBJ_DIR)/parallel_save.o

# --- Arms variant (additional, leaves originals untouched) ---
ARMS_SOURCES = $(SRC_DIR)/multi_sensor_parallel_arms.cpp \
               $(SRC_DIR)/theta_camera.cpp \
               $(SRC_DIR)/intel_d405.cpp \
               $(SRC_DIR)/livox_mid360.cpp \
               $(SRC_DIR)/npy_writer.cpp \
               $(SRC_DIR)/pcd_writer.cpp
ARMS_OBJECTS = $(ARMS_SOURCES:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/arms_%.o)

ARMS_TARGET = parallel_arms_recorder
ARMS_MAIN = $(SRC_DIR)/parallel_main_arms.cpp
ARMS_MAIN_OBJ = $(OBJ_DIR)/parallel_main_arms.o

ARMS_PYTHON_MODULE = arms_trigger_module$(shell python3.10-config --extension-suffix)
ARMS_PYTHON_MAIN = $(SRC_DIR)/arms_trigger_module.cpp

# Default target
all: $(DAEMON_TARGET) $(TRIGGER_TARGET) $(PARALLEL_TARGET) $(PARALLEL_TRIGGER_TARGET) python-module $(ARMS_TARGET) arms-python-module

# Build directories
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

# Daemon executable
$(DAEMON_TARGET): $(DAEMON_OBJ) $(CORE_OBJECTS) | $(OBJ_DIR)
	@echo "Linking $@..."
	$(CXX) $^ -o $@ $(ALL_LIBS)

# Trigger executable (lightweight, no sensor libs)
$(TRIGGER_TARGET): $(TRIGGER_OBJ) | $(OBJ_DIR)
	@echo "Linking $@..."
	$(CXX) $< -o $@ -lpthread

# Parallel trigger executable (lightweight, no sensor libs)
$(PARALLEL_TRIGGER_TARGET): $(PARALLEL_TRIGGER_OBJ) | $(OBJ_DIR)
	@echo "Linking $@..."
	$(CXX) $< -o $@ -lpthread

# Parallel recorder executable
$(PARALLEL_TARGET): $(PARALLEL_OBJ) $(OBJ_DIR)/parallel_multi_sensor_parallel.o $(OBJ_DIR)/parallel_theta_camera.o $(OBJ_DIR)/parallel_intel_d405.o $(OBJ_DIR)/parallel_livox_mid360.o $(OBJ_DIR)/parallel_npy_writer.o $(OBJ_DIR)/parallel_pcd_writer.o | $(OBJ_DIR)
	@echo "Linking $@..."
	$(CXX) $^ -o $@ $(ALL_LIBS)

# Python module (pybind11)
$(PYTHON_MODULE): $(PYTHON_MAIN) | $(OBJ_DIR)
	@echo "Building Python module: $@..."
	$(CXX) $(CXXFLAGS) -fPIC $(PYBIND11_CFLAGS) $(PYTHON_CFLAGS) $< -o $@ $(PYBIND11_LDFLAGS) $(PYTHON_LIBS)

# Parallel Python module (pybind11)
$(PARALLEL_PYTHON_MODULE): $(PARALLEL_PYTHON_MAIN) | $(OBJ_DIR)
	@echo "Building Parallel Python module: $@..."
	$(CXX) $(CXXFLAGS) -fPIC $(PYBIND11_CFLAGS) $(PYTHON_CFLAGS) $< -o $@ $(PYBIND11_LDFLAGS) $(PYTHON_LIBS)

.PHONY: python-module python-parallel-module arms-python-module
python-module: $(PYTHON_MODULE)
python-parallel-module: $(PARALLEL_PYTHON_MODULE)
arms-python-module: $(ARMS_PYTHON_MODULE)

# Arms recorder executable
$(ARMS_TARGET): $(ARMS_MAIN_OBJ) $(ARMS_OBJECTS) | $(OBJ_DIR)
	@echo "Linking $@..."
	$(CXX) $^ -o $@ $(ALL_LIBS)

# Arms python module
$(ARMS_PYTHON_MODULE): $(ARMS_PYTHON_MAIN) | $(OBJ_DIR)
	@echo "Building Arms Python module: $@..."
	$(CXX) $(CXXFLAGS) -fPIC $(PYBIND11_CFLAGS) $(PYTHON_CFLAGS) $< -o $@ $(PYBIND11_LDFLAGS) $(PYTHON_LIBS)

# Arms object compilation (arms_ prefix to avoid conflicts with existing object files)
$(OBJ_DIR)/arms_%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	@echo "Compiling $< (arms)..."
	$(CXX) $(ALL_CFLAGS) $(INCLUDES) -c $< -o $@

$(OBJ_DIR)/parallel_main_arms.o: $(ARMS_MAIN) | $(OBJ_DIR)
	@echo "Compiling $<..."
	$(CXX) $(ALL_CFLAGS) $(INCLUDES) -c $< -o $@

# Compile core objects
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	@echo "Compiling $<..."
	$(CXX) $(ALL_CFLAGS) $(INCLUDES) -c $< -o $@

# Compile main files
$(OBJ_DIR)/sync_daemon.o: $(DAEMON_MAIN) | $(OBJ_DIR)
	@echo "Compiling $<..."
	$(CXX) $(ALL_CFLAGS) $(INCLUDES) -c $< -o $@

$(OBJ_DIR)/trigger_save.o: $(TRIGGER_MAIN) | $(OBJ_DIR)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/parallel_save.o: $(PARALLEL_TRIGGER_MAIN) | $(OBJ_DIR)
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/parallel_main.o: $(PARALLEL_MAIN) | $(OBJ_DIR)
	@echo "Compiling $<..."
	$(CXX) $(ALL_CFLAGS) $(INCLUDES) -c $< -o $@

# Compile parallel objects (with parallel_ prefix to avoid conflicts)
$(OBJ_DIR)/parallel_%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	@echo "Compiling $< (parallel)..."
	$(CXX) $(ALL_CFLAGS) $(INCLUDES) -c $< -o $@

# Clean
clean:
	rm -rf $(OBJ_DIR) $(DAEMON_TARGET) $(TRIGGER_TARGET) $(PARALLEL_TARGET) $(PARALLEL_TRIGGER_TARGET) $(PYTHON_MODULE) $(PARALLEL_PYTHON_MODULE) $(ARMS_TARGET) $(ARMS_PYTHON_MODULE)

# Run
run-daemon: $(DAEMON_TARGET)
	./$(DAEMON_TARGET)

run-trigger: $(TRIGGER_TARGET)
	./$(TRIGGER_TARGET)

run-parallel: $(PARALLEL_TARGET)
	./$(PARALLEL_TARGET)

run-parallel-save: $(PARALLEL_TRIGGER_TARGET)
	./$(PARALLEL_TRIGGER_TARGET)

# Debug flags
test-flags:
	@echo "GStreamer CFLAGS: $(GST_CFLAGS)"
	@echo "OpenCV CFLAGS:    $(OPENCV_CFLAGS)"
	@echo "RealSense CFLAGS: $(REALSENSE_CFLAGS)"
	@echo "Livox INCLUDE:    $(LIVOX_INCLUDE)"
	@echo "Pybind11 CFLAGS:  $(PYBIND11_CFLAGS)"
	@echo "Python CFLAGS:    $(PYTHON_CFLAGS)"
	@echo "Python LIBS:      $(PYTHON_LIBS)"
	@echo "ALL_CFLAGS:       $(ALL_CFLAGS)"

.PHONY: all clean run-daemon run-trigger run-parallel test-flags python-module