#!/usr/bin/env python3
"""
Parallel Loop Control - Using parallel_trigger_module

This version uses the compiled Python module for better performance.
Falls back to direct pipe writing if module is not available.

Usage:
    python3 parallel_loop_control_with_module.py
"""

import time
import sys

# Try to import the compiled module, fall back to direct method
try:
    import parallel_trigger_module
    USE_MODULE = True
    print("Using compiled parallel_trigger_module")
except ImportError:
    USE_MODULE = False
    print("parallel_trigger_module not found, using direct pipe method")
    print("To build the module: make python-parallel-module")
    import os

def send_trigger_direct(mode="both", pipe_path="/tmp/multi_sensor_parallel_trigger_multi_sensor_parallel"):
    """Direct pipe writing fallback"""
    try:
        if not os.path.exists(pipe_path):
            return False
        with open(pipe_path, 'w') as pipe:
            message = f"SAVE_{mode.upper()}\n"
            pipe.write(message)
            pipe.flush()
        return True
    except Exception as e:
        print(f"Error: {e}")
        return False

def send_trigger(mode="both"):
    """Send trigger using module or fallback"""
    if USE_MODULE:
        try:
            parallel_trigger_module.save(mode=mode)
            return True
        except Exception as e:
            print(f"Module error: {e}")
            return False
    else:
        return send_trigger_direct(mode=mode)

def main():
    # Configuration
    frequency_hz = 5.0
    mode = "both"
    interval = 1.0 / frequency_hz

    print("=" * 60)
    print("Parallel Loop Control (with module support)")
    print("=" * 60)
    print(f"Frequency: {frequency_hz} Hz")
    print(f"Mode: {mode}")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    print()

    trigger_count = 0
    fail_count = 0
    start_time = time.time()

    try:
        while True:
            loop_start = time.perf_counter()

            if send_trigger(mode=mode):
                trigger_count += 1
                if trigger_count % 10 == 0:  # Report every 10 triggers
                    elapsed = time.time() - start_time
                    actual_freq = trigger_count / elapsed if elapsed > 0 else 0
                    print(f"[{trigger_count:4d}] {elapsed:.1f}s elapsed, "
                          f"actual: {actual_freq:.2f} Hz")
            else:
                fail_count += 1
                print(f"[FAIL] Total failures: {fail_count}")
                time.sleep(1)
                continue

            elapsed = time.perf_counter() - loop_start
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("Stopped by user")
        elapsed_time = time.time() - start_time
        print(f"Triggers sent: {trigger_count}")
        print(f"Failures: {fail_count}")
        print(f"Time: {elapsed_time:.1f}s")
        if trigger_count > 0:
            print(f"Avg frequency: {trigger_count / elapsed_time:.2f} Hz")
        print("=" * 60)
        return 0

if __name__ == "__main__":
    sys.exit(main())
