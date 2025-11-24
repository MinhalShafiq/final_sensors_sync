import time
import trigger_module

def main():
    frequency_hz = 5.0
    interval = 1.0 / frequency_hz
    mode = "both"  # or "numpy", "normal"

    print(f"Starting periodic save at {frequency_hz} Hz (mode: {mode})")
    print("Press Ctrl+C to stop")

    try:
        while True:
            start = time.perf_counter()
            try:
                trigger_module.save(mode=mode)
                print(f"Trigger sent ({mode})")
            except Exception as e:
                print(f"Trigger failed: {e}")
                time.sleep(1)  # retry slower if daemon down
                continue

            elapsed = time.perf_counter() - start
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\nStopped by user")

if __name__ == "__main__":
    main()