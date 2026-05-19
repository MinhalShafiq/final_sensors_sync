#!/usr/bin/env python3
"""
Episode recorder - dual Piper arm control + synchronized multi-sensor capture.

Owns the C_PiperInterface_V2 (replaces dual_piper.py during data-collection
sessions). Spawns parallel_arms_recorder once at startup so sensors warm up
exactly one time. Press SPACE to toggle recording on/off; press q to quit.

Per episode the script writes a high-rate arms_trajectory_epNNN.csv (arm state
sampled at the control-loop rate, ~200 Hz). At ~5 Hz a SAVE trigger is sent to
the recorder with the current arm state inlined; the recorder writes one
parallel_data.csv row per trigger with sensor file paths + arm columns + the
episode_id, so episodes can be reconstructed by joining both CSVs on
(episode_id, save_id) or on timestamp.

Usage:
    python3 episode_recorder.py --can can0
"""

import argparse
import os
import select
import signal
import subprocess
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DUAL_PIPER_ROOT = REPO_ROOT.parent / "dual_piper_control"

sys.path.insert(0, str(REPO_ROOT))          # for arms_trigger_module
sys.path.insert(0, str(DUAL_PIPER_ROOT))    # for piper_sdk

from piper_sdk import C_PiperInterface_V2
import arms_trigger_module


# ---- tunables --------------------------------------------------------------

LOOP_RATE_HZ = 200                  # arm read + master/slave maintenance
TRIGGER_RATE_HZ = 5                 # sensor save rate
ENABLE_REFRESH_PERIOD = 100         # every N loop iters, re-send enable (0.5s @ 200Hz)
ENABLE_TIMEOUT = 10                 # seconds to wait for slave enable

LOOP_PERIOD = 1.0 / LOOP_RATE_HZ
TRIGGER_EVERY_N = max(1, LOOP_RATE_HZ // TRIGGER_RATE_HZ)


# ---- piper helpers (mirrored from dual_piper.py) ---------------------------

def is_arm_enabled(piper):
    msgs = piper.GetArmLowSpdInfoMsgs()
    return all([
        msgs.motor_1.foc_status.driver_enable_status,
        msgs.motor_2.foc_status.driver_enable_status,
        msgs.motor_3.foc_status.driver_enable_status,
        msgs.motor_4.foc_status.driver_enable_status,
        msgs.motor_5.foc_status.driver_enable_status,
        msgs.motor_6.foc_status.driver_enable_status,
    ])


def enable_slave(piper):
    print("  Enabling slave arm...")
    start = time.time()
    while time.time() - start < ENABLE_TIMEOUT:
        piper.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
        piper.EnableArm(7)
        piper.GripperCtrl(0, 1000, 0x01, 0)
        time.sleep(0.5)
        if is_arm_enabled(piper):
            print("  Slave arm enabled (high-follow mode).")
            return True
    print("  WARNING: Slave enable timed out!")
    return False


def read_arm_state(piper, mg):
    """Return (master_arm_vec[8], slave_arm_vec[8]) as floats.

    Vector layout: j1..j6 (raw SDK units), grip_angle, grip_effort. `mg` is the
    already-fetched GetArmGripperCtrl().gripper_ctrl, passed in to avoid a
    duplicate CAN-state read every loop.
    """
    mc = piper.GetArmJointCtrl().joint_ctrl
    sj = piper.GetArmJointMsgs().joint_state
    sg = piper.GetArmGripperMsgs().gripper_state

    master = [
        float(mc.joint_1), float(mc.joint_2), float(mc.joint_3),
        float(mc.joint_4), float(mc.joint_5), float(mc.joint_6),
        float(mg.grippers_angle), float(mg.grippers_effort),
    ]
    slave = [
        float(sj.joint_1), float(sj.joint_2), float(sj.joint_3),
        float(sj.joint_4), float(sj.joint_5), float(sj.joint_6),
        float(sg.grippers_angle), float(sg.grippers_effort),
    ]
    return master, slave


# ---- keyboard (non-blocking, cbreak) ---------------------------------------

class KeyPoller:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.original = None

    def __enter__(self):
        if sys.stdin.isatty():
            self.original = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        if self.original is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original)

    def poll(self):
        if not sys.stdin.isatty():
            return None
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            return sys.stdin.read(1)
        return None


# ---- recorder subprocess management ---------------------------------------

def spawn_recorder(base_dir: Path, base_name: str, log_path: Path):
    """Start parallel_arms_recorder pointing at base_dir; return Popen."""
    binary = REPO_ROOT / "parallel_arms_recorder"
    if not binary.exists():
        raise FileNotFoundError(f"Recorder binary not found: {binary}. Run `make parallel_arms_recorder`.")
    cmd = [str(binary), "-n", base_name, "-D", str(base_dir)]
    log_f = open(log_path, "w")
    proc = subprocess.Popen(
        cmd,
        cwd=str(REPO_ROOT),
        stdout=log_f,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    return proc, log_f


def wait_for_pipe(pipe_path: Path, timeout: float = 30.0) -> bool:
    """Wait until the recorder creates its trigger pipe (signal it's ready)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pipe_path.exists():
            # Sensors still warming up in C++ side ~3s after pipe appears, give it a moment
            time.sleep(3.5)
            return True
        time.sleep(0.1)
    return False


# ---- arms CSV writer -------------------------------------------------------

ARMS_CSV_HEADER = (
    "loop_index,wall_time,monotonic,episode_id,"
    "master_j1,master_j2,master_j3,master_j4,master_j5,master_j6,"
    "master_grip_angle,master_grip_effort,"
    "slave_j1,slave_j2,slave_j3,slave_j4,slave_j5,slave_j6,"
    "slave_grip_angle,slave_grip_effort\n"
)


def open_arms_csv(base_dir: Path, episode_id: int):
    path = base_dir / f"arms_trajectory_ep{episode_id:03d}.csv"
    f = open(path, "w", buffering=1)  # line-buffered
    f.write(ARMS_CSV_HEADER)
    return f, path


# ---- main ------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Episode recorder for dual Piper + multi-sensor sync")
    parser.add_argument("--can", default="can0", help="CAN interface (default: can0)")
    parser.add_argument("--name", default="multi_sensor_parallel_arms",
                        help="Base name (must match parallel_arms_recorder's -n flag)")
    parser.add_argument("--out", default=None,
                        help="Where arm trajectory CSVs go. If --spawn, also passed to "
                             "the recorder as -D. Default: ./<name>_<timestamp> in repo root.")
    parser.add_argument("--spawn", action="store_true",
                        help="Launch parallel_arms_recorder ourselves. Default: expect "
                             "the recorder to already be running in another terminal.")
    args = parser.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.out:
        base_dir = Path(args.out).resolve()
    else:
        base_dir = (REPO_ROOT / f"{args.name}_{timestamp}").resolve()
    base_dir.mkdir(parents=True, exist_ok=True)

    pipe_path = Path(f"/tmp/multi_sensor_parallel_arms_trigger_{args.name}")
    recorder_log = base_dir / "recorder.log"

    print("============================================")
    print("  Episode Recorder")
    print("============================================")
    print(f"  CAN bus:      {args.can}")
    print(f"  Output:       {base_dir}")
    print(f"  Trigger pipe: {pipe_path}")
    print(f"  Recorder:     {'spawn (self-managed)' if args.spawn else 'external (expected)'}")
    print("============================================\n")

    # 1) Handle the recorder: spawn or attach
    recorder_proc = None
    recorder_log_f = None
    pipe_exists = pipe_path.exists()

    if args.spawn:
        if pipe_exists:
            print(f"  ERROR: --spawn was given but pipe already exists at {pipe_path}.")
            print("         An external recorder is already running. Drop --spawn or kill the other one.")
            sys.exit(1)
        print(f"  Starting parallel_arms_recorder (logs -> {recorder_log})...")
        recorder_proc, recorder_log_f = spawn_recorder(base_dir, args.name, recorder_log)
    else:
        if not pipe_exists:
            print(f"  Pipe {pipe_path} not found yet — waiting up to 60s.")
            print(f"  Start in another terminal: ./parallel_arms_recorder -n {args.name} -D {base_dir}")

    # 2) Bring up piper while the recorder warms up
    piper = C_PiperInterface_V2(args.can)
    piper.ConnectPort()
    time.sleep(2)

    if not enable_slave(piper):
        print("Failed to enable slave. Check connections and power.")
        piper.DisconnectPort()
        if recorder_proc:
            recorder_proc.terminate()
            recorder_log_f.close()
        sys.exit(1)

    print("  Activating high-follow mode...")
    for _ in range(20):
        piper.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
        piper.EnableArm(7)
        time.sleep(0.05)

    # 3) Make sure the pipe is up before we start triggering
    if not wait_for_pipe(pipe_path, timeout=60.0):
        print(f"  ERROR: trigger pipe never appeared at {pipe_path}.")
        if recorder_proc:
            print(f"         See recorder log: {recorder_log}")
            recorder_proc.terminate()
            recorder_log_f.close()
        else:
            print("         Is parallel_arms_recorder running? Check `-n` matches `--name`.")
        piper.DisconnectPort()
        sys.exit(1)
    print("  Trigger pipe ready.\n")

    # 3) Episode state
    recording = False
    episode_id = 0      # incremented at every press of SPACE to start an episode
    arms_csv_f = None
    arms_csv_path = None
    loop_count = 0
    save_count_for_episode = 0
    quit_flag = False

    print("=== READY ===")
    print("  SPACE = start/stop episode")
    print("  q     = quit")
    print()

    def stop_episode():
        nonlocal arms_csv_f, arms_csv_path, recording, save_count_for_episode
        if not recording:
            return
        recording = False
        if arms_csv_f is not None:
            arms_csv_f.close()
            print(f"\n[episode {episode_id}] stopped. arm rows -> {arms_csv_path}, sensor saves: {save_count_for_episode}\n")
        arms_csv_f = None
        arms_csv_path = None
        save_count_for_episode = 0

    def start_episode():
        nonlocal arms_csv_f, arms_csv_path, recording, episode_id, save_count_for_episode
        if recording:
            return
        episode_id += 1
        save_count_for_episode = 0
        arms_csv_f, arms_csv_path = open_arms_csv(base_dir, episode_id)
        recording = True
        print(f"\n[episode {episode_id}] recording -> {arms_csv_path}\n")

    def handle_sigterm(_sig, _frm):
        nonlocal quit_flag
        quit_flag = True

    signal.signal(signal.SIGTERM, handle_sigterm)

    start_wall = time.time()
    start_monotonic = time.monotonic()

    try:
        with KeyPoller() as keys:
            while not quit_flag:
                loop_count += 1
                loop_start = time.perf_counter()

                # --- master/slave maintenance ---
                if loop_count % ENABLE_REFRESH_PERIOD == 0:
                    piper.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
                    piper.EnableArm(7)

                # Gripper relay every loop — identical to dual_piper.py. No
                # try/except: if the SDK raises here, we want a real traceback,
                # not a silent slave that drifts.
                mg = piper.GetArmGripperCtrl().gripper_ctrl
                grip_effort = mg.grippers_effort or 3000
                piper.GripperCtrl(abs(mg.grippers_angle), grip_effort, 0x01, 0)

                # Reuse `mg` for the arm-state read so we don't poll the
                # gripper control twice per loop.
                master, slave = read_arm_state(piper, mg)

                # --- high-rate arms log ---
                if recording and arms_csv_f is not None:
                    now_mono = time.monotonic() - start_monotonic
                    now_wall = time.time()
                    arms_csv_f.write(
                        f"{loop_count},{now_wall:.6f},{now_mono:.6f},{episode_id},"
                        + ",".join(f"{v:.6f}" for v in master) + ","
                        + ",".join(f"{v:.6f}" for v in slave) + "\n"
                    )

                # --- 5 Hz sensor trigger ---
                if recording and loop_count % TRIGGER_EVERY_N == 0:
                    try:
                        arms_trigger_module.save(
                            mode="both",
                            episode_id=episode_id,
                            master_arm=master,
                            slave_arm=slave,
                            pipe_path=str(pipe_path),
                        )
                        save_count_for_episode += 1
                    except Exception as e:
                        print(f"  [warn] trigger send: {e}")

                # --- keyboard ---
                key = keys.poll()
                if key is not None:
                    if key == " ":
                        if recording:
                            stop_episode()
                        else:
                            start_episode()
                    elif key.lower() == "q":
                        print("\n  quit requested.")
                        break

                # --- pacing ---
                elapsed = time.perf_counter() - loop_start
                sleep_for = LOOP_PERIOD - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)

    except KeyboardInterrupt:
        print("\n  KeyboardInterrupt.")
    finally:
        if recording:
            stop_episode()

        if recorder_proc is not None:
            print("\n  Stopping recorder...")
            try:
                os.killpg(os.getpgid(recorder_proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                recorder_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                print("  Recorder didn't exit on SIGINT; sending SIGTERM.")
                os.killpg(os.getpgid(recorder_proc.pid), signal.SIGTERM)
                recorder_proc.wait(timeout=5)
            if recorder_log_f is not None:
                recorder_log_f.close()
        else:
            print("\n  Leaving external recorder running.")

        print("  Disconnecting piper...")
        piper.DisconnectPort()
        elapsed_total = time.time() - start_wall
        print(f"\n  Session done. Episodes: {episode_id}. Wall time: {elapsed_total:.1f}s")
        print(f"  Data: {base_dir}")


if __name__ == "__main__":
    main()
