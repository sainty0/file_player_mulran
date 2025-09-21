#!/usr/bin/env python3
"""
start_percent_test.py

Verifies that the headless player seeks to the requested --start-percent
position by comparing the first published data stamp after startup to the
expected timestamp computed from sensor_data/data_stamp.csv.

Optionally also launches:
- SC-LIO-SAM via roslaunch (use_sim_time:=true and optional params_file)
- RViz with an optional config file
so you can visually observe playback while the test runs.

Usage examples:
  # Basic functional test (no visualization): seek to 25% and publish one stamp
  python3 tests/start_percent_test.py --dir /full/path/to/sequence --pct 25 --timeout 30

  # Visualize: start SLAM + RViz, keep running for 20s (continuous playback)
  python3 tests/start_percent_test.py --dir /full/path/to/sequence --pct 25 \
    --with-slam --with-rviz --visualize-seconds 20 --keep-playing

  # Visualize with specific params & RViz config, still validate first stamp
  python3 tests/start_percent_test.py --dir /full/path/to/sequence --pct 40 \
    --with-slam --params-file SC-LIO-SAM/SC-LIO-SAM/config/params_mulran.yaml \
    --with-rviz --rviz-config SC-LIO-SAM/SC-LIO-SAM/doc/rviz/rviz.rviz \
    --visualize-seconds 15

Notes:
- Ensure your ROS workspace is sourced and the package is built so that:
    roslaunch lio_sam run_mulran.launch
    rosrun file_player file_player_headless
    rosrun rviz rviz
  are available on PATH.
- If --with-slam is used, roslaunch will start roscore. Otherwise this test
  starts a local roscore for isolation.
"""
import argparse
import subprocess
import sys
import time
import os
from typing import Optional, List

def start_roscore() -> subprocess.Popen:
    p = subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    return p

def terminate(proc: Optional[subprocess.Popen], name: str, gentle_secs: float = 2.0):
    if proc is None:
        return
    try:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=gentle_secs)
            except Exception:
                proc.kill()
    except Exception as e:
        print(f"[WARN] Failed to terminate {name}: {e}", file=sys.stderr)

def start_roslaunch_slam(params_file: Optional[str]) -> subprocess.Popen:
    launch_cmd = ['roslaunch', 'lio_sam', 'run_mulran.launch', 'use_sim_time:=true']
    if params_file:
        launch_cmd.append(f'params_file:={params_file}')
    print("Launching SLAM:", ' '.join(launch_cmd))
    return subprocess.Popen(launch_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def start_player(seq_dir: str, rate: float, pct: float, keep_playing: bool, extra_args: Optional[List[str]] = None) -> subprocess.Popen:
    cmd = ['rosrun', 'file_player', 'file_player_headless',
           '--dir', seq_dir, '--rate', str(rate), '--start-percent', str(pct)]
    if not keep_playing:
        # publish one stamp and exit (default behavior for assertion)
        cmd += ['--step', '1']
    if extra_args:
        cmd += extra_args
    print("Launching player:", ' '.join(cmd))
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def start_rviz(rviz_config: Optional[str]) -> subprocess.Popen:
    cmd = ['rosrun', 'rviz', 'rviz']
    if rviz_config:
        cmd += ['-d', rviz_config]
    print("Launching RViz:", ' '.join(cmd))
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def read_stamp_bounds(seq_dir: str):
    path = os.path.join(seq_dir, 'sensor_data', 'data_stamp.csv')
    if not os.path.exists(path):
        raise RuntimeError(f"data_stamp.csv not found at {path}")
    first = None
    last = None
    with open(path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if not parts:
                continue
            try:
                st = int(parts[0])
            except Exception:
                continue
            if first is None:
                first = st
            last = st
    if first is None or last is None:
        raise RuntimeError("No stamps found in data_stamp.csv")
    return first, last

def run_test(seq_dir, pct, timeout, rate, with_slam, params_file, with_rviz, rviz_config, visualize_seconds, keep_playing):
    roscore_proc = None
    slam_proc = None
    rviz_proc = None
    player_proc = None

    try:
        if with_slam:
            slam_proc = start_roslaunch_slam(params_file)
            time.sleep(5.0)  # let roscore and params initialize
        else:
            roscore_proc = start_roscore()

        # Deferred ROS imports until master is likely up
        try:
            import rospy
            from sensor_msgs.msg import NavSatFix, Imu, PointCloud2, Image
        except Exception as e:
            print("Failed to import rospy or message types. Ensure ROS python packages are installed and sourced.", file=sys.stderr)
            raise

        first_stamp, last_stamp = read_stamp_bounds(seq_dir)
        expected = int(first_stamp + (last_stamp - first_stamp) * (pct / 100.0))

        rospy.init_node('start_percent_test_node', anonymous=True)

        captured = {'stamp': None}
        def cb(msg):
            try:
                captured['stamp'] = msg.header.stamp.to_nsec()
            except Exception:
                pass

        subs = []
        subs.append(rospy.Subscriber('/gps/fix', NavSatFix, cb))
        subs.append(rospy.Subscriber('/imu/data_raw', Imu, cb))
        subs.append(rospy.Subscriber('/os1_points', PointCloud2, cb))
        subs.append(rospy.Subscriber('/radar/polar', Image, cb))

        # Optionally start RViz
        if with_rviz:
            rviz_proc = start_rviz(rviz_config)
            time.sleep(2.0)

        # Start player
        player_proc = start_player(seq_dir, rate, pct, keep_playing)

        start_t = time.time()
        if visualize_seconds > 0:
            end_t = start_t + visualize_seconds
            while time.time() < end_t:
                if captured['stamp'] is not None and not keep_playing:
                    # If we only asked for one step and already received it, we can still keep visualizing
                    pass
                if player_proc.poll() is not None and keep_playing:
                    # Player ended early unexpectedly; break visualization window
                    break
                rospy.sleep(0.05)
        else:
            # No visualization hold; wait for first message or for player to exit/timeout
            while time.time() - start_t < timeout:
                if captured['stamp'] is not None:
                    break
                if player_proc.poll() is not None:
                    # Player exited; break
                    break
                rospy.sleep(0.05)

        # Stop player if still running when not keeping playback
        if player_proc and player_proc.poll() is None and not keep_playing:
            terminate(player_proc, "player")

        # Allow small delay
        time.sleep(0.1)

        result_stamp = captured['stamp']
        if result_stamp is None:
            print("No message received within timeout/visualization window.", file=sys.stderr)
            return 2

        # Compare to expected with 1% tolerance
        span = last_stamp - first_stamp
        tol = max(1, int(span * 0.01))  # 1% tolerance
        diff = abs(result_stamp - expected)

        print(f"Requested percent: {pct}%")
        print(f"First stamp in CSV: {first_stamp}")
        print(f"Last stamp in CSV:  {last_stamp}")
        print(f"Expected stamp:     {expected}")
        print(f"Received stamp:     {result_stamp}")
        print(f"Difference:         {diff} (tolerance {tol})")

        if diff <= tol:
            print("Test passed.")
            return 0
        else:
            print("Test failed: received stamp is outside tolerance.", file=sys.stderr)
            return 2

    finally:
        # Cleanup in reverse order
        if rviz_proc:
            terminate(rviz_proc, "rviz")
        if player_proc:
            terminate(player_proc, "player")
        if slam_proc:
            terminate(slam_proc, "roslaunch SLAM", gentle_secs=5.0)
        if roscore_proc:
            terminate(roscore_proc, "roscore", gentle_secs=2.0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True, help='Sequence directory root (full path)')
    parser.add_argument('--pct', type=float, default=25.0, help='Start percent (0..100)')
    parser.add_argument('--timeout', type=int, default=30, help='Timeout seconds (used when not visualizing)')
    parser.add_argument('--rate', type=float, default=1.0, help='Playback rate multiplier')
    parser.add_argument('--with-slam', action='store_true', help='Also launch SC-LIO-SAM via roslaunch for visualization')
    parser.add_argument('--params-file', default=None, help='Optional params YAML for SC-LIO-SAM (launch arg params_file:=...)')
    parser.add_argument('--with-rviz', action='store_true', help='Launch RViz for visualization')
    parser.add_argument('--rviz-config', default=None, help='Optional RViz config file (-d path)')
    parser.add_argument('--visualize-seconds', type=int, default=0, help='Keep processes alive this many seconds to visualize')
    parser.add_argument('--keep-playing', action='store_true', help='Do not pass --step 1; keep playing continuously (good for visualization)')
    args = parser.parse_args()

    rc = run_test(args.dir, args.pct, args.timeout, args.rate, args.with_slam, args.params_file,
                  args.with_rviz, args.rviz_config, args.visualize_seconds, args.keep_playing)
    sys.exit(rc)
