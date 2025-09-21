#!/usr/bin/env python3
"""
step_test.py

Runs the headless file player in step mode and verifies that it publishes
the requested number of data stamps (counting messages across main topics).
Optionally also launches SC-LIO-SAM (via roslaunch) and RViz so you can
visually observe the playback while the test runs.

Usage examples:
  # Basic functional test (no visualization)
  python3 tests/step_test.py --dir /full/path/to/sequence --step 5 --timeout 30

  # Also run SC-LIO-SAM and RViz, keep them open for 20 seconds to visualize
  python3 tests/step_test.py --dir /full/path/to/sequence --step 50 --with-slam --with-rviz --visualize-seconds 20

  # Inject SC-LIO-SAM params and use a custom RViz config
  python3 tests/step_test.py --dir /full/path/to/sequence --step 100 \
    --with-slam --params-file SC-LIO-SAM/SC-LIO-SAM/config/params_mulran.yaml \
    --with-rviz --rviz-config SC-LIO-SAM/SC-LIO-SAM/doc/rviz/rviz.rviz \
    --visualize-seconds 30

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
import signal
from typing import List, Optional

def start_roscore() -> subprocess.Popen:
    p = subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    return p

def start_roslaunch_slam(params_file: Optional[str]) -> subprocess.Popen:
    launch_cmd = ['roslaunch', 'lio_sam', 'run_mulran.launch', 'use_sim_time:=true']
    if params_file:
        launch_cmd.append(f'params_file:={params_file}')
    # Log to stdout; user can redirect if desired
    return subprocess.Popen(launch_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def start_player(seq_dir: str, rate: float, step_count: int, extra_args: Optional[List[str]] = None) -> subprocess.Popen:
    cmd = ['rosrun', 'file_player', 'file_player_headless', '--dir', seq_dir, '--rate', str(rate)]
    if step_count > 0:
        cmd += ['--step', str(step_count)]
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

def terminate(proc: subprocess.Popen, name: str, gentle_secs: float = 2.0):
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

def run_test(seq_dir, step_count, timeout, with_slam, params_file, with_rviz, rviz_config, visualize_seconds, rate):
    # Start ROS master (roscore) unless roslaunch is going to do it
    roscore_proc = None
    slam_proc = None
    rviz_proc = None
    player_proc = None

    try:
        if with_slam:
            slam_proc = start_roslaunch_slam(params_file)
            # give SLAM some time to spin up roscore and params
            time.sleep(5.0)
        else:
            roscore_proc = start_roscore()

        # Import ROS after master is likely up
        try:
            import rospy
            from sensor_msgs.msg import NavSatFix, Imu, PointCloud2, Image
        except Exception as e:
            print("Failed to import rospy or message types. Ensure ROS python packages are installed and sourced.", file=sys.stderr)
            raise

        rospy.init_node('step_test_node', anonymous=True)

        # Simple message counter across main topics
        counter = {'count': 0}
        last_stamp = {'val': 0}

        def generic_cb(msg):
            counter['count'] += 1
            try:
                hdr = msg.header
                last_stamp['val'] = hdr.stamp.to_nsec()
            except Exception:
                pass

        subs = []
        subs.append(rospy.Subscriber('/gps/fix', NavSatFix, generic_cb))
        subs.append(rospy.Subscriber('/imu/data_raw', Imu, generic_cb))
        subs.append(rospy.Subscriber('/os1_points', PointCloud2, generic_cb))
        subs.append(rospy.Subscriber('/radar/polar', Image, generic_cb))

        # Optionally start RViz for visualization
        if with_rviz:
            rviz_proc = start_rviz(rviz_config)
            time.sleep(2.0)

        # Start the player (headless)
        player_proc = start_player(seq_dir, rate, step_count)

        start_t = time.time()

        if visualize_seconds > 0:
            # During visualization window, just keep spinning and counting messages
            end_t = start_t + visualize_seconds
            while time.time() < end_t:
                if player_proc.poll() is not None:
                    # If player ended early (e.g., steps completed), we still wait for visualization time
                    pass
                rospy.sleep(0.05)
        else:
            # No extra visualization time; wait for player or timeout
            while time.time() - start_t < timeout:
                if player_proc.poll() is not None:
                    break
                rospy.sleep(0.05)

        # If process still running, try to terminate gracefully
        if player_proc and player_proc.poll() is None:
            terminate(player_proc, "player")

        # Allow messages to flush
        time.sleep(0.2)

        received = counter['count']
        print(f"Requested steps: {step_count}, messages received on monitored topics: {received}")
        success = (received >= step_count) or (visualize_seconds > 0 and received > 0)
        # For pure step test: expect >= step_count
        # For visualization mode (no strict step wait), just require some messages to be seen.

        if success:
            print("Test passed.")
            return 0
        else:
            print("Test failed: insufficient messages observed.", file=sys.stderr)
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
    parser.add_argument('--step', type=int, default=1, help='Number of steps to request')
    parser.add_argument('--timeout', type=int, default=30, help='Timeout in seconds')
    parser.add_argument('--rate', type=float, default=1.0, help='Playback rate multiplier')
    parser.add_argument('--with-slam', action='store_true', help='Also launch SC-LIO-SAM via roslaunch for visualization')
    parser.add_argument('--params-file', default=None, help='Optional params YAML for SC-LIO-SAM (launch arg params_file:=...)')
    parser.add_argument('--with-rviz', action='store_true', help='Launch RViz for visualization')
    parser.add_argument('--rviz-config', default=None, help='Optional RViz config file (-d path)')
    parser.add_argument('--visualize-seconds', type=int, default=0, help='Keep processes alive this many seconds to visualize (relaxes strict step assertion)')
    args = parser.parse_args()

    rc = run_test(args.dir, args.step, args.timeout, args.with_slam, args.params_file, args.with_rviz, args.rviz_config, args.visualize_seconds, args.rate)
    sys.exit(rc)
