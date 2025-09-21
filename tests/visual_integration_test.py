#!/usr/bin/env python3
"""
visual_integration_test.py

Launches SC-LIO-SAM (roslaunch), the MulRan headless file player, and RViz,
then performs a basic functional verification:
- Step mode: ensure at least N messages are observed across key topics, or
- Start-percent mode: verify first published stamp is near expected position.

This script is designed to avoid "master not running" issues by launching
roslaunch (which starts roscore), waiting for the master to be available,
then starting RViz and the player.

Usage examples:
  # Step 50 data-stamp entries and watch for 20 seconds
  python3 file_player_mulran/tests/visual_integration_test.py \
    --dir /data/mulran/KAIST01 \
    --step 50 \
    --params-file SC-LIO-SAM/SC-LIO-SAM/config/params_mulran.yaml \
    --rviz-config SC-LIO-SAM/SC-LIO-SAM/doc/rviz/rviz.rviz \
    --visualize-seconds 20

  # Seek to 25% and keep playing for 30 seconds
  python3 file_player_mulran/tests/visual_integration_test.py \
    --dir /data/mulran/KAIST01 \
    --pct 25 \
    --keep-playing \
    --params-file SC-LIO-SAM/SC-LIO-SAM/config/params_mulran.yaml \
    --rviz-config SC-LIO-SAM/SC-LIO-SAM/doc/rviz/rviz.rviz \
    --visualize-seconds 30

Notes:
- Ensure your ROS workspace is built and sourced so these commands exist:
    roslaunch lio_sam run_mulran.launch
    rosrun file_player file_player_headless
    rosrun rviz rviz
- This script always launches SLAM via roslaunch (with use_sim_time:=true).
- RViz is launched by default; use --no-rviz to disable.
"""

import argparse
import os
import subprocess
import sys
import time
from typing import Optional

def start_roscore() -> subprocess.Popen:
    p = subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    return p

def terminate(proc: Optional[subprocess.Popen], name: str, gentle_secs: float = 3.0):
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

def wait_for_master(timeout: float = 20.0) -> bool:
    """Wait for ROS master to be online."""
    try:
        import rosgraph
    except Exception:
        # Fallback: simple sleep if rosgraph not importable
        time.sleep(3.0)
        return True
    start = time.time()
    while time.time() - start < timeout:
        if rosgraph.is_master_online():
            return True
        time.sleep(0.25)
    return False

def start_slam(params_file: Optional[str]) -> subprocess.Popen:
    """Launch SC-LIO-SAM; roslaunch starts roscore."""
    cmd = ['roslaunch', 'lio_sam', 'run_mulran.launch', 'use_sim_time:=true']
    if params_file:
        cmd.append(f'params_file:={params_file}')
    print("Launching SLAM:", ' '.join(cmd))
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def start_rviz(rviz_config: Optional[str]) -> subprocess.Popen:
    # Use bundled default RViz config if none provided
    if not rviz_config:
        rviz_config = os.path.join(os.path.dirname(__file__), 'rviz_mulran_demo.rviz')
    cmd = ['rosrun', 'rviz', 'rviz', '-d', rviz_config]
    print("Launching RViz:", ' '.join(cmd))
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def start_static_tf(parent: str = 'map', child: str = 'ouster') -> subprocess.Popen:
    """
    Publish a static identity transform parent->child so RViz can place point clouds
    in the same fixed frame as LIO-SAM (map).
    """
    cmd = ['rosrun', 'tf', 'static_transform_publisher',
           '0', '0', '0', '0', '0', '0', parent, child, '10']
    print("Launching static TF:", ' '.join(cmd))
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def set_use_sim_time():
    """
    Ensure /use_sim_time is enabled so RViz and all nodes use the simulated clock.
    Must be set before RViz starts to ensure RViz displays time-dependent data.
    """
    try:
        subprocess.check_call(['rosparam', 'set', '/use_sim_time', 'true'],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("[INFO] Set /use_sim_time=true")
    except Exception as e:
        print(f"[WARN] Failed to set /use_sim_time: {e}", file=sys.stderr)

def start_player(seq_dir: str, rate: float, step: int, pct: float, keep_playing: bool) -> subprocess.Popen:
    cmd = ['rosrun', 'file_player', 'file_player_headless', '--dir', seq_dir, '--rate', str(rate)]
    if pct is not None and pct > 0.0:
        cmd += ['--start-percent', str(pct)]
        if not keep_playing and step <= 0:
            # default to one step in pct mode unless explicitly keeping playing or step provided
            cmd += ['--step', '1']
    if step and step > 0:
        cmd += ['--step', str(step)]
    print("Launching Player:", ' '.join(cmd))
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def read_stamp_bounds(seq_dir: str):
    path = os.path.join(seq_dir, 'sensor_data', 'data_stamp.csv')
    if not os.path.exists(path):
        raise RuntimeError(f"data_stamp.csv not found: {path}")
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
        raise RuntimeError("No valid stamps found in data_stamp.csv")
    return first, last

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True, help='Sequence directory root (full path)')
    parser.add_argument('--rate', type=float, default=1.0, help='Playback rate multiplier')
    parser.add_argument('--step', type=int, default=0, help='Step N data-stamp entries and stop')
    parser.add_argument('--pct', type=float, default=0.0, help='Seek to percentage (0..100) before starting')
    parser.add_argument('--keep-playing', action='store_true', help='Keep playing after seeking (ignore default single step)')
    parser.add_argument('--params-file', default=None, help='SC-LIO-SAM params YAML (launch arg params_file:=...)')
    parser.add_argument('--with-rviz', dest='with_rviz', action='store_true')
    parser.add_argument('--no-rviz', dest='with_rviz', action='store_false')
    parser.set_defaults(with_rviz=True)
    parser.add_argument('--rviz-config', default=None, help='Optional RViz config file')
    parser.add_argument('--visualize-seconds', type=int, default=20, help='Visualization window seconds')
    parser.add_argument('--timeout', type=int, default=60, help='Timeout for initial checks (seconds)')
    parser.add_argument('--roscore-first', action='store_true', help='Start a private roscore before launching SLAM')
    args = parser.parse_args()

    roscore_proc = None
    slam_proc = None
    rviz_proc = None
    player_proc = None
    static_tf_proc = None
    try:
        # 1) Optionally start roscore first, then launch SLAM
        if args.roscore_first:
            roscore_proc = start_roscore()
            if not wait_for_master(timeout=args.timeout):
                print("[ERROR] ROS master did not become available after starting roscore.", file=sys.stderr)
                return 2
            slam_proc = start_slam(args.params_file)
        else:
            slam_proc = start_slam(args.params_file)
            # 2) Wait for ROS master to come up
            if not wait_for_master(timeout=args.timeout):
                print("[WARN] ROS master not detected; starting private roscore...", file=sys.stderr)
                roscore_proc = start_roscore()
                if not wait_for_master(timeout=args.timeout):
                    print("[ERROR] ROS master did not become available within timeout.", file=sys.stderr)
                    return 2

        # Start static TF to connect map -> ouster so RViz can display points in map
        static_tf_proc = start_static_tf('map', 'ouster')
        # 3) Import ROS and set up subscribers
        try:
            import rospy
            from sensor_msgs.msg import NavSatFix, Imu, PointCloud2, Image
            import rosgraph
        except Exception as e:
            print("[ERROR] Failed to import rospy or message types. Ensure ROS environment is sourced.", file=sys.stderr)
            return 2

        rospy.init_node('visual_integration_test', anonymous=True)

        # Ensure simulated time is enabled before RViz starts
        set_use_sim_time()

        # 4) Optionally start RViz
        if args.with_rviz:
            rviz_proc = start_rviz(args.rviz_config)
            time.sleep(1.5)

        # 5) Start the player
        player_proc = start_player(args.dir, args.rate, args.step, args.pct, args.keep_playing)

        # 6) Subscriptions and checks
        counter = {'count': 0}
        first_msg_stamp = {'val': None}

        def generic_cb(msg):
            counter['count'] += 1
            try:
                first_msg_stamp['val'] = first_msg_stamp['val'] or msg.header.stamp.to_nsec()
            except Exception:
                pass

        subs = []
        subs.append(rospy.Subscriber('/gps/fix', NavSatFix, generic_cb))
        subs.append(rospy.Subscriber('/imu/data_raw', Imu, generic_cb))
        subs.append(rospy.Subscriber('/os1_points', PointCloud2, generic_cb))
        subs.append(rospy.Subscriber('/radar/polar', Image, generic_cb))

        # Proactively wait for first message to help RViz display and to catch misconfigurations early
        try:
            rospy.wait_for_message('/os1_points', PointCloud2, timeout=10.0)
            print("[INFO] First /os1_points message received.")
        except Exception:
            try:
                rospy.wait_for_message('/radar/polar', Image, timeout=10.0)
                print("[INFO] First /radar/polar message received.")
            except Exception:
                # Print topic list for debugging if neither appeared
                try:
                    topic_list = subprocess.check_output(['rostopic', 'list'], stderr=subprocess.STDOUT).decode()
                except Exception as e:
                    topic_list = f"rostopic list failed: {e}"
                print("[WARN] No /os1_points or /radar/polar received within 10s. Current topics:\n" + topic_list, file=sys.stderr)

        start_t = time.time()
        # If visualize-seconds is provided, keep node spinning for that window
        end_t = start_t + max(args.visualize_seconds, 1)
        while time.time() < end_t:
            if player_proc.poll() is not None:
                # Player ended (e.g., step mode completed)
                pass
            rospy.sleep(0.05)

        # 7) Evaluate basic criteria
        ok = True
        details = []

        if args.step and args.step > 0:
            if counter['count'] < args.step:
                ok = False
                details.append(f"step: expected >= {args.step} msgs, got {counter['count']}.")

        if args.pct and args.pct > 0.0:
            try:
                first, last = read_stamp_bounds(args.dir)
                expected = int(first + (last - first) * (args.pct / 100.0))
                if first_msg_stamp['val'] is None:
                    ok = False
                    details.append("pct: no message received to compare stamp.")
                else:
                    span = last - first
                    tol = max(1, int(span * 0.01))  # 1% tolerance
                    diff = abs(first_msg_stamp['val'] - expected)
                    print(f"[DEBUG] pct={args.pct}, expected={expected}, received={first_msg_stamp['val']}, diff={diff}, tol={tol}")
                    if diff > tol:
                        ok = False
                        details.append(f"pct: stamp diff {diff} > tol {tol}.")
            except Exception as e:
                ok = False
                details.append(f"pct: exception {e}")

        if ok:
            print("✅ Visual integration test passed.")
            return 0
        else:
            print("❌ Visual integration test failed: " + "; ".join(details), file=sys.stderr)
            return 2

    finally:
        # Cleanup in reverse order
        if player_proc:
            terminate(player_proc, "player")
        if rviz_proc:
            terminate(rviz_proc, "rviz")
        if slam_proc:
            terminate(slam_proc, "roslaunch SLAM", gentle_secs=5.0)
        if static_tf_proc:
            terminate(static_tf_proc, "static TF", gentle_secs=1.0)
        if roscore_proc:
            terminate(roscore_proc, "roscore", gentle_secs=2.0)

if __name__ == '__main__':
    sys.exit(main())
