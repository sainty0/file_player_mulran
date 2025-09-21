#!/usr/bin/env python3
"""
step_test.py

Runs the headless file player in step mode and verifies that it publishes
the requested number of data stamps (counting messages across main topics).

Usage:
  python3 tests/step_test.py --dir /full/path/to/sequence --step 5 [--timeout 30]

Notes:
- This script starts a local roscore. Make sure no other roscore is running.
- It expects the headless binary to be available as:
    rosrun file_player file_player_headless
  or available on PATH as 'file_player_headless' in package build.
- The test subscribes to /gps/fix, /imu/data_raw, /os1_points, /radar/polar
  and counts messages received on those topics.
"""
import argparse
import subprocess
import sys
import time
import os
import signal

# ROS imports are deferred until we ensured roscore is running
def start_roscore():
    p = subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # give roscore time to start
    time.sleep(1.5)
    return p

def kill_proc(p):
    try:
        p.terminate()
        p.wait(timeout=2.0)
    except Exception:
        try:
            p.kill()
        except Exception:
            pass

def find_executable():
    # prefer rosrun if package exists; user can adjust if necessary
    # We'll run the binary directly via rosrun; assume package installed in workspace
    return ['rosrun', 'file_player', 'file_player_headless']

def run_test(seq_dir, step_count, timeout):
    roscore_proc = start_roscore()
    try:
        import rospy
        from sensor_msgs.msg import NavSatFix, Imu, PointCloud2, Image
    except Exception as e:
        kill_proc(roscore_proc)
        print("Failed to import rospy or message types. Ensure ROS python packages are installed and sourced.", file=sys.stderr)
        raise

    rospy.init_node('step_test_node', anonymous=True)

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

    cmd = find_executable()
    cmd += ['--dir', seq_dir, '--rate', '1.0', '--step', str(step_count)]
    print("Launching: ", ' '.join(cmd))
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    start_t = time.time()
    # wait for process to exit or timeout
    while time.time() - start_t < timeout:
        if proc.poll() is not None:
            break
        rospy.sleep(0.05)

    # If process still running, try to terminate
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except Exception:
            proc.kill()

    # Allow messages to flush
    time.sleep(0.2)

    received = counter['count']
    print(f"Requested steps: {step_count}, messages received on monitored topics: {received}")
    success = (received >= step_count)
    # (>= because some stamps may produce multiple topics, but per-stamp typically one)
    if not success:
        print("Test failed: fewer messages received than requested steps.", file=sys.stderr)
    else:
        print("Test passed.")

    # cleanup roscore
    kill_proc(roscore_proc)
    return 0 if success else 2

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True, help='Sequence directory root (full path)')
    parser.add_argument('--step', type=int, default=1, help='Number of steps to request')
    parser.add_argument('--timeout', type=int, default=30, help='Timeout in seconds')
    args = parser.parse_args()

    rc = run_test(args.dir, args.step, args.timeout)
    sys.exit(rc)
