#!/usr/bin/env python3
"""
start_percent_test.py

Verifies that the headless player seeks to the requested --start-percent
position by comparing the first published data stamp after startup to the
expected timestamp computed from sensor_data/data_stamp.csv.

Usage:
  python3 tests/start_percent_test.py --dir /full/path/to/sequence --pct 25 [--timeout 30]

Notes:
- Starts a local roscore. Ensure no other roscore is running.
- Requires the headless binary available via:
    rosrun file_player file_player_headless
- The test runs the headless binary with --start-percent and --step 1 so it publishes
  one stamp after seeking; it captures the first message's header stamp and compares
  to the expected stamp computed from data_stamp.csv (linear interpolation).
"""
import argparse
import subprocess
import sys
import time
import os

def start_roscore():
    p = subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
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

def read_stamp_bounds(seq_dir):
    path = os.path.join(seq_dir, 'sensor_data', 'data_stamp.csv')
    if not os.path.exists(path):
        raise RuntimeError(f"data_stamp.csv not found at {path}")
    first = None
    last = None
    with open(path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) < 1:
                continue
            try:
                st = int(parts[0])
            except:
                continue
            if first is None:
                first = st
            last = st
    if first is None or last is None:
        raise RuntimeError("No stamps found in data_stamp.csv")
    return first, last

def find_executable():
    return ['rosrun', 'file_player', 'file_player_headless']

def run_test(seq_dir, pct, timeout):
    roscore = start_roscore()
    try:
        import rospy
        from sensor_msgs.msg import NavSatFix, Imu, PointCloud2, Image
    except Exception as e:
        kill_proc(roscore)
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

    # subscribe to topics where stamps are published; whichever publishes first will be used
    subs = []
    subs.append(rospy.Subscriber('/gps/fix', NavSatFix, cb))
    subs.append(rospy.Subscriber('/imu/data_raw', Imu, cb))
    subs.append(rospy.Subscriber('/os1_points', PointCloud2, cb))
    subs.append(rospy.Subscriber('/radar/polar', Image, cb))

    cmd = find_executable() + ['--dir', seq_dir, '--rate', '1.0', '--start-percent', str(pct), '--step', '1']
    print("Launching:", ' '.join(cmd))
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    start_t = time.time()
    while time.time() - start_t < timeout:
        if captured['stamp'] is not None:
            break
        if proc.poll() is not None:
            # process exited, but maybe no message was published
            break
        rospy.sleep(0.05)

    # cleanup
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except Exception:
            proc.kill()

    # allow small delay
    time.sleep(0.1)

    result_stamp = captured['stamp']
    if result_stamp is None:
        print("No message received within timeout.", file=sys.stderr)
        kill_proc(roscore)
        return 2

    # The implementation maps percent -> exact linear interpolation of stamp range.
    # Allow a small tolerance of +/- 1% of the total span (in nanoseconds) to account for discrete stamp lookup.
    span = last_stamp - first_stamp
    tol = max(1, int(span * 0.01))  # 1% tolerance
    diff = abs(result_stamp - expected)

    print(f"Requested percent: {pct}%")
    print(f"First stamp in CSV: {first_stamp}")
    print(f"Last stamp in CSV:  {last_stamp}")
    print(f"Expected stamp:     {expected}")
    print(f"Received stamp:     {result_stamp}")
    print(f"Difference:         {diff} (tolerance {tol})")

    success = (diff <= tol)
    if success:
        print("Test passed.")
    else:
        print("Test failed: received stamp is outside tolerance.", file=sys.stderr)

    kill_proc(roscore)
    return 0 if success else 2

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True, help='Sequence directory root (full path)')
    parser.add_argument('--pct', type=float, default=25.0, help='Start percent (0..100)')
    parser.add_argument('--timeout', type=int, default=30, help='Timeout seconds')
    args = parser.parse_args()

    rc = run_test(args.dir, args.pct, args.timeout)
    sys.exit(rc)
