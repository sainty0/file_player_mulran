// file_player_mulran - Headless replay node
// ------------------------------------------------------------
// Plays a recorded MulRan sequence without any Qt GUI.
// Publishes:
//   * /clock                (rosgraph_msgs/Clock)
//   * /gps/fix              (sensor_msgs/NavSatFix)
//   * /imu/data_raw         (sensor_msgs/Imu)
//   * /os1_points           (sensor_msgs/PointCloud2)
//   * /radar/polar          (sensor_msgs/Image)
// Parameters (namespaced under this node):
//   seq_dir   (string, required)  – Sequence root
//   rate      (double, default:1) – Play speed multiplier
//   loop      (bool,   default:false)
// ------------------------------------------------------------

#include <ros/ros.h>
#include <boost/program_options.hpp>
#include "ROSThread.h"

namespace po = boost::program_options;

class FilePlayerNode
{
public:
  FilePlayerNode(const po::variables_map& vm, ros::NodeHandle nh_private)
  : player_{nullptr, &mutex_}
  {
    player_.ros_initialize(nh_private);

    player_.data_folder_path_ = vm["dir"].as<std::string>();
    player_.play_rate_        = vm["rate"].as<double>();
    player_.loop_flag_        = vm["loop"].as<bool>();
    player_.pause_flag_       = false;
    player_.play_flag_        = true;

    player_.Ready();
    player_.start();
  }

private:
  QMutex      mutex_;
  ROSThread   player_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "file_player_headless");

  //-----------------------------------------------------------------
  // 1. CLI / parameter handling
  //-----------------------------------------------------------------
  po::options_description opts("Allowed options");
  opts.add_options()
        ("help,h",   "produce help message")
        ("dir,d",    po::value<std::string>()->required(), "Sequence directory root")
        ("rate,r",   po::value<double>()->default_value(1.0), "Playback speed multiplier")
        ("loop",     po::bool_switch()->default_value(false), "Loop sequence");

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, opts), vm);
    if (vm.count("help")) {
      std::cout << opts << std::endl;
      return 0;
    }
    po::notify(vm);
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("Argument error: " << e.what() << "\n" << opts);
    return 1;
  }

  //-----------------------------------------------------------------
  // 2. Node startup
  //-----------------------------------------------------------------
  ros::NodeHandle nh_private("~");
  const unsigned n_threads = std::max(2u, std::thread::hardware_concurrency() - 1);
  ros::AsyncSpinner spinner(n_threads);

  FilePlayerNode node(vm, nh_private);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
