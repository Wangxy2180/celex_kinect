#include "event_traj_detector/tracker.h"
#include "event_traj_detector/obj_detector.h"
#include "ros/ros.h"


using namespace tracker;

/**
 * @brief
 *
 * @param argc
 * @param argv 
 *
 * @return int
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "detector_node");
  ros::NodeHandle nh("~");

  TrackSingleObj tracker(nh);
  tracker.main();
  ROS_INFO("Hello, detector is running...");
  ros::spin();

  return 0;
}
