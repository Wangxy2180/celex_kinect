#ifndef DETECTOR_TRACKER_H_
#define DETECTOR_TRACKER_H_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <vector>
#include <string>
#include "time.h"
#include <unistd.h>


#include "celex5_msgs/Event.h"
#include "celex5_msgs/EventVector.h"
// #include "dvs_msgs/Event.h"
// #include "dvs_msgs/EventArray.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
// #include "sensor_msgs/Imu.h"


/* local headers */
#include "event_traj_detector/depth_est.h"
#include "event_traj_detector/ekf_filter.hpp"
#include "event_traj_detector/eventor.h"
#include "event_traj_detector/obj_detector.h"
// #include "event_traj_detector/velocity_est.h"

#include "timer.h"

using namespace std;

namespace tracker {
class TrackSingleObj {
 private:
  /**
   * @brief
   * ON: detector is working
   * OFF: detector is waiting
   * TRIGGER_SENT: a trigger has been sent
   */
  enum DETECTION_STATE {
    ON = 0,
    OFF = -1,
    TRIGGER_SENT = 1,
  };

  /* flags */
  DETECTION_STATE state_;  // indicating drone status, TODO: rename this
  // int event_count_times_;

  /* parameters */
  std::string k_img_raw_topic_, k_event_topic_, k_imu_topic_,
    k_depth_topic_, k_odometry_topic_, k_imu_type_;
  double kNewObjThresTime;  // time threshold for judging a new object
  double KNewObjThresDis;  // distance threshold for judging a new object

  bool isVis;
  bool isSave;

  /* tracking utilities */
  Eventor::Ptr eventor_;  // data loader and motion compensation manager
  ObjDetector::Ptr obj_detector_;  // object detector
  DepthEst::Ptr depth_estimator_;  // depth estimator
  // VelocityEst::Ptr velocity_est_;  // velocity estimator
  EkfFilter ekf_obj_;

  /* trajectory utilities */
  std::deque<vector<cv::Point2d>> vis_trajs_;  // trajectory
  geometry_msgs::PointStamped point_last_;     // last point
  int start_traj_id_;

  /* ROS utilities */
  ros::NodeHandle &nh_;
  ros::Subscriber img_raw_sub_, events_sub_, imu_sub_, trigger_sub_, depth_sub_,
      odom_sub_;
  ros::Publisher start_avoidance_pub_, bullet_estimate_pub_, depth_pub_;
  image_transport::Publisher detection_res_pub, depth_res_pub_, cnt_image_pub_;

  Timer timer_;

  /* ROS functions */
  void ReadParameters(ros::NodeHandle &n);
//   void ImuCallback(const sensor_msgs::ImuConstPtr &imu);
//   void OdometryCallback(const nav_msgs::Odometry::ConstPtr &odom);
  // void TriggerCallback(const geometry_msgs::PoseStamped &p);
  void ImageCallback(const sensor_msgs::Image::ConstPtr &imsg);
//   void EventsCallback(const dvs_msgs::EventArray::ConstPtr &emsg);
  void EventsCallback(const celex5_msgs::EventVector::ConstPtr &emsg);
  void DepthCallback(const sensor_msgs::ImageConstPtr &msg);

  /* inline functions */
  inline bool IsNewObj(const geometry_msgs::PointStamped &point_now);
  inline void PubTrigger(void);
  inline void InitVisualization(void);

  /* visualization */
  std::vector<cv::Scalar> m_colors_;

 public:
  TrackSingleObj(ros::NodeHandle &nh) : nh_(nh) {
    start_traj_id_ = 0;
    // event_count_times_ = 0;
  }
  ~TrackSingleObj() {}
  void main();
  void Visualize(void);

  void saveRect();
};
}  // namespace tracker

#endif  // DETECTOR_TRACKER_H_