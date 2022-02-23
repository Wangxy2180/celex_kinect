#include "event_traj_detector/depth_est.h"
#include <opencv2/rgbd.hpp>

/**
 * @brief Set the event detecion result object
 *
 * @param roi_rect roi rectangle
 */
void DepthEst::SetEventDetectionRes(cv::Rect& roi_rect) {
  // 告知深度，我找到目标啦
  is_obj_ = true;
  roi_rect_ = roi_rect;
  valid_count_ = 0;
}

/**
 * @brief Set the event detecion result object
 *
 * @param is_obj   is valid object
 */
void DepthEst::SetEventDetectionRes(bool is_obj) {
  is_obj_ = is_obj;
  if (is_obj) {
    valid_count_ = 0;
  }
}

/**
 * @brief Depth camera callback
 *
 * @param msg
 */
void DepthEst::main(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

  /* init depth image */
  cv::Mat depth_gray_u8(msg->height, msg->width, CV_8UC1);
  depth_gray_ = cv::Mat::zeros(cv::Size(msg->height, msg->width), CV_8UC1);

  // 做深度事件两个相机配准吗？
  /* register depth data to an external camera */
  cv::rgbd::registerDepth(k_depth_camera_intrinsic_, k_event_camera_intrinsic_,
                          k_distort_coeff_, k_RT_event2depth_, cv_ptr->image,
                          k_event_camera_plane_, depth_gray_, false);

  /* morphology operations */
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5),
                                             cv::Point(-1, -1));
  morphologyEx(depth_gray_, depth_gray_, CV_MOP_CLOSE, kernel,
               cv::Point(-1, -1), 1);
  depth_gray_.convertTo(depth_gray_u8, CV_8UC1, 1.0 / 256);

  /* if number of frames > k_valid_frame_, then the rest
  * detection will be regrad as invalid
  */
//  k_valid_frame_固定是10，valid_count会在event检测到之后被置0，反正不大理解啥意思
  valid_count_++;
  if (valid_count_ > k_valid_frame_) {
    // 重置，认为还没有找到目标
    is_obj_ = false;
  }

  if (is_obj_) {  
    // 这个roi_rect_在事件相机检测到物体之后,调用setEventDetectionRes更新该值，
    // 也就是说这里一直保持着最新的roi_rect_
    cv::Rect r(roi_rect_);
    // 就是更新Rect r的值，把他扩大
    CropDepthImage(depth_gray_, &r);

    cv::Mat obj_img_u8 = depth_gray_u8(r);
    cv::Mat obj_img = depth_gray_(r);
    // topleft坐标
    float u = r.x; 
    float v = r.y;  
    // 计算直方图，分割属于物体的部分，这里得到的是符合条件的距离最近的灰度值的下标
    int loc = SegmentDepth(obj_img_u8);

    if (obj_depth_flag_) {
      // 这里是给做了个二值吗，输出的值是mask_range，就是图5中二值化的部分
      cv::Mat mask_range;
      cv::inRange(obj_img_u8, loc - 1, loc + 1, mask_range);

      /* compute mean and std */
      cv::Scalar mean, std;
      cv::meanStdDev(obj_img, mean, std, mask_range);
      // 标准差在一定范围内，才认为它有效
      if ((std[0] < 100) && (std[0] > 0)) {
        // 计算图像的中心矩，经过测试，边界的0不会影响结果
        auto m = cv::moments(mask_range, true);

        float roi_u = m.m10 / m.m00;
        float roi_v = m.m01 / m.m00;
        // uv就是质心在整张深度图上的坐标
        u += roi_u;
        v += roi_v;
        float u0 = k_event_camera_intrinsic_.at<float>(0, 2);
        float v0 = k_event_camera_intrinsic_.at<float>(1, 2);
        float fx = k_event_camera_intrinsic_.at<float>(0, 0);
        float fy = k_event_camera_intrinsic_.at<float>(1, 1);

        /* if detections on depth image are too closed to image edge, 
         * we will not fuse the coordinates from depth detection,
         * instead, we only fuse approximate depth data
         *
         */
        float du =
            (mask_range.cols - roi_u) > roi_u ? roi_u : mask_range.cols - roi_u;
        float dv =
            (mask_range.rows - roi_v) > roi_v ? roi_v : mask_range.rows - roi_v;
        
        // 应该是到事件相机的坐标系吧，只有这里发送三维坐标来着
        // geometry_msgs::PointStamped depth_p;
        depth_p_.header.stamp = msg->header.stamp;
        depth_p_.header.frame_id = "/world";
        depth_p_.point.x = mean[0] * (u - u0) / fx;
        depth_p_.point.y = mean[0] * (v - v0) / fy;
        depth_p_.point.z = mean[0];  // millimeters

        /* WARNING: deprecated */
        /* chech the center of mass of the detection point,
         * if the detection point is too close to the edge,
         * we will ignore the center of mass,
         * and only use the depth to optimize.
        */
        // if ((du < 20) || (dv < 20)) {
        //   depth_p.header.frame_id = "/only_depth";
        // }
      }
    }

    /* visualization */
    cv::Mat vis_depth_(msg->height, msg->width, CV_8UC3);
    cv::applyColorMap(depth_gray_u8, vis_depth_, cv::COLORMAP_JET);
    cv::rectangle(vis_depth_, r, cv::Scalar(0, 255, 0), 2, cv::LINE_8, 0);
  }
}

/**
 * @brief
 *
 * @param src source depth image
 * @param dst_rect pointer to destination rectangle
 */
void DepthEst::CropDepthImage(const cv::Mat src, cv::Rect* dst_rect) {
  // 是给他边长扩大两倍？
  dst_rect->x =
      (dst_rect->x - dst_rect->width / 2) < 0 ? 0 : (dst_rect->x - dst_rect->width / 2);
  dst_rect->y =
      (dst_rect->y - dst_rect->height / 2) < 0 ? 0 : (dst_rect->y - dst_rect->height / 2);
  dst_rect->height *= 2;
  dst_rect->width *= 2;
  if ((dst_rect->height + dst_rect->y) > src.rows)
    dst_rect->height = src.rows - dst_rect->y;
  if ((dst_rect->width + dst_rect->x) > src.cols) dst_rect->width = src.cols - dst_rect->x;
}

/**
 * @brief  segment depth belongs to objects on the histogram
 *
 * @param img depth image
 * @return int record
 */
int DepthEst::SegmentDepth(const cv::Mat& img) {
  cv::MatND hist_info;
  // 搞不懂这里为啥是128呢
  const int hist_size = 128;
  float hist_range[] = {1, 128};
  const float* hist_ranges[] = {hist_range};
  const int chs = 0;  // image channels
  // 这里对应图5中间那张图，计算某个亮度的像素的数量
  /* compute histogram from depth image */
  cv::calcHist(&img, 1, &chs, cv::Mat(), hist_info, 1, &hist_size,
               &hist_ranges[0]);
  // 上边的输出是hist_info
  int record;
  obj_depth_flag_ = false;
  for (record = 0; record < hist_size; record++) {
    // 这里比值计算是什么个道理？是说图中至少有2%的灰度值要和他一样？
    // 似乎是在寻找距离最近的直方图
    if (hist_info.at<int>(record) > 0.02 * img.rows * img.cols) {
      record++; 
      obj_depth_flag_ = true;
      break;
    }
  }
  // 也就是说，record是一个idx，一个代表灰度值得idx
  return record;
}
