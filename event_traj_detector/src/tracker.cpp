#include "event_traj_detector/tracker.h"
#include "ros/ros.h"

namespace tracker
{

    void TrackSingleObj::main()
    {
        /* params */
        // 初始化可视化的内容
        InitVisualization();
        // 读取ros topic参数
        ReadParameters(nh_);

        kNewObjThresTime = 0.3; // 30 milliseconds
        KNewObjThresDis = 200;  // 400 pixels

        image_transport::ImageTransport it_img_rst(nh_);
        image_transport::ImageTransport it_depth_rst(nh_);
        image_transport::ImageTransport it_img_mid(nh_); // time image

          eventor_.reset(new Eventor);
        //   motion_compensation_->SetIMUType(k_imu_type_);
        depth_estimator_.reset(new DepthEst);
        //   velocity_est_.reset(new VelocityEst);
        obj_detector_.reset(new ObjDetector);

        /* callback */
        // 改变状态，输出WARNING
        //   trigger_sub_ =
        //       nh_.subscribe("/traj_start_trigger", 1, &TrackSingleObj::TriggerCallback,
        //                     this, ros::TransportHints().tcpNoDelay());
        // 啥都没干
        //   img_raw_sub_ =
        //       nh_.subscribe(k_img_raw_topic_, 1, &TrackSingleObj::ImageCallback, this);
        // 检测ROI区域
        events_sub_ =
            nh_.subscribe(k_event_topic_, 2, &TrackSingleObj::EventsCallback, this);

        //   imu_sub_ = nh_.subscribe(k_imu_topic_, 10, &TrackSingleObj::ImuCallback, this,
        //                            ros::TransportHints().tcpNoDelay());
        // 去ROI区域中读取深度数据
        depth_sub_ =
            nh_.subscribe(k_depth_topic_, 1, &TrackSingleObj::DepthCallback, this);
        // 动态补偿？
        //   odom_sub_ =
        //       nh_.subscribe(k_odometry_topic_, 10, &TrackSingleObj::OdometryCallback,
        //                     this, ros::TransportHints().tcpNoDelay());

        /* advertiser */
        image_pub_ = it_img_rst.advertise("/dvs/detection_res", 1);
        depth_res_pub_ = it_depth_rst.advertise("/depth/res", 1);
        time_image_pub_ = it_img_mid.advertise("/dvs/time_image", 1);
        depth_pub_ =
            nh_.advertise<geometry_msgs::PointStamped>("/cam_depth_bullet_point", 1);
        start_avoidance_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>("/avoid_start_trigger", 1);
        bullet_estimate_pub_ =
            nh_.advertise<geometry_msgs::PointStamped>("/cam_bullet_point", 10);

        /* detection on: only for debug */
        state_ = ON;
    }

    void TrackSingleObj::ReadParameters(ros::NodeHandle &n)
    {
        n.getParam("/event_traj_detector_node/depth_topic", k_depth_topic_);
        //   n.getParam("/event_traj_detector_node/imu_topic", k_imu_topic_);
        n.getParam("/event_traj_detector_node/raw_image_topic", k_img_raw_topic_);
        n.getParam("/event_traj_detector_node/event_topic", k_event_topic_);
        //   n.getParam("/event_traj_detector_node/odometry_topic", k_odometry_topic_);
        //   k_imu_type_ = "px4";
        // (k_imu_type_.find("dji") < k_imu_type_.length()) ? "dji" : "px4";
    }

    /**
 * @brief
 *
 * @param emsg
 */
    void TrackSingleObj::EventsCallback(
        const celex5_msgs::EventVector::ConstPtr &emsg)
    {
        static int obj_count = 0;
        event_count_times_++;

        /* motion compensate input events */
        // 将事件信息拷贝到这补偿类中
        eventor_->LoadEvents(emsg);

        eventor_->LoadDepth(depth_estimator_->GetDepth());
        // 就是在这里边进行的更新time_img cnt_img
        eventor_->generate();

        /* detect objects on compensated images */
        cv::Mat time_image, event_count, small, mask;
        time_image = eventor_->GetTimeImage();
        event_count = eventor_->GetEventCount();

        obj_detector_->LoadImages(event_count, time_image);
        obj_detector_->Detect();

        // 这里顺序不大对啊，应该先判断再获取max_rect
        cv::Rect max_rect = obj_detector_->GetDetectRsts();

        /**
   * @brief No objected detected in this frame
   */
        if (obj_detector_->ObjMissed())
        {
            Visualize();
            return;
        }

        // 如果到这里了，那就是找到目标物体了，先更新一下深度图中ROI的值
        /* project ROI into depth image */
        depth_estimator_->SetEventDetectionRes(max_rect);

        // 计算区域内的平均时间
        small = time_image(max_rect);
        small.convertTo(small, CV_8U);
        auto ts = cv::mean(small, small); // detection's timestamp

        geometry_msgs::PointStamped point_in_plane;

        //   这里改一下，改成记录最老的事件，然后这里直接调用
        ros::Time tt;
        tt.fromNSec(emsg->events[0].in_pixel_timestamp * 1000);
        point_in_plane.header.stamp = tt + ros::Duration(ts[0]);
        //   point_in_plane.header.stamp = emsg->events[0].in_pixel_timestamp + ros::Duration(ts[0]);
        point_in_plane.header.frame_id = "/cam";
        //   计算中心点
        point_in_plane.point.x = max_rect.x + max_rect.width * 0.5f;
        point_in_plane.point.y = max_rect.y + max_rect.height * 0.5f;
        point_in_plane.point.z = 0;

        // cout << "(" << point_in_plane.point.x << "," << point_in_plane.point.y << ")"
        //      << std::endl;

        /* chech if new objects are detected */
        if (IsNewObj(point_in_plane))
        {
            if (state_ == TRIGGER_SENT)
            {
                state_ = ON;
                obj_count = 0;
            }

            ekf_obj_.reset_data(point_in_plane.point.x, point_in_plane.point.y,
                                point_in_plane.header.stamp);

            if (vis_trajs_.size() > 5)
            {
                /* buffer full, pop front */
                start_traj_id_++;
                vis_trajs_.pop_front();
            }
            else
            {
                vis_trajs_.emplace_back(*new std::vector<cv::Point2d>);
                vis_trajs_.back().emplace_back(cv::Point2d(ekf_obj_.x_, ekf_obj_.y_));
            }

            std::cout << "traj size: " << vis_trajs_.size() << std::endl;
        }
        else
        {
            //still the old points accumulated observations send a trigger to planner
            switch (state_)
            {
            case ON:
            {
                obj_count++;
                if (obj_count > 4)
                {
                    PubTrigger();
                }
                break;
            }
            case TRIGGER_SENT:
                break;
            default:
                break;
            }

            ekf_obj_.add_new_obs(point_in_plane.point.x, point_in_plane.point.y,
                                 point_in_plane.header.stamp);

            vis_trajs_.back().emplace_back(cv::Point2d(ekf_obj_.x_, ekf_obj_.y_));
        }

        if (state_ == ON || state_ == TRIGGER_SENT)
        {
            // 二维轨迹发布
            point_in_plane.point.x = ekf_obj_.x_;
            point_in_plane.point.y = ekf_obj_.y_;
            bullet_estimate_pub_.publish(point_in_plane);
            depth_estimator_->istart_ = true;
            ROS_WARN("[DETECTION] 2d TRAJECTORY PUBLISHED!");
        }

        point_last_ = point_in_plane;

        /* visualize time image */
        Visualize();
        cv::waitKey(30);
    }

    /**
 * @brief send a trigger to avoid obstacles
 *
 * @param p
 */
    void TrackSingleObj::TriggerCallback(const geometry_msgs::PoseStamped &p)
    {
        switch (state_)
        {
        case OFF:
            ROS_WARN("[DETECTION] START! >>>>>");
            state_ = ON;
            vis_trajs_.clear();
            break;
        default:
            ROS_INFO("[DETECTION] STOP! <<<<<");
            state_ = OFF;
            break;
        }
    }

    // void TrackSingleObj::ImuCallback(const sensor_msgs::ImuConstPtr &imu) {
    //   motion_compensation_->LoadIMUs(imu);
    // }

    // void TrackSingleObj::OdometryCallback(
    //     const nav_msgs::Odometry::ConstPtr &odom) {
    //   motion_compensation_->LoadOdometry(odom);
    // }

    // void TrackSingleObj::ImageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    //   /* do nothing:
    //    * if image from gazebo is not subsribed,
    //    * event will not be triggered */
    // }

    void TrackSingleObj::DepthCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        depth_estimator_->main(msg);
        geometry_msgs::PointStamped depth_point;
        // 获取在事件相机下的坐标
        depth_point = depth_estimator_->GetDepthPoint();
        if (depth_estimator_->istart_)
        {
            // 三维轨迹发布
            depth_pub_.publish(depth_point);
        }

        cv::Mat depth_visualize = depth_estimator_->GetDepthVisualization();

        sensor_msgs::ImagePtr depth_vis_msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_visualize)
                .toImageMsg();
        depth_res_pub_.publish(depth_vis_msg);
    }

    /**
 * @brief check if a new object is detected
 * if two detection have a large time interval or a far distance,
 * then they will be regarded as two individual objects
 *
 * @param point_now
 */
    inline bool TrackSingleObj::IsNewObj(
        const geometry_msgs::PointStamped &point_now)
    {
        // 时间和距离都满足阈值，才认为他不是新物体
        if ((point_now.header.stamp - point_last_.header.stamp) >
            ros::Duration(kNewObjThresTime))
        { // over than 10 milliseconds
            ROS_DEBUG("Time duration between two observations is too long");
        }
        else
        {
            double dx = point_now.point.x - point_last_.point.x;
            double dy = point_now.point.y - point_last_.point.y;
            double dis = fabs(dx) + fabs(dy);

            if (dis > KNewObjThresDis)
            { // if two objects are too far
                // TODO: Use KF to estimate new position and judge if it's a new object
            }
            else
            {
                return false;
            }
        }
        // start_traj_id_++;
        return true;
    }

    inline void TrackSingleObj::PubTrigger()
    {
        ROS_WARN("AVOIDANCE START! Launch !!!");
        geometry_msgs::PointStamped start_trigger;
        start_trigger.header.stamp = ros::Time::now();
        start_avoidance_pub_.publish(start_trigger);
        state_ = TRIGGER_SENT;
    }

    /**
 * @brief add pre-defined colors into the buffer
 */
    inline void TrackSingleObj::InitVisualization()
    {
        m_colors_.push_back(cv::Scalar(255, 0, 0));
        m_colors_.push_back(cv::Scalar(0, 255, 0));
        m_colors_.push_back(cv::Scalar(0, 0, 255));
        m_colors_.push_back(cv::Scalar(255, 255, 0));
        m_colors_.push_back(cv::Scalar(0, 255, 255));
        m_colors_.push_back(cv::Scalar(255, 0, 255));
        m_colors_.push_back(cv::Scalar(255, 127, 255));
        m_colors_.push_back(cv::Scalar(127, 0, 255));
        m_colors_.push_back(cv::Scalar(127, 0, 127));
    }

    /**
 * @brief visualize event-image and trajectories
 */
    void TrackSingleObj::Visualize()
    {
        cv::Mat img = obj_detector_->GetVisualization();
        std_msgs::Header h;
        h.stamp = ros::Time::now();
        for (auto iter = vis_trajs_.begin(); iter < vis_trajs_.end(); iter++)
        {
            cv::Scalar cl = m_colors_[(iter - vis_trajs_.begin() + start_traj_id_) %
                                      m_colors_.size()];
            auto &point_list = *iter;
            for (size_t iter_point = 0; iter_point < (point_list.size() - 1);
                 iter_point++)
            {
                const auto &pt1 = point_list.at(iter_point);
                const auto &pt2 = point_list.at(iter_point + 1);
                cv::line(img, pt1, pt2, cl, 2, LINE_4);
            }
        }
        double ekf_obj_x, ekf_obj_y;
        ekf_obj_.GetPosition(&ekf_obj_x, &ekf_obj_y);
        cv::circle(img, cv::Point2d(ekf_obj_x, ekf_obj_y), 2,
                   cv::Scalar(255, 255, 255), 2);
        cv_bridge::CvImage cv_image(h, "bgr8", img);
        image_pub_.publish(cv_image.toImageMsg());

        /* visualize time image after motion compensation */
        cv::Mat time_img = eventor_->GetVisualization();
        ///////////////////////////
        //     Writer.write(time_img);
        ///////////////////////////

        cv_bridge::CvImage cv_time_image(h, "bgr8", time_img);
        time_image_pub_.publish(cv_time_image.toImageMsg());

        // cout << "=========================" << endl;
    }

} // namespace tracker
