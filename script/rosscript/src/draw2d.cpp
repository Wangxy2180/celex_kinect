#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace cv;
Mat img;

void event2dObsCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{

    int ix = msg->point.x;
    int iy = msg->point.y;
    img.at<uchar>(iy, ix) = 255;
    cv::circle(img, cv::Point(ix, iy), 10, cv::Scalar(255, 255, 255));

    imshow("2dimg", img);
    waitKey(30);
}

int main(int argc,char** argv)
{
  ros::init(argc, argv, "draw2d");

    ros::NodeHandle nh("~");
    ros::Subscriber event_obs_sub;

    img = cv::Mat::zeros(cv::Size(1280, 800), CV_8UC1);

    event_obs_sub = nh.subscribe("/cam_bullet_point", 10, event2dObsCallback);

    ros::spin();
    return 0;
}