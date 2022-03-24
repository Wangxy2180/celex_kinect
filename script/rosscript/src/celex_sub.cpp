#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>


using namespace std;
using namespace cv;
Mat img;

void event2dObsCallback(const sensor_msgs::Image::ConstPtr msg)
{

std::cout<<"asd"<<std::endl;
    // imshow("2dimg", img);
    waitKey(30);
}

int main(int argc,char** argv)
{
  ros::init(argc, argv, "celex_sub");

    ros::NodeHandle nh("~");
    ros::Subscriber event_obs_sub;

    img = cv::Mat::zeros(cv::Size(1280, 800), CV_8UC1);

    event_obs_sub = nh.subscribe("/celex5_mipi/display/full_frame_img/raw_image", 10, event2dObsCallback);

    ros::spin();
    return 0;
}