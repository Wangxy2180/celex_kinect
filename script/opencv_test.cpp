#include <opencv2/rgbd.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <Eigen/Geometry>  // Eigen 几何模块
#include <Eigen/Core>

using namespace std;

/* GLOBAL DEFINES */
#define MAT_ROWS 480 // 240  //800
#define MAT_COLS 640 // 346  //1280

int cal_hist_test()
{
    cv::Mat img = cv::Mat::zeros(10, 10, CV_8UC1);
    img.at<int>(1, 1) = 127;
    cv::MatND hist_info;
    // 搞不懂这里为啥是128呢
    const int hist_size = 128;
    float hist_range[] = {0, 128};
    const float *hist_ranges[] = {hist_range};
    const int chs = 0; // image channels
    // 这里对应图5中间那张图，计算某个亮度的像素的数量
    /* compute histogram from depth image */
    cv::calcHist(&img, 1, &chs, cv::Mat(), hist_info, 1, &hist_size,
                 &hist_ranges[0]);
    cout << hist_info << endl;
}

int matptr_test()
{
    cv::Mat img_ = cv::Mat::zeros(cv::Size(5, 5), CV_8UC1);
    //   这两种方式都是可以的，用箭头或者点运算符
    int *c = (&img_)->ptr<int>(3, 3);
    *c = 88;
    //   cout<<img_<<endl;

    int *d = img_.ptr<int>(1, 1);
    *d = 66;
    cout << img_ << endl;
}

void mean_test()
{
    cv::Mat img_ = cv::Mat::ones(cv::Size(2, 2), CV_8UC1) * 10;
    cv::Mat mask = cv::Mat::ones(cv::Size(2, 2), CV_8UC1);

    uint8_t *i = img_.ptr<uint8_t>(0, 0);
    *i = 5;
    i = img_.ptr<uint8_t>(1, 0);
    *i = 10;
    i = img_.ptr<uint8_t>(0, 1);
    *i = 15;
    i = img_.ptr<uint8_t>(1, 1);
    *i = 20;

    uint8_t *m = mask.ptr<uint8_t>(0, 0);
    *m = 0;
    m = mask.ptr<uint8_t>(1, 0);
    *m = 10;

    //   mask.at<int>(0,0)=0;
    cout << "img :" << endl
         << img_ << endl;
    cout << "mask:" << endl
         << mask << endl;
    cv::Scalar s = cv::mean(img_, mask);
    cout << s << endl;
}

void moment_margin_0()
{
    cv::Matx33f matrix(0, 0, 0, 0, 1, 1, 0, 1, 1);
    // cv::Matx<int, 4, 4> matrix(0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0);
    cout << matrix << endl;

    cv::Moments mom = cv::moments(matrix);

    cout << mom.m01 << "," << mom.m00 << endl
         << mom.m10 << "," << mom.m00 << endl;

    cv::Point center_mass(static_cast<int>(mom.m10 / mom.m00),
                          static_cast<int>(mom.m01 / mom.m00));
    cout << "center:" << center_mass << endl;
    // m是几何矩，mu是中心矩，nu为归一化后的中心矩
    float epsilon_x = sqrt(mom.mu20 / mom.m00);
    float epsilon_y = sqrt(mom.mu02 / mom.m00);
    cout << "--------" << endl;
    cout << mom.mu20 << "," << mom.m00 << endl
         << mom.mu02 << "," << mom.m00 << endl;
    cout << epsilon_x << "," << epsilon_y << endl;
}

void moment_test()
{
    // cv::Matx33f matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
    cv::Matx33f matrix(2, 1, 1, 1, 5, 1, 1, 8, 2);
    cout << matrix << endl;

    cv::Moments mom = cv::moments(matrix);
    cout << mom.m01 << "," << mom.m00 << endl
         << mom.m10 << "," << mom.m00 << endl;

    cv::Point center_mass(static_cast<int>(mom.m10 / mom.m00),
                          static_cast<int>(mom.m01 / mom.m00));
    cout << "center:" << center_mass << endl;
    // m是几何矩，mu是中心矩，nu为归一化后的中心矩
    float epsilon_x = sqrt(mom.mu20 / mom.m00);
    float epsilon_y = sqrt(mom.mu02 / mom.m00);
    cout << "--------" << endl;
    cout << mom.mu20 << "," << mom.m00 << endl
         << mom.mu02 << "," << mom.m00 << endl;
    cout << epsilon_x << "," << epsilon_y << endl;
}

void rect_size_test()
{
    cv::Rect src2;
    cv::Rect2d src(10, 10, 2.5, 3.45);
    cout << src2.width << "," << src2.height << endl;
    cout << static_cast<double>(src.width * src.height) /
                static_cast<double>(src2.width * src2.height)
         << endl;
}

void eigen_test()
{
      Eigen::Matrix3d R_c2b;
  R_c2b << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Eigen::Quaterniond q_c2b(R_c2b);  ///< quaternion
  cout<<q_c2b.matrix()<<endl;
}

int main()
{
    // cal_hist_test();
    // matptr_test();
    // mean_test();
    // moment_test();
    // rect_size_test();
    // moment_margin_0();
    eigen_test();
}
