#include <opencv2/rgbd.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <Eigen/Geometry> // Eigen 几何模块
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
    cout << "x:" << epsilon_x << ","
         << "y:" << epsilon_y << endl;
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
    Eigen::Quaterniond q_c2b(R_c2b); ///< quaternion
    cout << q_c2b.matrix() << endl;
}

void mul_test()
{
    cv::Mat img_ = cv::Mat::ones(cv::Size(2, 2), CV_8UC1);

    uint8_t *i = img_.ptr<uint8_t>(0, 0);
    *i = 100;
    i = img_.ptr<uint8_t>(1, 0);
    *i = 255;
    i = img_.ptr<uint8_t>(0, 1);
    *i = 1;
    i = img_.ptr<uint8_t>(1, 1);
    *i = 10;

    cout << "1" << endl
         << img_ << endl;
    img_ = img_.mul(img_);
    cout << "2" << endl
         << img_ << endl;
}

void meanstddev_test()
{
    cv::Matx33f matrix(2, 1, 1, 1, 5, 1, 1, 8, 2);
    cv::Mat meann, stdd, c;
    cv::meanStdDev(matrix, meann, stdd, c);
    cout << "mean:" << meann << ", std:" << stdd << endl;
}

void calhist_test()
{
    cv::Mat matrix = cv::Mat::zeros(cv::Size(3, 3), CV_8UC1);
    // matrix = (1, 1, 1, 1, 5, 1, 1, 8, 7);
    matrix.col(0) = 1;
    matrix.col(1) = 2;
    matrix.col(2) = 127;
    cout << matrix << endl;
    cv::Mat hist_info;
    const int hist_size = 128;
    float hist_range[] = {1, 128};
    const float *hist_ranges[] = {hist_range};

    const int chs = 0; // image channels

    cv::calcHist(&matrix, 1, &chs, cv::Mat(), hist_info, 1, &hist_size, &hist_ranges[0]);
    cout << hist_info << endl;
}

void convert_test()
{
    cv::Mat u2f = cv::Mat::zeros(cv::Size(3, 1), CV_8UC1);
    u2f.col(0) = 1;
    u2f.col(1) = 2;
    u2f.col(2) = 127;
    cout << "u2f  8UC1" << u2f << endl;
    u2f.convertTo(u2f, CV_32FC1);
    cout << "u2f 32FC1" << u2f << endl;

    cv::Mat f2u = cv::Mat::zeros(cv::Size(3, 1), CV_16UC1);
    f2u.col(0) = 0;
    f2u.col(1) = 100;
    f2u.col(2) = 10000;
    cout << "f2u 32FC1" << f2u << endl;
    f2u.convertTo(f2u, CV_8UC1, 1.0 / 256);
    cout << "f2u  8UC1" << f2u << endl;
}

string Type2String(int type)
{
    string strType;
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);
    switch (depth)
    {
    case CV_8U:
        strType = "CV_8U";
        break;
    case CV_8S:
        strType = "CV_8S";
        break;
    case CV_16U:
        strType = "CV_16U";
        break;
    case CV_16S:
        strType = "CV_16S";
        break;
    case CV_32S:
        strType = "CV_32S";
        break;
    case CV_32F:
        strType = "CV_32F";
        break;
    case CV_64F:
        strType = "CV_64F";
        break;
    default:
        strType = "UNKNOWN_TYPE";
        break;
    }
    strType += "C";
    strType += (chans + '0');

    return strType;
}

void chessboardTest()
{
    // cv::Mat colorDisp = cv::imread("../image_chess.png", cv::IMREAD_GRAYSCALE);
    cv::Mat colorDisp = cv::imread("../image_chess_paper.png", cv::IMREAD_GRAYSCALE);
    cout << colorDisp.channels() << endl;
    // cv::imshow("chess color",colorDisp);

    vector<cv::Point2f> pointsColor;
    bool foundColor = true;
    cv::Size boardDims = cv::Size(5, 7);
    // foundColor = cv::findChessboardCorners(colorDisp, boardDims, pointsColor, cv::CALIB_CB_FAST_CHECK);
    foundColor = cv::findChessboardCorners(colorDisp, boardDims, pointsColor);
    cout << foundColor << endl;
          cv::cvtColor(colorDisp, colorDisp, CV_GRAY2BGR);
    cv::drawChessboardCorners(colorDisp, boardDims, pointsColor, foundColor);
    cv::imshow("chess color", colorDisp);
}

int main()
{
    // cal_hist_test();
    // matptr_test();
    // mean_test();
    // moment_test();
    // rect_size_test();
    // moment_margin_0();
    // eigen_test();
    // mul_test();
    // meanstddev_test();
    // calhist_test();
    // convert_test();
    // cout << Type2String(2) << endl;
    chessboardTest();

    cv::waitKey(3000);
}
