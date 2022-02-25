#include "event_traj_detector/eventor.h"

#include "ros/ros.h"

/**
 * @brief warp events and utilize motion compensation
 *
 */
void Eventor::generate()
{
    Clear();
    // notCompensate(&time_img_, &event_counts_);
    rotationalCompensate(&time_img_, &event_counts_);
    // translationalCompensate(&time_img_, &event_counts_);
}

void Eventor::Clear()
{
    // omega_avg_.setZero();
    events_buffer_.clear();
    //   imu_size_ = IMU_buffer_.size();

    time_img_ = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_32FC1);
    event_counts_ = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);

    fill(block_rows.begin(), block_rows.end(), 0);
    fill(block_cols.begin(), block_cols.end(), 0);

    block_rows_eigen.setZero();
    block_cols_eigen.setZero();

    // block_rows.clear();
    // block_cols.clear();
    // block_rows.resize(MAT_ROWS / BLOCK_SIZE, 0);
    // block_cols.resize(MAT_COLS / BLOCK_SIZE, 0);
}

void Eventor::LoadEvents(const celex5_msgs::EventVector::ConstPtr &emsg)
{
    events_buffer_.assign(emsg->events.begin(), emsg->events.end());
    event_size_ = events_buffer_.size();
}

void Eventor::LoadDepth(const cv::Mat &depth) { depth_img_ = depth; }

cv::Mat Eventor::GetVisualization()
{
    cv::Mat m, m_color;
    cv::normalize(time_img_, m, 0, 255, cv::NORM_MINMAX);
    m.convertTo(m, CV_8UC1);
    cv::applyColorMap(m, m_color, cv::COLORMAP_JET);
    // cv::namedWindow("time image");
    // cv::imshow( "window", m_color);
    // cv::waitKey(0);
    return m_color;
}

/**
 * @brief Accumulate events without any compensation
 *
 * @param timeImg
 * @param eventCount
 */
void Eventor::notCompensate(cv::Mat *timeImg, cv::Mat *eventCount)
{
    auto t0 = events_buffer_[0].in_pixel_timestamp;
    float prevDeltaT = 0.0f;

    for (int i = 0; i < event_size_; i++)
    {
        celex5_msgs::Event e = events_buffer_[i];
        // float deltaT = (e.in_pixel_timestamp - t0).toSec();
        float deltaT = (e.in_pixel_timestamp - t0) / 1000000;

        int ix = e.x;
        int iy = e.y;

        if (!IsWithinTheBoundary(ix, iy))
        {
            continue;
        }
        else
        {
            int *c = eventCount->ptr<int>(iy, ix);
            float *q = timeImg->ptr<float>(iy, ix);
            *c += 1;
            float v = *q;
            *q += (deltaT - v) / (*c);
        }
    }
}

void Eventor::edgeBlock(const int x, const int y)
{
    block_rows[y / BLOCK_SIZE] += 1;
    block_cols[x / BLOCK_SIZE] += 1;

    block_rows_eigen[y / BLOCK_SIZE] += 1;
    block_cols_eigen[x / BLOCK_SIZE] += 1;

    // cout<<"x,y:"<<x<<","<<y<<endl;

    // if(y/BLOCK_SIZE == 7)cout<<"888<"<<block_rows[7]<<endl;
}

void Eventor::getEdgeBlock(Eigen::Array<int, MAT_ROWS/BLOCK_SIZE, 1>& rowVar,
    Eigen::Array<int, MAT_COLS/BLOCK_SIZE, 1>& colVar)
{
    // rowBlock.assign(block_rows.begin(), block_rows.end());
    // colBlock.assign(block_cols.begin(), block_cols.end());
    rowVar=block_rows_eigen;
    colVar=block_cols_eigen;


    // float col_variance = (block_cols_eigen.mean() - block_cols_eigen).square().sum() / (MAT_COLS / BLOCK_SIZE);
    // float row_variance = (block_rows_eigen.mean() - block_rows_eigen).square().sum() / (MAT_ROWS / BLOCK_SIZE);

    // rowVar=row_variance;
    // colVar=col_variance;

    // if(row_variance>1500 && col_variance>1500)
    // {
        
    // }

    // double sum1 = std::accumulate(rowBlock.begin(), rowBlock.end(), 0.0);
    // double mean1 = sum1 / rowBlock.size(); //均
    // double ret_sum1 = 0;
    // for (auto kk : rowBlock)
    // {
    //     ret_sum1 += (mean1 - kk) * (mean1 - kk);
    // }
    // double vrow = ret_sum1 / rowBlock.size();

    // double sum2 = std::accumulate(colBlock.begin(), colBlock.end(), 0.0);
    // double mean2 = sum2 / colBlock.size(); //均
    // double ret_sum2 = 0;
    // for (auto kk : colBlock)
    // {
    //     ret_sum2 += (mean2 - kk) * (mean2 - kk);
    // }
    // double vcol = ret_sum2 / colBlock.size();

    // cout <<">>>"<< col_variance << "," << row_variance << endl
    //      <<"<<<"<< vcol << "," << vrow << endl;
    // cout << "--------------" << endl;
}

void Eventor::rotationalCompensate(cv::Mat *timeImg, cv::Mat *eventCount)
{
    //   寻找min时间
    celex5_msgs::Event oldestEvent = *min_element(events_buffer_.begin(), events_buffer_.end(), [](celex5_msgs::Event &a, celex5_msgs::Event &b)
                                                  { return a.in_pixel_timestamp < b.in_pixel_timestamp; });
    auto t0 = oldestEvent.in_pixel_timestamp;
    //   auto t0 = events_buffer_[0].in_pixel_timestamp;

    for (int i = 0; i < event_size_; i++)
    {
        celex5_msgs::Event e = events_buffer_[i];
        // float deltaT = (e.in_pixel_timestamp - t0).toSec();
        // 单位是秒
        float deltaT = (e.in_pixel_timestamp - t0) / 1000000;

        // 这里要注意，celex数据xy是反的，x最大800，y最大1280
        int ix = static_cast<int>(e.y);
        int iy = static_cast<int>(MAT_ROWS - e.x);
        // cout<<"x,y"<<ix<<","<<iy<<endl;

        if (IsWithinTheBoundary(ix, iy))
        {
            // 计算落在那个block中
            edgeBlock(ix, iy);
            // 更新两张图
            int *c = eventCount->ptr<int>(iy, ix);
            *c += 1;

            // 平均time_img
            float *q = timeImg->ptr<float>(iy, ix);
            float v = *q;
            *q += (deltaT - v) / (*c);
        }
    }
    // cout << "--block_rows:" << block_rows.size() << "," << block_rows[8] << endl;
    // cout << "--block_cols:" << block_cols.size() << "," << block_cols[8] << endl;

    ///////////// to debug ////////////////////////
    // double maxValue;
    // maxValue = *max_element(timeImg->begin<float>(), timeImg->end<float>());
    // std::cout << "time image max " << maxValue << std::endl;
    // maxValue = *max_element(eventCount->begin<uchar>(),
    // eventCount->end<uchar>()); std::cout << "event count max " << maxValue <<
    // std::endl; GetVisualization();
    //////////////////////////////////////////////
}

bool Eventor::updateEventWindow(int dataSize)
{
    // 这个值可以考虑修改一下
    if (dataSize < event_init_threshold_)
        return false;
    if (dataSize == env_window_(envWindowSize - 1))
        return false;
    env_window_.topRows<envWindowSize - 1>() = env_window_.bottomRows<envWindowSize - 1>();
    env_window_(env_window_.size() - 1) = dataSize;
    // 这里改成了11个大小的，其中前10个用来计算阈值，第11个是最新的
    // dynamic_threshold_=((env_window_.sum()-env_window_(envWindowSize-1))/(env_window_.size()-1))*dynamic_threshold_scale_;
    dynamic_threshold_ = (env_window_.topRows<envWindowSize - 1>().sum() / (env_window_.size() - 1)) * dynamic_threshold_scale_;
    return true;
}

bool Eventor::initComplete()
{
    // 根据第一窗口数值判断是否完成初始化
    if (env_window_(0) == 0)
        return false;
    return true;
}

bool Eventor::objAppear()
{
    // 事件数量超出阈值，目标出现，可以进行下一步检测动作
    if (env_window_(envWindowSize - 1) > dynamic_threshold_)
    {
        cout << "===" << dynamic_threshold_ << endl;
        return true;
    }
}

/**
 * @brief read depth value from the depth image
 *
 * @param I depth image
 * @param v event vector
 * @return double
 */
// deprecated: too slow
// inline double MotComp::ReadDepth(const cv::Mat& I, const int& x, const int&
// y) {
//   float depth = I.at<uint16_t>(x, y);
//   return sqrt(pow2(depth) / (1.0 + pow2(y - K(0, 2)) / pow2(K(0, 0)) +
//                              pow2(x - K(1, 2)) / pow2(K(1, 1))));
// }
inline double Eventor::ReadDepth(const cv::Mat &I, const int &x, const int &y)
{
    float depth = I.at<uint16_t>(y, x);
    return sqrt((depth * depth) /
                (1.0 + (x - K(0, 2)) * (x - K(0, 2)) / (K(0, 0) * K(0, 0)) +
                 (y - K(1, 2)) * (y - K(1, 2)) / (K(1, 1) * K(1, 1))));
}

// /**
//  * @brief convert a vector to homogeneous coordinates
//  *
//  * @param v
//  */
// inline void Eventor::ConvertToHomogeneous(Eigen::Vector3f *v)
// {
//     (*v)[0] = (*v)[0] / (*v)[2];
//     (*v)[1] = (*v)[1] / (*v)[2];
//     (*v)[2] = 1;
// }

/**
 * @brief check if this event in the bounds
 *
 * @param v
 * @return true: 0 < x < MAT_COLS && 0 < y < MAT_COLS
 * @return false
 */
inline bool Eventor::IsWithinTheBoundary(const Eigen::Vector3f &v)
{
    return (static_cast<int>(v[0]) >= 0 && static_cast<int>(v[0]) < MAT_COLS &&
            static_cast<int>(v[1]) >= 0 && static_cast<int>(v[1]) < MAT_ROWS);
}
inline bool Eventor::IsWithinTheBoundary(const int &x, const int &y)
{
    return (x >= 0 && x < MAT_COLS && y >= 0 && y < MAT_ROWS);
}

/**
 * @brief check if event's corresponding point in depth image at a valid
 * distance
 *
 * @param depthImg
 * @param x, y: event's corresponding point
 * @param min minimum depth value
 * @param max maximum depth value
 * @return true: this event in range
 * @return false
 */
inline bool Eventor::IsDepthInRange(const cv::Mat &depthImg, const int &x,
                                    const int &y, int min, int max)
{
    return depthImg.at<uint16_t>(y, x) <= max &&
           depthImg.at<uint16_t>(y, x) >= min;
}