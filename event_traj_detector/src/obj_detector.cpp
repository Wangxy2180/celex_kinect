#include "event_traj_detector/obj_detector.h"
#include "timer.h"
/**
 * @brief read data for detection object
 * @param event_counts
 * @param time_image
 */
void ObjDetector::LoadImages(const cv::Mat &event_counts,
                             const cv::Mat &event_image_bin)
{
    event_counts_ = event_counts;
    event_image_bin_ = event_image_bin;
}

// void ObjDetector::Detect()
// {
//     /* MorphologyOperations */
//     // 下边的函数利用时间图获取到了gray_image
//     MorphologyOperations(&gray_image_);

//     /* find the maximum value and its location */
//     cv::Point max_loc; // center-of-mass
//     cv::minMaxLoc(gray_image_, NULL, NULL, NULL, &max_loc);
//     // 这里是有问题的吧，直接这么去找极大值的位置

//     /* initialize ROI convergence */
//     cv::Rect ori_rect;
//     // 初始化一个大的ROI，大小为1/4个图像，max_loc在中心位置，然后对ROI进行边缘处理
//     InitRoi(gray_image_, max_loc, &ori_rect);
//     GetRect(gray_image_, &ori_rect);

//     cv::blur(gray_image_, iter_mat_, cv::Size(5, 5));
//     cv::threshold(iter_mat_, iter_mat_, 50, 255, cv::THRESH_TOZERO);

//     // 高斯迭代找出精细的roi
//     cv::Rect temp_rect;
//     bool is_roi = GaussianModelConvergence(iter_mat_, ori_rect, &processed_image_,
//                                            &temp_rect, 2);
//     if (is_roi)
//     {
//         /* if found the smallest ROI */
//         double ratio_x = 0, ratio_y = 0;
//         // 找到最小了，进行区域细化
//         GetRatio(processed_image_, &ratio_x, &ratio_y);
//         // 没看懂这一步是要干嘛，似乎是如果满足，那就要进行一步细化
//         if (IsTrue(temp_rect, last_rect_, ratio_x, ratio_y, k_ratio_thres_))
//         {
//             // processed_image是处理之后的ROI区域的图像，这一步就是在精细化rect，当然有可能精细化失败
//             // 如果上边的条件为真，那就说明误差太大了，还得细化
//             Mask(processed_image_, &temp_rect);
//         }
//         last_rect_ = temp_rect;
//         // is_object 只有这一处为true
//         is_object_ = true;
//     }
//     else
//     {
//         // 就直接认为他没找到了？
//         is_object_ = false;
//     }

//     /* corner case: check if roi large enough */
//     // 这一步还是跟之前干一样的事，担心last是空的,如果经历了上边的Mask，那么这个也一定能通过
//     if (last_rect_.area() < k_min_area_)
//     {
//         is_object_ = false; // reject small roi
//     }
//     else if (is_object_)
//     {
//         // 找包围此轮廓的最小矩形，计算非0像素的最小矩形
//         cv::Rect min_obj = cv::boundingRect(gray_image_(last_rect_));
//         // 更新roi
//         last_rect_.x += min_obj.x;
//         last_rect_.y += min_obj.y;
//         last_rect_.width = min_obj.width;
//         last_rect_.height = min_obj.height;
//     }
// }

void ObjDetector::getBlockCenPoint(int &x_idx, int &y_idx)
{

    Eigen::MatrixXd::Index rowMaxIdx, colMaxIdx;
    int rowMaxVal = block_rows.maxCoeff(&rowMaxIdx);
    int colMaxVal = block_cols.maxCoeff(&colMaxIdx);
    x_idx = colMaxIdx;
    y_idx = rowMaxIdx;

    // 他一定不会越界的
    // x_idx = colMaxIdx * BLOCK_SIZE + BLOCK_SIZE / 2;
    // y_idx = rowMaxIdx * BLOCK_SIZE + BLOCK_SIZE / 2;
}

bool ObjDetector::isObjAppear(int &rowMaxIdx, int &colMaxIdx)
{
    // 判断是否出现物体，同时获取事件量最大的block的idx
    float col_variance = (block_cols.mean() - block_cols).square().sum() / (MAT_COLS / BLOCK_SIZE);
    float row_variance = (block_rows.mean() - block_rows).square().sum() / (MAT_ROWS / BLOCK_SIZE);

    // int rowMaxIdx, colMaxIdx;
    getBlockCenPoint(colMaxIdx, rowMaxIdx);
    // cout << "----" << colMaxIdx << "," << rowMaxIdx << endl;
    if (col_variance > 1000 && row_variance > 1000)
    // if (col_variance > 100 && row_variance > 100)
    {
        // 30ms 1000 is good
        if (block_rows[rowMaxIdx] > 1000 && block_cols[colMaxIdx] > 1000)
        // if (block_rows[rowMaxIdx] > 500 && block_cols[colMaxIdx] > 500)
        {
            return true;
        }
    }
    is_object_ = false;
    return false;
}

void ObjDetector::InitRoiByBlock(const cv::Point &p, cv::Rect *dst)
{
    dst->width = MAT_COLS / 4;
    dst->height = MAT_ROWS / 4;
    dst->x = p.x - MAT_COLS / 8;
    dst->y = p.y - MAT_ROWS / 8;
}

void ObjDetector::getEventCntImg(cv::Mat &cntImg)
{
    Timer timerGenCnt;
    timerGenCnt.start();
    // 简单的来说,就是对时间图归一化,然后用平均时间做阈值,进行01区分
    cv::Mat normed_event_image = cv::Mat::zeros(cv::Size(cntImg.cols, cntImg.rows), CV_8UC1);

    cv::Mat tmpM; // image matrix
    // cv::Mat tmpM2; // image matrix

    /* normalization */
    cv::normalize(event_counts_, normed_event_image, 0, 255, cv::NORM_MINMAX);
    // cv::normalize(event_counts_, cntImg, 0, 255, cv::NORM_MINMAX);
    // return;
    // cv::imshow("norm time", normed_event_image);

    //mean函数 mask只有0和1的区别，1就是参与运算，0就是不参与,不是权重
    float thres = cv::mean(normed_event_image, event_counts_)[0] +
                  k_a_th_ * k_omega_ + k_b_th_;
    thres = cv::mean(normed_event_image, event_counts_)[0];
    // 小于阈值为0，大于为原始值
    cv::threshold(normed_event_image, tmpM, thres, 1, cv::THRESH_TOZERO);

    // // 使用上边阈值的方法，rect更小，下边的方法，更稳定this is better吗
    cv::threshold(normed_event_image, tmpM, 0, 255, cv::THRESH_OTSU);
    // cv::threshold(normed_event_image, cntImg, 0, 255, cv::THRESH_OTSU);
    // return;

    /* Gaussian Blur */
    cv::blur(tmpM, tmpM, cv::Size(5, 5));
    cv::normalize(tmpM, tmpM, 0, 255, cv::NORM_MINMAX);

    /* Morphology */
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(k_rect_kernel_size_, k_rect_kernel_size_));

    // ///////////////////////////////////////
    // 开操作，先腐蚀再膨胀
    cv::morphologyEx(tmpM, tmpM, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
    // timercnt.stop();
    // std::cout << "cnt img222: " << timercnt.getElapsedMilliseconds() << "ms\n";
    /* 图像按位平方，增加对比度 */
    // cv::imshow("morph", tmpM);
    cv::normalize(tmpM, cntImg, 0, 255, cv::NORM_MINMAX);

    // cv::imshow("gray", cntImg);
    // 时间图归一化了三次
    if (1)
    {
        static int save_num = 0;
        cv::imwrite("/home/free/catkin_cd/datasets/block_image/event_norm/" + to_string(save_num) + ".png", normed_event_image);
        cv::imwrite("/home/free/catkin_cd/datasets/block_image/event_gray/" + to_string(save_num) + ".png", cntImg);
        save_num++;
    }

    timerGenCnt.stop();
    std::cout << "generate cnt img: " << timerGenCnt.getElapsedMilliseconds() << "ms\n";
}

void ObjDetector::edgeDetect()
{
    Timer timeedge;
    timeedge.start();

    // 满足方差条件才进行计算

    int rowMaxBlockIdx, colMaxBlockIdx;
    bool isAppear = isObjAppear(rowMaxBlockIdx, colMaxBlockIdx);
    if (isAppear)
    {
        // 这里可以做一步操作吧，直接切出roi区域然后进行操作，可以省时间啊
        // 因为需要做全局优化吧，所以还是整体来搞比较好
        getEventCntImg(gray_image_);

        /* initialize ROI convergence */
        cv::Rect ori_rect;
        cv::Point max_loc = cv::Point(colMaxBlockIdx * BLOCK_SIZE + BLOCK_SIZE / 2, rowMaxBlockIdx * BLOCK_SIZE + BLOCK_SIZE / 2); // center-of-mass

        // 初始化一个大的ROI，大小为1/16图像，max_loc在中心位置，然后对ROI进行边缘处理
        InitRoiByBlock(max_loc, &ori_rect);
        GetRect(gray_image_, &ori_rect);
        // imshow("gray_cut",gray_image_(ori_rect));
        // test_mat.at<uchar>(rowMaxBlockIdx * BLOCK_SIZE, colMaxBlockIdx * BLOCK_SIZE) = 255;
        {
            // cv::Mat init_roi_img;
            // cv::rectangle(gray_image_, ori_rect, (255, 255, 255), 2);
            // cv::imshow("init roi", gray_image_);
        }
        ////////////////////////////////unused/////////////////////////////////////////////////////
        // cv::Mat iter_mat_;
        // cv::blur(gray_image_, iter_mat_, cv::Size(5, 5));
        // cv::threshold(iter_mat_, iter_mat_, 50, 255, cv::THRESH_TOZERO);
        // // imshow("iter", iter_mat_);
        // // timeee.stop();
        // // std::cout << "blur: " << timeee.getElapsedMilliseconds() << "ms\n";
        // imshow("gray obj app",gray_image_);
        // imshow("iter obj app",iter_mat_);
        // // 高斯迭代找出精细的roi
        // cv::Rect temp_rect;
        // bool is_roi = GaussianModelConvergence(iter_mat_, ori_rect, &processed_image_,&temp_rect, 2);
        /////////////////////////////////////////////////////////////////////////////////////////////////////////

        cv::Rect temp_rect;
        bool is_roi = GaussianModelConvergence(gray_image_, ori_rect, &processed_image_, &temp_rect, 2);
        if (is_roi)
        {
            /* if found the smallest ROI */
            double ratio_x = 1, ratio_y = 1;
            // 找到最小了，进行区域细化
            // GetRatio(processed_image_, &ratio_x, &ratio_y);
            // 没看懂这一步是要干嘛，似乎是如果满足，那就是偏差太大，要进行一步细化
            if (IsTrue(temp_rect, last_rect_, ratio_x, ratio_y, k_ratio_thres_))
            {
                // cout<<"??????????????????????process"<<endl;
                // processed_image是处理之后的ROI区域的图像，这一步就是在精细化rect，当然有可能精细化失败
                // 如果上边的条件为真，那就说明误差太大了，还得细化
                Mask(processed_image_, &temp_rect);
            }
            last_rect_ = temp_rect;
            // is_object 只有这一处为true
            is_object_ = true;
        }
        else
        {
            // 就直接认为他没找到了？
            is_object_ = false;
            cout << "false" << endl;
        }

        // 这一步还是跟之前干一样的事，担心last是空的,如果经历了上边的Mask，那么这个也一定能通过
        if (last_rect_.area() < k_min_area_)
        {
            is_object_ = false; // reject small roi
        }

        if (is_object_)
        {
            // 找包围此轮廓的最小矩形，计算非0像素的最小矩形
            cv::Rect min_obj = cv::boundingRect(gray_image_(last_rect_));
            // 更新roi
            last_rect_.x += min_obj.x;
            last_rect_.y += min_obj.y;
            last_rect_.width = min_obj.width;
            last_rect_.height = min_obj.height;
        }
        if (1)
        {
            static int cccnt = 0;
            drawRectForThesis(cccnt, ori_rect);
            if (cccnt == 7)
            {
                // sleep(100);
            }
            cccnt++;
            cv::rectangle(test_mat, last_rect_, cv::Scalar(255, 255, 255));
            imshow("max_ele", test_mat);
            cv::waitKey(1);
        }
    }
    timeedge.stop();
    if (isAppear)
    {
        std::cout << "edge detect: " << timeedge.getElapsedMilliseconds() << "ms\n";
        // total_detect_time += timeedge.getElapsedMilliseconds();
        // total_detect_cnt += 1;
        // std::cout << "avg edge detect: " << total_detect_time / total_detect_cnt << "ms\n";
    }
    else
    {
        std::cout << "empty detect: " << timeedge.getElapsedMilliseconds() << "ms\n";
    }
}

void ObjDetector::LoadEdge(const Eigen::Array<int, MAT_ROWS / BLOCK_SIZE, 1> &rowVar,
                           const Eigen::Array<int, MAT_COLS / BLOCK_SIZE, 1> &colVar)
{
    block_rows = rowVar;
    block_cols = colVar;
    // block_rows.assign(rowBloc.begin(), rowBloc.end());
    // block_cols.assign(colBloc.begin(), colBloc.end());
}

void ObjDetector::LoadEdgePxiel(const Eigen::Array<int, MAT_ROWS, 1> &rowVar,
                                const Eigen::Array<int, MAT_COLS, 1> &colVar)
{
    pixel_rows = rowVar;
    pixel_cols = colVar;
    // block_rows.assign(rowBloc.begin(), rowBloc.end());
    // block_cols.assign(colBloc.begin(), colBloc.end());
}

// /**
//  * @brief image processing before bounding box convergence
//  *
//  * @param dst CV_8UC1 image
//  */
// void ObjDetector::MorphologyOperations(cv::Mat *dst)
// {
//     // 简单的来说,就是对时间图归一化,然后用平均时间做阈值,进行01区分
//     cv::Mat normed_time_image =
//         cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_32FC1);

//     cv::Mat m; // image matrix

//     /* normalization */
//     cv::normalize(time_image_, normed_time_image, 0, 1, cv::NORM_MINMAX);
//     cv::imshow("norm time", normed_time_image);

//     /* thresholding */ //mean函数 mask只有0和1的区别，1就是参与运算，0就是不参与,不是权重
//     float thres = cv::mean(normed_time_image, event_counts_)[0] +
//                   k_a_th_ * k_omega_ + k_b_th_;
//     thres = cv::mean(normed_time_image, event_counts_)[0];
//     // 小于阈值为0，大于为原始值
//     cv::threshold(normed_time_image, m, thres, 1, cv::THRESH_TOZERO);

//     /* Gaussian Blur */
//     cv::blur(m, m, cv::Size(5, 5));
//     cv::normalize(m, m, 0, 255, cv::NORM_MINMAX);

//     /* Morphology */
//     cv::Mat kernel = cv::getStructuringElement(
//         cv::MORPH_RECT, cv::Size(k_rect_kernel_size_, k_rect_kernel_size_),
//         cv::Point(-1, -1));
//     // 开操作，先腐蚀再膨胀
//     cv::morphologyEx(m, m, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);

//     /* 图像按位平方，增加对比度 */
//     m = m.mul(m);
//     cv::imshow("morph", m);
//     cv::normalize(m, m, 0, 255, cv::NORM_MINMAX);
//     m.convertTo(*dst, CV_8UC1);
//     cv::imshow("gray", *dst);
//     // 时间图归一化了三次
// }

/**
 * @brief Gaussian Model iteration to find out the optimal ROI
 *
 *
 * we use second order central moments of events clusters to represent the
 * variance of the cluster, then we compute the standard deviation for
 * ROI convergence
 *
 * @param src
 * @param dst output image
 * @param iterTimes
 * @return true
 * @return false
 */
bool ObjDetector::GaussianModelConvergence(const cv::Mat &msrc,
                                           const cv::Rect &rsrc, cv::Mat *mdst,
                                           cv::Rect *rdst, int iterTimes)
{
    //  反正就是我看不懂的内容了，应该就是算法1吧
    cv::Rect iter_rect = rsrc;

    cv::Mat matrix = msrc(iter_rect); // matirx for iteration

    cv::Rect final_rect = iter_rect;
    // 两轮迭代
    for (int i = 0; i < iterTimes; i++)
    {
        // 这里的matrix是rect中的哦
        cv::Moments m = cv::moments(matrix);
        // 计算中心矩的公式，分别为xy，也就是求轮廓的质心（均值）
        cv::Point center_mass(static_cast<int>(m.m10 / m.m00),
                              static_cast<int>(m.m01 / m.m00));

        // 好家伙，这个搞宽高的方法真奇特（方差）
        double width, height;
        int epsilon_x = static_cast<int>(sqrt(m.mu20 / m.m00));
        int epsilon_y = static_cast<int>(sqrt(m.mu02 / m.m00));
        width = 4 * epsilon_x;
        height = 4 * epsilon_y;

        if (width == 0 || height == 0)
        {
            final_rect.width = 0;
            final_rect.height = 0;
            *mdst = matrix;
            *rdst = final_rect;
            return false;
        }
        // 就是边界处理，别让他超出范围
        IntTruncate(matrix.cols, &width);
        IntTruncate(matrix.rows, &height);

        // 下边就是单纯的赋值
        iter_rect.width = static_cast<int>(width);
        iter_rect.height = static_cast<int>(height);
        iter_rect.x = center_mass.x - iter_rect.width / 2;
        iter_rect.y = center_mass.y - iter_rect.height / 2;
        // 因为左上角可能小于0，宽高也可能超出边界
        GetRect(matrix, &iter_rect);
        final_rect.x += iter_rect.x;
        final_rect.y += iter_rect.y;
        final_rect.width = iter_rect.width;
        final_rect.height = iter_rect.height;

        matrix = matrix(iter_rect);
    }

    // 他就是算了个时间图的方差，如果边框拟合的很好，方差应该很小
    double variance = 0.0f;
    GetVariance(matrix, &variance);

    // 这里就是那个if-break吧，就是计算两次迭代之后的误差值
    // 为啥方差小，范围要给他的宽高重设，并为false呢？是因为方差小的时候，就是检测到背景了？
    if (variance < 1500)
    {
        final_rect.width = 0;
        final_rect.height = 0;
        *mdst = matrix;
        *rdst = final_rect;
        return false;
    }
    else
    {
        *mdst = matrix;
        *rdst = final_rect;
        return true;
    }
}

/**
 * @brief
 *
 * @param src
 * @param roi
 */
void ObjDetector::Mask(const cv::Mat &src, cv::Rect *roi)
{
    // 本质上就是在精细化rect，但是有可能精细化失败
    cv::Mat img_bool, canny, labels, stats, centroids;
    src.convertTo(img_bool, CV_8UC1);
    // 使用OTSU算法自动选择最佳阈值，该方法使得前景与背景之间方差最大
    cv::threshold(img_bool, img_bool, 0, 255, cv::THRESH_OTSU);

    // 寻找连通区域，返回值为连通区域的个数
    // stats分别对应各个轮廓的x,y,width,height和面积。注意0的区域标识的是background
    // centroids则对应的是中心点
    // label则对应于表示是当前像素是第几个轮廓
    int n_connected_components =
        cv::connectedComponentsWithStats(img_bool, labels, stats, centroids);

    /* init order map */
    std::map<float, int> mean_time_clusters;

    for (size_t i = 0; i < n_connected_components; i++)
    {
        if (stats.at<int>(i, cv::CC_STAT_AREA) < k_min_area_)
        {
            continue; // skip too small areas
        }
        else
        {
            cv::Rect2f rect(stats.at<int>(i, cv::CC_STAT_LEFT),
                            stats.at<int>(i, cv::CC_STAT_TOP),
                            stats.at<int>(i, cv::CC_STAT_WIDTH),
                            stats.at<int>(i, cv::CC_STAT_HEIGHT));

            /* crop image */
            cv::Mat cropped_img = src(rect);

            /* get mean time of the ROI */
            float avg = cv::mean(cropped_img, cropped_img)[0];
            // 把时间平均值和对应的序号插入map中
            mean_time_clusters.insert(std::pair<float, int>(avg, i));
        }
    }
    // 如果为空，那就是一个连通区域都没有roi就是没变化
    if (!mean_time_clusters.empty())
    {
        // 因为他是有序map，所以平均时间的最大值一定在最后处
        auto highest_mean_time = mean_time_clusters.end();
        highest_mean_time--; // select cluster with the highest mean time
        int idx = highest_mean_time->second;

        /* update ROI */
        // cv::Rect2f r(stats.at<int>(idx, cv::CC_STAT_LEFT),
        //              stats.at<int>(idx, cv::CC_STAT_TOP),
        //              stats.at<int>(idx, cv::CC_STAT_WIDTH),
        //              stats.at<int>(idx, cv::CC_STAT_HEIGHT));
        roi->x += stats.at<float>(idx, cv::CC_STAT_LEFT);
        roi->y += stats.at<int>(idx, cv::CC_STAT_TOP);
        roi->width = stats.at<int>(idx, cv::CC_STAT_WIDTH);
        roi->height = stats.at<int>(idx, cv::CC_STAT_HEIGHT);
    }
}

/**
 * @brief   1/4 of image is set to be initial ROI of Gaussian Model Cluster
 *
 * @param src
 * @param p
 * @param dst
 */
void ObjDetector::InitRoi(const cv::Mat &src, const cv::Point &p,
                          cv::Rect *dst)
{
    dst->width = src.cols / 2;
    dst->height = src.rows / 2;
    dst->x = p.x - src.cols / 4;
    dst->y = p.y - src.rows / 4;
}

/**
 * @brief
 *
 * @param src
 * @param ratio_x
 * @param ratio_y
 */
void ObjDetector::GetRatio(const cv::Mat &src, double *ratio_x,
                           double *ratio_y)
{
    // 这个函数基本没有用处，因为噪声太大了
    // 计算的是xy方向上，第一次出现事件的位置/总量 是个比值。应该是在扣边缘，理论上这两个数应该接近1啊
    // 从上向下遍历,计算有多少行存在事件
    double cnt_x = 0, cnt_y = 0;
    for (int i = 0; i < src.rows; i++)
    {
        for (int j = 0; j < src.cols; j++)
        {
            if (src.at<uchar>(i, j))
            {
                cnt_x++;
                break;
            }
        }
    }

    *ratio_x = cnt_x / src.rows;

    // 从左向右遍历,计算有多少列存在事件
    for (int j = 0; j < src.cols; j++)
    {
        for (int i = 0; i < src.rows; i++)
        {
            if (src.at<uchar>(i, j))
            {
                cnt_y++;
                break;
            }
        }
    }

    *ratio_y = cnt_y / src.cols;
    cout << "///////////////kkkkkdskskdkd：" << *ratio_x << "," << *ratio_y << endl;
}

/**
 * @brief
 *
 * @param src source image
 * @param O rectangle
 * @return cv::Rect
 */
void ObjDetector::GetRect(const cv::Mat &src, cv::Rect *O)
{
    if (O->x < 0)
    {
        O->width -= O->x; // left edge moves right
        O->x = 0;
    }

    if (O->x + O->width > src.cols - 1)
    {
        O->width = src.cols - 1 - O->x; // right edge truncates
    }

    if (O->y < 0)
    {
        O->height -= O->y;
        O->y = 0;
    }

    if (O->y + O->height > src.rows - 1)
    {
        O->height = src.rows - 1 - O->y;
    }
}

/**
 * @brief Type conversion and image visualization
 *    TODO: Node Handle required
 * @param src
 */
cv::Mat ObjDetector::GetVisualization()
{
    cv::Mat m, m_color;
    // gray_image_.convertTo(m, CV_8UC1);
    cv::normalize(gray_image_, m, 0, 255, cv::NORM_MINMAX);
    cv::applyColorMap(m, m_color, cv::COLORMAP_JET);
    if (is_object_)
    { // draw bounding box
        cv::rectangle(m_color, last_rect_, cv::Scalar(0, 255, 0), 2, cv::LINE_8, 0);
    }
    return m_color;
}

cv::Mat ObjDetector::getEventVis()
{
    cv::Mat m_event;
    // gray_image_.convertTo(m, CV_8UC1);
    // cv::normalize(event_image_, m, 0, 255, cv::NORM_MINMAX);
    // cv::applyColorMap(m, m_color, cv::COLORMAP_JET);
    if (is_object_)
    { // draw bounding box
        m_event = event_image_bin_;
        cv::rectangle(m_event, last_rect_, cv::Scalar(255, 255, 255), 2, cv::LINE_8, 0);
    }
    return m_event;
}

/**
 * @brief truncate value w.r.t ref
 *
 * @param ref
 * @param value
 */
inline void ObjDetector::IntTruncate(const int &ref, double *value)
{
    if (*value > ref)
    {
        *value = ref;
    }
}

/**
 * @brief compute average (in order to compute variance)
 *
 * @param m
 * @param avg
 * @param num
 */
inline void ObjDetector::GetAverage(const cv::Mat m, double *avg, int *num)
{
    // int n = *num;
    double sum;
    for (int i = 0; i < m.rows; i++)
    {
        for (int j = 0; j < m.cols; j++)
        {
            if (m.at<uchar>(i, j))
            {
                uchar v = m.at<uchar>(i, j);
                sum += v;
                (*num)++;
            }
        }
    }
    *avg = sum / *num;
}

/**
 * @brief compute variance
 *
 * @param m
 * @param var
 */
inline void ObjDetector::GetVariance(const cv::Mat m, double *var)
{
    // double variance = *var;
    double average = 0.0f, square_sum = 0.0f;
    int n = 0;
    // 顾名思义，就是算了个平均灰度值，但是为啥不直接用mean函数算呢
    GetAverage(m, &average, &n);

    // 算方差。。。我不理解，写得好复杂。。。
    for (int i = 0; i < m.rows; i++)
    {
        for (int j = 0; j < m.cols; j++)
        {
            if (m.at<uchar>(i, j))
            {
                uchar t = m.at<uchar>(i, j);
                square_sum += (t - average) * (t - average);
            }
        }
    }

    *var = square_sum / n;
}

// 我了解了，只有第一次时src2才是空的，后续时才是上一轮次的rect值啊
// 但是默认rect的宽高值是0啊，不会出问题吗？我试了一下，求出来的值是inf，似乎对于double类型的值引入了这样的计算
inline bool ObjDetector::IsTrue(const cv::Rect &curRect, const cv::Rect &lastRect,
                                const double &rx, const double &ry,
                                const double &thres)
{
    // 为啥是0.25这么小的值
    // return rx < 0.25 || ry < 0.25 ||
    return static_cast<double>(curRect.width * curRect.height) /
               static_cast<double>(lastRect.width * lastRect.height) >
           thres;
}

void ObjDetector::drawRectForThesis(const int idx, const cv::Rect &ori_rect)
{

    {
        // cout << block_rows << endl
        //      << "--------------" << endl
        //      << block_cols << endl;
        // cout << "=================================" << endl;
        // cout << pixel_rows << endl
        //      << "--------------" << endl
        //      << pixel_cols << endl;
    }
    cv::Mat image_draw_init_roi;
    cv::cvtColor(gray_image_, image_draw_init_roi, CV_GRAY2RGB);
    cv::rectangle(image_draw_init_roi, ori_rect, cv::Scalar(0, 255, 0), 3);
    cv::imwrite("/home/free/catkin_cd/datasets/block_image/image_init_roi/" + to_string(idx) + ".png", image_draw_init_roi);

    cv::cvtColor(gray_image_, image_draw_init_roi, CV_GRAY2RGB);
    cv::rectangle(image_draw_init_roi, last_rect_, cv::Scalar(0, 255, 0), 3);
    cv::imwrite("/home/free/catkin_cd/datasets/block_image/image_final_roi/" + to_string(idx) + ".png", image_draw_init_roi);

    cv::cvtColor(event_image_bin_, image_draw_init_roi, CV_GRAY2RGB);
    cv::imwrite("/home/free/catkin_cd/datasets/block_image/event_bin/" + to_string(idx) + ".png", event_image_bin_);
    cv::rectangle(image_draw_init_roi, last_rect_, cv::Scalar(0, 255, 0), 3);
    cv::imwrite("/home/free/catkin_cd/datasets/block_image/event_bin_final_roi/" + to_string(idx) + ".png", image_draw_init_roi);
}
