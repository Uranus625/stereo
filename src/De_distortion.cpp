#include "De_distortion.h"
       
cv::Mat F_MAT_left, F_MAT_right; //内参矩阵
cv::Mat C_MAT_left, C_MAT_right; //畸变系数
cv::Mat R, T, E, F; //R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵
cv::Mat R1, R2, P1, P2, Q; //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
cv::Mat mapx1, mapy1, mapx2, mapy2; //映射表

void load_param()
{
    cv::FileStorage fin("/home/uranus/桌面/stereo/param/left_camera.yml", cv::FileStorage::READ);
    fin["K"] >> F_MAT_left; //内参矩阵
    fin["D"] >> C_MAT_left;  //畸变系数

    cv::FileStorage fin2("/home/uranus/桌面/stereo/param/right_camera.yml", cv::FileStorage::READ);
    fin2["K"] >> F_MAT_right; //内参矩阵
    fin2["D"] >> C_MAT_right;  //畸变系数
}

void De_distortion(cv::Mat &left_image, cv::Mat &right_image)
{
    load_param();
    // std::cout << F_MAT_right << std::endl;
    // std::cout << C_MAT_right << std::endl;
    cv::Mat undistortedImage = left_image.clone();
    cv::undistort(undistortedImage, left_image, F_MAT_left, C_MAT_left);

    undistortedImage = right_image.clone();
    cv::undistort(undistortedImage, right_image, F_MAT_right, C_MAT_right); 
}

void stereo_correction(cv::Mat &left_image, cv::Mat &right_image)
{
    cv::FileStorage fin("/home/uranus/桌面/stereo/param/stereo.yml", cv::FileStorage::READ);
    fin["R"] >> R; 
    fin["T"] >> T;   
    //生成立体映射矩阵
    cv::stereoRectify(F_MAT_left, C_MAT_left, F_MAT_right, C_MAT_right, left_image.size(), R, T, R1, R2, P1, P2, Q);
    //生成映射表
    cv::initUndistortRectifyMap(F_MAT_left, C_MAT_left, R1, P1, left_image.size(), CV_16SC2, mapx1, mapy1);
    cv::initUndistortRectifyMap(F_MAT_right, C_MAT_right, R2, P2, right_image.size(), CV_16SC2, mapx2, mapy2);
    //应用立体映射
    cv::Mat undistortedImage = left_image.clone();
    cv::remap(undistortedImage, left_image, mapx1, mapy1, cv::INTER_LINEAR);
    undistortedImage = right_image.clone();
    cv::remap(undistortedImage, right_image, mapx2, mapy2, cv::INTER_LINEAR);
}

void disparity_image(cv::Mat &left_image, cv::Mat &right_image)
{
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 16, 3, 8 * 9 * 9, 32 * 9 * 9, 1, 24, 10, 100, 32);

    cv::Mat disparity_sgbm, disparity;
    sgbm->compute(left_image, right_image, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    insertDepth32f(disparity);

    double fx = 1581.9, fy = 1582.4, cx = 894.2269, cy = 520.4674;
    // 基线
    double b = 0.6;

    // 生成点云
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pointcloud;

    // 如果你的机器慢，请把后面的v++和u++改成v+=2, u+=2
    for (int v = 0; v < left_image.rows; v++)
        for (int u = 0; u < left_image.cols; u++) {
            if (disparity.at<float>(v, u) <= 0.0) continue;

            Eigen::Vector4d point(0, 0, 0, left_image.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色

            // 根据双目模型计算 point 的位置
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double depth = fx * b / (disparity.at<float>(v, u));
            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;

            pointcloud.push_back(point);
        }
    cv::imshow("disparity", disparity / 48.0);
    cv::waitKey(0);
    // 画出点云
    showPointCloud(pointcloud);
}

void insertDepth32f(cv::Mat& depth) //空洞填充
{
    const int width = depth.cols;
    const int height = depth.rows;
    float* data = (float*)depth.data;
    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
    double* integral = (double*)integralMap.data;
    int* ptsIntegral = (int*)ptsMap.data;
    memset(integral, 0, sizeof(double) * width * height);
    memset(ptsIntegral, 0, sizeof(int) * width * height);
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                integral[id2] = data[id2];
                ptsIntegral[id2] = 1;
            }
        }
    }
    // 积分区间
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 1; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - 1];
            ptsIntegral[id2] += ptsIntegral[id2 - 1];
        }
    }
    for (int i = 1; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - width];
            ptsIntegral[id2] += ptsIntegral[id2 - width];
        }
    }
    int wnd;
    double dWnd = 2;
    while (dWnd > 1)
    {
        wnd = int(dWnd);
        dWnd /= 2;
        for (int i = 0; i < height; ++i)
        {
            int id1 = i * width;
            for (int j = 0; j < width; ++j)
            {
                int id2 = id1 + j;
                int left = j - wnd - 1;
                int right = j + wnd;
                int top = i - wnd - 1;
                int bot = i + wnd;
                left = std::max(0, left);
                right = std::min(right, width - 1);
                top = std::max(0, top);
                bot = std::min(bot, height - 1);
                int dx = right - left;
                int dy = (bot - top) * width;
                int idLeftTop = top * width + left;
                int idRightTop = idLeftTop + dx;
                int idLeftBot = idLeftTop + dy;
                int idRightBot = idLeftBot + dx;
                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
                if (ptsCnt <= 0)
                {
                    continue;
                }
                data[id2] = float(sumGray / ptsCnt);
            }
        }
        int s = wnd / 2 * 2 + 1;
        if (s > 201)
        {
            s = 201;
        }
        cv::GaussianBlur(depth, depth, cv::Size(s, s), s, s);
    }
}

void showPointCloud(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        std::cerr << "Point cloud is empty!" << std::endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}
