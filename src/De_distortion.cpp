#include "De_distortion.h"

cv::Mat undistortedImage;
// Eigen::Matrix3d F_left, F_right;
// Eigen::Matrix<double, 1, 5> C_lefit, C_right;         
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
    undistortedImage = left_image.clone();
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
    std::cout << R1 << std::endl;
    std::cout << P1 << std::endl;
    //生成映射表
    cv::initUndistortRectifyMap(F_MAT_left, C_MAT_left, R1, P1, left_image.size(), CV_16SC2, mapx1, mapy1);
    cv::initUndistortRectifyMap(F_MAT_right, C_MAT_right, R2, P2, right_image.size(), CV_16SC2, mapx2, mapy2);
    //应用立体映射
    cv::Mat rectifiedLeft, rectifiedRight;
    // cv::Mat gray_Left, gray_Right;
    // cv::cvtColor(left_image, gray_Left, cv::COLOR_BGR2GRAY);  // 转换为灰度图
    cv::remap(left_image, rectifiedLeft, mapx1, mapy1, cv::INTER_LINEAR);
    cv::remap(right_image, rectifiedRight, mapx2, mapy2, cv::INTER_LINEAR);
    cv::imshow("right_image", rectifiedRight);
    cv::imshow("left_image", rectifiedLeft);
}
