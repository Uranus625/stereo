#include "De_distortion.h"

Eigen::Matrix3d F_left, F_right;
Eigen::Matrix<double, 1, 5> C_lefit, C_right;
cv::Mat F_MAT_left, F_MAT_right;
cv::Mat C_MAT_left, C_MAT_right;
void load_param()
{
    cv::FileStorage fin("/home/uranus/桌面/stereo/param/left_camera.yml", cv::FileStorage::READ);
    fin["K"] >> F_MAT_left; //内参矩阵
    fin["D"] >> C_MAT_left;  //畸变矩阵

    // cv::cv2eigen(F_MAT_left, F_left);
    // cv::cv2eigen(C_MAT_left, C_lefit);

    cv::FileStorage fin2("/home/uranus/桌面/stereo/param/right_camera.yml", cv::FileStorage::READ);
    fin2["K"] >> F_MAT_right; //内参矩阵
    fin2["D"] >> C_MAT_right;  //畸变矩阵

    // cv::cv2eigen(F_MAT_right, F_right);
    // cv::cv2eigen(C_MAT_right, C_right);
}

void De_distortion(cv::Mat &left_image, cv::Mat &right_image)
{
    load_param();
    // std::cout << F_MAT_right << std::endl;
    // std::cout << C_MAT_right << std::endl;
    cv::Mat undistortedImage;
    
    cv::undistort(left_image, undistortedImage, F_MAT_left, C_MAT_left);  
    cv::undistort(right_image, right_image, F_MAT_right, C_MAT_right);  
}
