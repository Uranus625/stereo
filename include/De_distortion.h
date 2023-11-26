#ifndef DE_DISTORTION_H
#define DE_DISTORTION_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <string>

void load_param();
void De_distortion(cv::Mat &left_image, cv::Mat &right_image);
void stereo_correction(cv::Mat &left_image, cv::Mat &right_image);
void disparity_image(cv::Mat &left_image, cv::Mat &right_image);
void insertDepth32f(cv::Mat& depth);
void showPointCloud(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud);

#endif
