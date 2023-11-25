#ifndef DE_DISTORTION_H
#define DE_DISTORTION_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

void load_param();
void De_distortion(cv::Mat &left_image, cv::Mat &right_image);

#endif
