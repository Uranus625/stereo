#include <opencv2/opencv.hpp>
#include "De_distortion.h"

int main()
{
    cv::Mat right_image = cv::imread("/home/uranus/桌面/stereo/image/right/right.jpg");
    cv::Mat left_image = cv::imread("/home/uranus/桌面/stereo/image/left/left.jpg");

    De_distortion(left_image, right_image);

    // cv::imshow("right_image", right_image);
    cv::imshow("left_image", left_image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

// int main() {
//     cv::VideoCapture cap(2);
//     if (!cap.isOpened()) {
//         std::cout << "无法打开摄像头" << std::endl;
//         return -1;
//     }

//     cap.set(cv::CAP_PROP_FRAME_WIDTH, 3840);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

//     cv::namedWindow("Camera", cv::WINDOW_NORMAL);
//     cv::namedWindow("right", cv::WINDOW_NORMAL);
//     cv::namedWindow("left", cv::WINDOW_NORMAL);

//     while (true) {
//         cv::Mat frame;
//         cap >> frame; 

//         cv::Rect roi1(0, 0, 1920, 1080);
//         cv::Rect roi2(1920, 0, 1920, 1080);
//         cv::Mat right = frame(roi1);
//         cv::Mat left = frame(roi2);

//         cv::imshow("right", right);
//         cv::imshow("left", left);

//         if (cv::waitKey(1) == 27) {
//             break;
//         }
//     }

//     cap.release();
//     cv::destroyAllWindows();

//     return 0;
// }
