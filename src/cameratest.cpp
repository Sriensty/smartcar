#include "camera.h"
#include "search.h"
#include <iostream>
#include <string>             // 用于文件名生成
#include <opencv2/opencv.hpp> // 添加OpenCV头文件
#include <iostream>

cv::VideoCapture cap;
cv::Mat frame;
cv::Mat grayFrame;
cv::Mat binaryFrame;
int image_process_start_idx = 906;
int image_process_end_idx = 924;
void ReadImage(){
    grayFrame = cv::imread("/home/sriensty/使用双系统的复制该压缩包/input/" + to_string(image_process_start_idx) + ".jpg");
    // 将图像转换为灰度图像
    cv::cvtColor(grayFrame, grayFrame, cv::COLOR_BGR2GRAY);
    // 二值化（OTSU自动阈值）
    IMAGE_MIDDLE = cv::threshold(grayFrame, binaryFrame, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    std::cout << "grayFrame尺寸: " << grayFrame.rows << " x " << grayFrame.cols << std::endl;
    // 坐标变换
    for (int j = 0; j < 197; ++j) {
        {
            for (int i = 0; i < 97; ++i)
            {
                g_VideoImageData[i][j] = grayFrame.at<uchar>(96 - i, 159 - j);
            }
        }
    }
}
void showFrame()
{
    cv::cvtColor(grayFrame, grayFrame, cv::COLOR_GRAY2BGR);
    cv::Scalar redColor(0, 0, 255);
    cv::Scalar greenColor(0, 255, 0);
    cv::Scalar blueColor(255, 0, 0);
    cv::Scalar yellowColor(255, 255, 0);
    for (int i = 0; i < g_LeftEdgeNum; i++)
    {
        cv::circle(grayFrame, cv::Point(g_LeftEdge[i].y, 97 - g_LeftEdge[i].x), 1, redColor, -1);
    }
    for (int i = 0; i < g_RightEdgeNum; i++)
    {
        cv::circle(grayFrame, cv::Point(g_RightEdge[i].y, 97 - g_RightEdge[i].x), 1, greenColor, -1);
    }
    for (int i = 0; i < g_CenterNum; i++)
    {
        cv::circle(grayFrame, cv::Point(g_CenterPosition[i].y, 97 - g_CenterPosition[i].x), 1, blueColor, -1);
    }
    cv::line(grayFrame, Point(g_DirectionControl, 97), Point(g_DirectionControl, 30), yellowColor, 1);
    // cv::imshow("Camera", frame);
    // cv::imshow("Camera1", binaryFrame);
}
void OutputImage(){
    const string outputPath = "/home/sriensty/使用双系统的复制该压缩包/output/" + to_string(image_process_start_idx) + ".jpg";
    try {
        bool success = cv::imwrite(outputPath, grayFrame);
        if (success) {
            std::cout << outputPath << " 处理成功" << std::endl;
        } else {
            std::cerr << outputPath << " 处理失败" << std::endl;
        }
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV Error in OutputImage: " << e.what() << std::endl;
    }
}