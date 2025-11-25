#include <opencv2/opencv.hpp>
#include <chrono>
using namespace std;
using namespace cv;
void ReadImage();
void OutputImage();
void showFrame();
extern int image_process_start_idx;
extern int image_process_end_idx;
extern cv::VideoCapture cap;
extern cv::Mat frame;
extern cv::Mat grayFrame;
extern cv::Mat binaryFrame;