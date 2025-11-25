#include <iostream>
#include "search.h"
#include "camera.h"

#include "zaber_detector.cpp"

using namespace std;

// 斑马线检测
bool g_has_zebra = false;
double g_zebra_k = 0.0;
// // 停车库检测
// extern int g_garage_state; // 0=不在，1=在
// extern int g_garage_event; // 0=无，1=进入，2=离开

int main() {
    for(;image_process_start_idx <= image_process_end_idx; image_process_start_idx++){
        // std::cout << "image_process_start_idx: " << image_process_start_idx << std::endl;
        // std::cout << "image_process_end_idx: " << image_process_end_idx << std::endl;
        try {
            ReadImage();
        } catch (const cv::Exception& e) {
            // std::cout << "Error: " << e.what() << std::endl;
            continue;
        }
        zebra::detect_and_fit_zebra(grayFrame, g_has_zebra, g_zebra_k);
        { 
    static int _zebra_state = -1; 
    int cur = (g_has_zebra ? 1 : 0);
    // printf("[ZEBRA] 斑马线检测: %s\n", cur ? "启动" : "解除");
    // if (cur != _zebra_state) {
    //     printf("进入斑马线判断");
        _zebra_state = cur;
        if (g_has_zebra){
            // printf("[ZEBRA] 斑马线检测: 启动\n");
            printf("[ZEBRA] 斑马线检测: 命中, 斜率=%.3f\n", g_zebra_k);
        }else 
            printf("[ZEBRA] 斑马线检测: 解除\n");
        }
    // }
        // std::cout<<"定位search 开始"<<std::endl;
        Search();
        showFrame();
        OutputImage();
    }
    return 0;
}