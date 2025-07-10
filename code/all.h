#ifndef ALL_HEAD_H
#define ALL_HEAD_H

#define ROI_WIDTH  160
#define ROI_HEIGHT 80


#include <opencv2/opencv.hpp>
#include "image.h"
#include "Image_tracing.h"
#include "control_servo.h"
#include <atomic>
#include <thread>
#include <chrono>
#include <pthread.h>
#include "zf_common_headfile.h"
#include "pid.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>  // 用于异常处理
#endif //ALL_HEAD_H