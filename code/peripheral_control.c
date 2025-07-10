
#include "zf_common_headfile.h"

//距离累加
float sum_distance(float encoder_speed)
{
    float dist = 0;
    // dist = (encoder_speed * 0.05f)*k
    dist = (abs(encoder_speed) / 1024) * 0.048f * 3.14;
    return dist;
}


//电机赋值
void set_motor_direction_pwm(uint16_t motor_pin, uint16_t dir_pin, int16_t duty)
{
    if(duty > 0)
    {
        gpio_set_level(dir_pin, 1);
    }
    else
    {
        gpio_set_level(dir_pin, 0);
    }
    pwm_set_duty(motor_pin, abs(duty) * (PWM_MAX / 100));
}


void wheel_control()
{
    if(daohang ==0){
    while (1)
    {
        switch (mode)
        {
        case 1:
            duty_left = Incremental_PID_Calculate(&Pid_left, target_left, g_encoder_left);
            duty_right = Incremental_PID_Calculate(&Pid_right, target_right, g_encoder_right);
            set_motor_direction_pwm( MOTOR2_PWM,MOTOR2_DIR,abs(duty_left) * ( PWM_MAX/ 100) );
            set_motor_direction_pwm( MOTOR1_DIR,MOTOR1_PWM,abs(duty_left) * ( PWM_MAX/ 100) );
            system_delay_ms(10);
            break;

        case 4: // 倒车出库
            gpio_set_level(MOTOR1_DIR, 1);
            gpio_set_level(MOTOR2_DIR, 1);
            system_delay_ms(1000);
            while (distance <= 0.2)
            {

                pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(85));
                pwm_set_duty(MOTOR1_PWM, 10 * (PWM_MAX / 100));
                pwm_set_duty(MOTOR2_PWM, 10 * (PWM_MAX / 100));
                system_delay_ms(5);
            }
            while (distance <= 0.9)
            {
                pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(71));
                system_delay_ms(5);

                if (distance >= 0.8)
                {
                    pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(79));
                    pwm_set_duty(MOTOR1_PWM, 0);
                    pwm_set_duty(MOTOR2_PWM, 0);
                    system_delay_ms(5);
                    mode = 1;
                    break;
                }
            }
            break;
        case 2: // 倒车入库
            if(cnt ==2){
            pwm_set_duty(MOTOR1_PWM, 0);
            pwm_set_duty(MOTOR2_PWM, 0);

            system_delay_ms(1000);
            while (distance_1 <= 0.7)
            {
                gpio_set_level(MOTOR1_DIR, 0);
                gpio_set_level(MOTOR2_DIR, 0);
                pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(71));
                pwm_set_duty(MOTOR1_PWM, 10 * (PWM_MAX / 100));
                pwm_set_duty(MOTOR2_PWM, 10 * (PWM_MAX / 100));
                system_delay_ms(10);
            }
            // distance = 0 ;
            while (distance_1 <= 0.9) // 入库回正
            {
                pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(85));
                // system_delay_ms(1000);
                system_delay_ms(10);
                // distance = 0;
                if (distance_1 >= 0.8)
                {
                    pwm_set_duty(MOTOR1_PWM, 0);
                    pwm_set_duty(MOTOR2_PWM, 0);
                    mode = 1;
                    break;
                }
            }
        }
            break;
        case 3: // 侧方停车
        if(cnt == 1){
            gpio_set_level(MOTOR1_DIR, 0);
            gpio_set_level(MOTOR2_DIR, 0);
            pwm_set_duty(MOTOR1_PWM, 10 * (PWM_MAX / 100));
            pwm_set_duty(MOTOR2_PWM, 10 * (PWM_MAX / 100));
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(85));
            pwm_set_duty(MOTOR1_PWM, 0);
            pwm_set_duty(MOTOR2_PWM, 0);
            system_delay_ms(500);
            distance_2 = 0;
            while (1)
            {
                /* code */
                if (distance_2 >= 0.29)
                {
                    break;
                }
                else
                {
                    gpio_set_level(MOTOR1_DIR, 1);
                    gpio_set_level(MOTOR2_DIR, 1);
                    pwm_set_duty(MOTOR1_PWM, 10 * (PWM_MAX / 100));
                    pwm_set_duty(MOTOR2_PWM, 10 * (PWM_MAX / 100));
                }
                system_delay_ms(10);
                printf("qianjin\r\n");
            }
            gpio_set_level(MOTOR1_DIR, 0);
            gpio_set_level(MOTOR2_DIR, 0);
            pwm_set_duty(MOTOR1_PWM, 5 * (PWM_MAX / 100));
            pwm_set_duty(MOTOR2_PWM, 5 * (PWM_MAX / 100));
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(85));
            system_delay_ms(500);
            pwm_set_duty(MOTOR1_PWM, 0);
            pwm_set_duty(MOTOR2_PWM, 0);
            system_delay_ms(1000);
            distance_2 = 0;

            while (distance_2 <= 0.5)
            {
                gpio_set_level(MOTOR1_DIR, 0);
                gpio_set_level(MOTOR2_DIR, 0);
                pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(71));
                pwm_set_duty(MOTOR1_PWM, 10 * (PWM_MAX / 100));
                pwm_set_duty(MOTOR2_PWM, 10 * (PWM_MAX / 100));
                system_delay_ms(10);
            }
            while (distance_2 <= 0.7)
            {
                pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(85));
                system_delay_ms(10);
            }

            while (distance_2 <= 1.0)
            {
                pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(100));
                system_delay_ms(10);
                if (distance_2 >= 0.9)
                {

                    pwm_set_duty(MOTOR1_PWM, 0);
                    pwm_set_duty(MOTOR2_PWM, 0);
                    mode = 5;
                    distance = 0;
                    break;
                }
            }
            break;
        case 5: // 侧方出库
            gpio_set_level(MOTOR1_DIR, 1);
            gpio_set_level(MOTOR2_DIR, 1);
            system_delay_ms(1000);
            while (distance <= 0.3)
            {
                printf("666");
                pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(103));
                pwm_set_duty(MOTOR1_PWM, 10 * (PWM_MAX / 100));
                pwm_set_duty(MOTOR2_PWM, 10 * (PWM_MAX / 100));
                system_delay_ms(5);
            }
            while (distance <= 0.4)
            {
                pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(85));
                system_delay_ms(5);

                if (distance >= 0.35)
                {
                    pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(71));
                    pwm_set_duty(MOTOR1_PWM, 0);
                    pwm_set_duty(MOTOR2_PWM, 0);
                    system_delay_ms(5);
                    mode = 1;
                    break;
                }
            }
            break;
            }
            }
        }
    }
}

void imageProcessingThread(void) // 图像处理
{

    // 变量定义区
    cv::Mat frame;
    cv::Mat binary;
    cv::Mat binary_color;
    std::vector<cv::Point> largestAreaContour;                                      // 存储最大轮廓
    std::vector<cv::Point> left_points, right_points, center_points;                // 提取左右轮廓点
    std::vector<cv::Point> left_points_ture, right_points_ture, center_points_ture; // 最终左右轮廓点
    std::vector<cv::Point> left_points_temp, right_points_temp, center_points_temp; // 中间左右轮廓点
    constexpr std::array<double, 3> WEIGHTS = {0.2, 0.37, 0.43};                    // 加权值
    // 感兴趣区域（可调节）
    cv::Rect roiRect(0, 40, 160, 80);
    int cheku_mode = 0;

    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cerr << "无法打开摄像头！" << std::endl;
        return;
    }

    // 摄像头设置
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 160); // 你可以调整为摄像头支持的最佳分辨率
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
    cap.set(cv::CAP_PROP_FPS, 60);
    if(daohang==0){
    while (true)
    {

        try
        {
            if (mode != 1)
            {
                system_delay_ms(10);
                continue;
            }
            cap >> frame;
            cv::flip(frame, frame, -1); // 参数-1表示同时水平和垂直翻转
            if (frame.empty())
            {
                std::cerr << "空帧，跳过" << std::endl;
                continue;
            }

            // 二值化
            ProcessResult res = processFrameOptimized(frame, roiRect);

            findLargestContour(res.binaryROI, largestAreaContour);

            // 找左右边线
            processLeftRightPoints(largestAreaContour, left_points, right_points);

            // 打印
            std::cout << "left:";
            for (size_t i = 0; i < left_points.size(); ++i)
            {
                std::cout << left_points[i].x << " ";
            }
            std::cout << std::endl;
            std::cout << "right:";
            for (size_t i = 0; i < right_points.size(); ++i)
            {
                std::cout << right_points[i].x << " ";
            }
            std::cout << std::endl;

            // 判断是否为直线
            std::cout << "左轨迹: " << (isStraightLine(left_points) ? "是直线" : "不是直线") << std::endl;
            std::cout << "右轨迹: " << (isStraightLine(right_points) ? "是直线" : "不是直线") << std::endl;

            // 计算距离数组（假设图像高度为120）
            std::vector<int> distances = calculateVerticalDistances(left_points, right_points, 80);

            // 打印
            std::cout << "distances:";
            for (size_t i = 0; i < distances.size(); ++i)
            {
                std::cout << distances[i] << " ";
            }
            std::cout << std::endl;

            if (cheku_mode == 1)
            {
                auto region = find_special_region_pro(distances);
                if (region != -1)
                {
                    std::cout << "Start index: " << region << std::endl;
                }
                else
                {
                    std::cout << "No Start index found." << std::endl;
                }
                // 补线
                if (region != -1)
                {
                    cv::Point x = {left_points[79].x + 120, 79};
                    right_points_temp = getAllPointsOnLine(right_points[region - 3], x,
                                                           std::abs(right_points[region - 3].y - x.y) + 4);
                    std::cout << right_points[region - 3] << std::endl;
                    std::cout << x << std::endl;
                    right_points_ture = right_points;
                    replaceLongWithShort(right_points_ture, right_points_temp);
                }
                else
                {
                    mode = 3;
                    cheku_mode = 0;
                    cnt +=1;
                    right_points_ture = right_points;
                }
            }

            if (cheku_mode == 0 && ((isStraightLine(left_points)) == 1))
            {

                auto region = find_special_region(distances);
                if (region.first != -1)
                {
                    std::cout << "Start index: " << region.first << ", End index: " << region.second << std::endl;
                }
                else
                {
                    std::cout << "No region found." << std::endl;
                }
                // 补线
                if (region.first != -1)
                {
                    right_points_temp = getAllPointsOnLine(right_points[region.first - 3], right_points[region.second + 3],
                                                           std::abs(right_points[region.first - 3].y - right_points[region.second + 3].y) + 7);
                    right_points_ture = right_points;
                    replaceLongWithShort(right_points_ture, right_points_temp);
                    cheku_mode = 1;
                }
                else
                    right_points_ture = right_points;
            }
            else if (cheku_mode == 0)
                right_points_ture = right_points;

            // 找中线
            find_center(left_points, right_points_ture, center_points_ture);
            std::cout << center_points_ture.size() << std::endl;
            std::cout << left_points.size() << std::endl;
            std::cout << right_points_ture.size() << std::endl;

            // 计算加权点
            const double average = weighted_average
                (
                static_cast<double>(center_points_ture[70].x),
                static_cast<double>(center_points_ture[60].x),
                static_cast<double>(center_points_ture[50].x),
                WEIGHTS[0], WEIGHTS[1], WEIGHTS[2]
                );
            std::cout << "加权点：" << average << "\n";

            // 共享数据
            shared_average.store(average, std::memory_order_relaxed);
        }
        catch (const std::exception &e)
        {
            std::cerr << "图像处理异常: " << e.what() << std::endl;
        }
    }
    }
}



// 舵机线程
void pidThread(void)
{
    if(daohang== 0){
        PIDParams params;
        // 读取PID值
        if (readPIDParams("./params.txt", params))
        {
            std::cout << "Parameters loaded:\n"
                      << "  P: " << params.p << "\n"
                      << "  I: " << params.i << "\n"
                      << "  D: " << params.d << "\n";
        }
        else
        {
            std::cout << "Using default parameters\n";
        }

        PIDController_pro pid(params.p, params.i, params.d, 79.5);
        while (true)
        {
            switch (mode)
            {
                case 1:
                {
                    double current_average = shared_average.load(std::memory_order_relaxed);

                    double output = 85 + (pid.compute(current_average));
                    // if (std::abs(output) < 93.0) {  // 等价于 -5 < var < 5
                    //     output = 90.0;
                    // }

                    int servo_angle = 0 + static_cast<int>(output);
                    int min_ = std::min(100, servo_angle);
                    servo_angle = std::max(71, min_);
                    // if (servo_angle < 100)
                    // {
                    //     servo_angle = servo_angle + 1;
                    // }
                    pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(servo_angle));

                    // 执行控制输出，例如发送到硬件
                    std::cout << "PID Output: " << output << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    break;
                }
                default:
                    // 添加默认分支，避免忙等待
                        system_delay_ms(10);
                break;
            }
        }
    }
}


void sigint_handler(int signum)
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}


void all_cleanup()
{
    printf("程序异常退出，执行清理操作\n");
    // 处理程序退出！！！
    // 这里需要关闭电机，关闭电调等。
    timer->stop();
    system_delay_ms(10);

    pwm_set_duty(MOTOR1_PWM, 0);
    pwm_set_duty(MOTOR2_PWM, 0);
    pwm_set_duty(SERVO_MOTOR1_PWM, 0);
    printf("运行里程_1 = %.3fm\r\n", distance_1);
    printf("运行里程_2 = %.3fm\r\n", distance_2);
    printf("运行里程 = %.3fm\r\n", distance);
    close_tcp_send_ladar();
    close_tcp_send_sensor();
    close_tcp_recv_control();
}






