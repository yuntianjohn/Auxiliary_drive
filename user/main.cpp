
#include "class_thread_wrapper.h"
#include "class_posix_pit.h"
#include "peripheral_control.h"

TimerFdWrapper *timer;
std::atomic<double> shared_average(79.5);
struct pwm_info servo_pwm_info;
int16 duty_left = 0;
int16 duty_right = 0;
float distance_1 = 0;
float distance_2 = 0;
float distance = 0;
int mode = 4; //
float target_left = 150;
float target_right = -150;
float v_target = 150;
float kp1 = 0;
float ki1 = 0.01;
float kp2 = 0;
float ki2 = 0.01;
int cnt = 0;
int daohang = 0;//雷达导航
int exiting =0;

// 定义线程回调函数，用于处理定时器到期事件
void timer_callback(void) 
{
    if(daohang==1)
  {
    imu_callback();
    encoder_callback();
    distance += sum_distance(g_encoder_left);//距离累加
    if (mode == 2)
    {
        distance_1 += sum_distance(g_encoder_left);
    }
    else if (mode == 3)
    {
        distance_2 += sum_distance(g_encoder_left);
    }
   duty_left = Incremental_PID_Calculate(&Pid_left,g_car_control.left_speed,g_encoder_left);
   duty_right = Incremental_PID_Calculate(&Pid_right,-g_car_control.right_speed,g_encoder_right);
   set_motor_direction_pwm( MOTOR2_PWM,MOTOR2_DIR,abs(duty_left) * ( PWM_MAX/ 100) );
   set_motor_direction_pwm( MOTOR1_DIR,MOTOR1_PWM,abs(duty_left) * ( PWM_MAX/ 100) );
   servo_control(g_car_control.servo_duty);//舵机环控制
  }
}

int main(int, char**) 
{
    // 注册清理函数
    atexit(cleanup);
    PID_init(0, 0.01, 0, 0.01);
    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);

    // 创建周期定时器 周期10ms
    timer = new TimerFdWrapper(10, timer_callback);
    timer->start();

    // 创建并运行线程
    std::thread image_thread(imageProcessingThread);
    std::thread pid_thread(pidThread);
    pit_ms_init(10, wheel);

        // 获取六轴数据和编码器数据，10ms发送一次。
        ThreadWrapper thd_0(tcp_send_sensor_thd_entry);
        // 串口接收雷达数据，通过TCP发送到ubuntu中。
        ThreadWrapper thd_1(tcp_send_ladar_thd_entry);
        // TCP接收ROS2的数据，控制车模运动和转向
        ThreadWrapper thd_2(tcp_recv_control_thd_entry);

        while (!exiting)
    {
        system_delay_ms(10);
    }
        image_thread.join();
        pid_thread.join();
        all_cleanup(); //
}