#ifndef PID_H
#define PID_H

#include "zf_common_typedef.h"
//#include "all_head.h"
/*定义增量式、位置式内参数*/
typedef struct{
    float Kp;
    float Ki;
    float Kd;

    float p_out;	         //分别输出
    float i_out;
    float d_out;

    float Err;             //本次误差
    float Last_Err;        //上次误差
    float Previous_Err;    //上上次误差

    float Output;         //最终输出
    float Target;	      //目标值
    float Measure;        //测量值
}Incremental_PID; //速度


typedef struct {
    float Kp;         // 比例系数
    float Ki;         // 积分系数
    float Kd;         // 微分系数
    float Err;        // 当前误差
    float Last_Err;   // 上一次的误差
    float Integral;   // 误差的积分
    float Output;     // 输出控制量
} Positional_PID; //方向


extern Incremental_PID Pid_left;
extern Incremental_PID Pid_right;//声明该结构体可在其他处出现
extern Positional_PID Pid_place;//方向环


/*赋值PID参数*/
void PID_init(float kp1,float ki1,float kp2,float ki2);
/*增量式、位置式*/
float Incremental_PID_Calculate(Incremental_PID *pid ,float expect ,float measure);
float Positional_PID_Calculate(Positional_PID *pid, float SeeingErr);
/*其他函数*/
void conversion_speed(void);//转换速度并计算期望
float Limit(float value,float Amplitude);//轮子限幅函数

// bool readCntFromFile(const std::string& file_path, int& cnt);
#endif
