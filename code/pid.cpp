#include "pid.h"
#include <iostream>
Incremental_PID  Pid_left ;
Incremental_PID  Pid_right;
Positional_PID   Pid_place;

/**PID参数初始化**/

void PID_init(float kp1,float ki1,float kp2,float ki2)
{
    //左轮
    Pid_left.Kp = kp1;
    Pid_left.Ki = ki1;//PI即可
    Pid_left.Kd = 0.0;
    //右轮

    Pid_right.Kp = kp2;
    Pid_right.Ki = ki2;
    Pid_right.Kd = 0;
    //方向
    Pid_place.Kp = 0;
    Pid_place.Ki = 0;
    Pid_place.Kd = 0;

}

/**增量式算法_速度闭环**/
/*增量式:Pwm+=(Kp*(Err-Err1)+Ki*Err+Kd*(Err-2*Err1+Err2))*/

float Incremental_PID_Calculate(Incremental_PID *pid ,float expect ,float measure )
{
    pid->Err = expect - measure;
    pid->p_out = pid->Kp * (pid->Err - pid->Last_Err);
    pid->i_out = pid->Ki * pid->Err;
    pid->d_out = pid->Kd * (pid->Err - 2*pid->Last_Err + pid->Previous_Err);
    //累加

    pid->Output += (pid->p_out + pid->i_out + pid->d_out);
    //替换
    pid->Previous_Err = pid->Last_Err;
    pid->Last_Err =  pid->Err;
    //限幅


    pid->Output = Limit(pid->Output ,20);
	//std::cout<<pid->Output <<std::endl;
    return pid->Output;
}
float Positional_PID_Calculate(Positional_PID *pid, float SeeingErr)
{

    pid->Err = SeeingErr;// 计算当前误差

    float p_out = pid->Kp * pid->Err; // 计算比例项输出

    pid->Integral += pid->Err;
    float i_out = pid->Ki * pid->Integral;// 计算积分项输出

    float d_out = pid->Kd * (pid->Err - pid->Last_Err);    // 计算微分项输出

    pid->Output = (float)(p_out + i_out + d_out);// 计算总输出

    pid->Last_Err = pid->Err;// 更新上一次的误差

    pid->Output = Limit(pid->Output, 7700);  // 限幅

    return pid->Output;
}
float Limit(float value,float Amplitude)
{
    float temp;
    if(value>Amplitude) temp = Amplitude;
    else if(value<-Amplitude) temp = -Amplitude;
    else temp = value;
    return temp;
}

// bool readCntFromFile(const std::string& file_path, int& cnt) {
//     std::ifstream file(file_path);
//     if (!file.is_open()) {
//         std::cerr << "Error: Cannot open file " << file_path << std::endl;
//         return false;
//     }

//     std::string line;
//     while (std::getline(file, line)) {
//         line = line.substr(0, line.find('#'));  // 移除行内注释
//         line.erase(0, line.find_first_not_of(" \t"));  // 去除行首空格
//         if (line.empty()) continue;

//         size_t delimiter_pos = line.find('=');
//         if (delimiter_pos == std::string::npos) continue;

//         std::string key = line.substr(0, delimiter_pos);
//         std::string value = line.substr(delimiter_pos + 1);

//         // 去除键和值的首尾空格
//         key.erase(key.find_last_not_of(" \t") + 1);
//         value.erase(0, value.find_first_not_of(" \t"));
//         value.erase(value.find_last_not_of(" \t") + 1);

//         if (key == "cnt") {
//             try {
//                 cnt = std::stoi(value);
//                 return true;  // 成功读取后直接返回
//             } catch (...) {
//                 std::cerr << "Error: Invalid cnt value in file" << std::endl;
//                 return false;
//             }
//         }
//     }

//     std::cerr << "Error: cnt not found in file" << std::endl;
//     return false;
// }