#ifndef CONTROL_SERVO_H
#define CONTROL_SERVO_H
#include "all.h"
namespace pid_constants
{
    inline constexpr double MIN_TIME_INTERVAL = 1e-9; // 1纳秒
}

class PIDController_pro
{
public:
    // 显式构造函数，确保所有成员正确初始化
    PIDController_pro(double Kp, double Ki, double Kd, double initial_setpoint = 0.0)
        : Kp(Kp), Ki(Ki), Kd(Kd),
          setpoint_(initial_setpoint),
          prev_error_(0.0),
          integral_(0.0),
          prev_time_(std::chrono::steady_clock::now()) {}

    // 优化后的带dt计算方法
    double compute(double measured_value, double dt)
    {
        // 处理异常时间间隔
        dt = std::abs(dt); // 确保时间间隔为正
        const double safe_dt = std::max(dt, pid_constants::MIN_TIME_INTERVAL);

        const double error = setpoint_ - measured_value;

        // 条件积分计算（当Ki非零时累积）
        if (Ki != 0.0)
        {
            integral_ += error * safe_dt; // 使用safe_dt保证数值稳定性
        }

        // 条件导数计算（当Kd非零时计算）
        double derivative = 0.0;
        if (Kd != 0.0 && safe_dt > pid_constants::MIN_TIME_INTERVAL)
        {
            derivative = (error - prev_error_) / safe_dt;
        }

        const double output = Kp * error + Ki * integral_ + Kd * derivative;

        // 状态更新
        prev_error_ = error;
        return output;
    }

    // 优化后的时间自动计算方法
    double compute(double measured_value)
    {
        const auto current_time = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(current_time - prev_time_).count();
        std::cout << dt << std::endl;
        prev_time_ = current_time;
        return compute(measured_value, dt);
    }

    // 新增setter方法（根据需求可选）
    void set_setpoint(double new_setpoint) { setpoint_ = new_setpoint; }
    void reset_integral() { integral_ = 0.0; }

private:
    // 高频变量保持紧凑布局
    double setpoint_;   // 每次计算都访问
    double prev_error_; // 每次计算都访问
    double integral_;   // 每次计算都访问

    // 低频常量分组
    const double Kp, Ki, Kd; // 初始化后只读

    // 时间变量单独分组
    std::chrono::steady_clock::time_point prev_time_;
};

double weighted_average(
    double v0, double v1, double v2,
    double w0, double w1, double w2) noexcept;

struct PIDParams
{
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
};
bool readPIDParams(const std::string &filename, PIDParams &params);
#endif // CONTROL_SERVO_H
