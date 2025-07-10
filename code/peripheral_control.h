
#ifndef PERIPHERAL_CONTROL_H
#define PERIPHERAL_CONTROL_H


#include "get_config.h"
#include "user_pid.h"
#include "motor_control.h"
#include "pid.h"
#include "all.h"
#include "get_encoder.h"
#include "get_radar.h"
#include "get_imu.h"
#include "tcp_send_ladar.h"
#include "tcp_send_sensor.h"
#include "tcp_recv_control.h"


#define PWM_MAX 9999
#define SERVO_MOTOR_FREQ (servo_pwm_info.freq)
#define PWM_DUTY_MAX (servo_pwm_info.duty_max)
#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0))
#define SERVO_MOTOR_L_MAX (75)
#define SERVO_MOTOR_R_MAX (105)


float sum_distance(float encoder_speed);
void set_motor_direction_pwm(uint16_t motor_pin, uint16_t dir_pin, int16_t duty);
void wheel_control(void);
void pidThread(void);
void imageProcessingThread(void);
void sigint_handler(void);
void all_cleanup(void);

#endif
