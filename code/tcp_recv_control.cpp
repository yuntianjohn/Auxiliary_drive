
#include "zf_common_headfile.h"
#include "tcp_recv_control.h"
#include "radar_data_protocol.h"
#include "get_config.h"
#include "motor_control.h"


car_control_typedef tcp_car_control;
car_control_typedef g_car_control;

static int tcp_client_recv_socket;

// static pthread_mutex_t tcp_recv_control_mutex = PTHREAD_MUTEX_INITIALIZER;

// car_control_typedef get_recv_control()
// {
//     car_control_typedef temp_car_control;

//     // pthread_mutex_lock(&tcp_recv_control_mutex);
//     temp_car_control.speed = car_control.speed;
//     temp_car_control.servo = car_control.servo;
//     // pthread_mutex_unlock(&tcp_recv_control_mutex);

//     return temp_car_control;
// }
void close_tcp_recv_control(void)
{
    close(tcp_client_recv_socket);
}

void tcp_recv_control_thd_entry(void)
{
    // 尝试设置新的高优先级,值越大优先级越高（1-99）
	struct sched_param param;
	param.sched_priority = TCP_RECV_CONTROL_THD_PRO;
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    printf("tcp_init recv ing... \r\n");

    while(1)
    {
        do
        {
            // 初始化 tcp server
            tcp_client_recv_socket = tcp_init(SERVER_IP, RECV_CONTROL_PORT);
            if(tcp_client_recv_socket < 0)
            {
                system_delay_ms(1000);
                printf("tcp_client_recv_socket error \r\n");
            }
            else
            {
                printf("tcp_client_recv_socket OK \r\n");
            }
        } while (tcp_client_recv_socket <= 0);

        while(1)
        {
            // TCP周期发送传感器数据
            int str_len = tcp_client_read_data(tcp_client_recv_socket, (uint8_t *)&tcp_car_control, sizeof(car_control_typedef));
            
            if(str_len <= 0)
            {
                tcp_car_control.left_speed = 0;
                tcp_car_control.right_speed = 0;
                tcp_car_control.servo_duty = 0;
                break;
            }
            else if(str_len > 0)
            {
                printf("left_speed = %d\r\n", tcp_car_control.left_speed);
                printf("right_speed = %d\r\n", tcp_car_control.right_speed);
                printf("servo_duty = %d\r\n", tcp_car_control.servo_duty);
            }
    
            // pthread_mutex_lock(&tcp_recv_control_mutex);
            g_car_control.left_speed = tcp_car_control.left_speed;
            g_car_control.right_speed = tcp_car_control.right_speed;
            g_car_control.servo_duty = tcp_car_control.servo_duty;
            // pthread_mutex_unlock(&tcp_recv_control_mutex);

            // system_delay_ms(10);
        }
    }
}

