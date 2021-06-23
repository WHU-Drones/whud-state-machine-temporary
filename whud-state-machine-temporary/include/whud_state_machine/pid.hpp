/**
 * @file pid.hpp
 * @author LiuZhihao (liuzhihao@whu.edu.cn)
 * @brief An increment PID controller
 * @version 1.0
 * @date 2021-06-22
 * 
 * @copyright Copyright (c) 2021 WHU-Drones
 * 
 */
#ifndef _PID_H_
#define _PID_H_
typedef struct _pid{
float SetSpeed;
float ActualSpeed;
float err;
float err_next;
float err_last;
float Kp, Ki, Kd;
}Pid;
 
 
class Pid_control
{
public:
 
/**
 * @brief Initialize PID controller with given Kp, Ki, Kd 
 * 
 * @param speed The ideal speed we want to achieve.
 */
void PID_init(float speed, float Kp=0, float Ki=0, float Kd=0){
    pid.SetSpeed = 0.0;
    pid.ActualSpeed = 0.0;
    pid.err = 0.0;
    pid.err_last = 0.0;
    pid.err_next = 0.0;
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
}

/**
 * @brief PID control
 * 
 * @param speed_now Speed of current state.
 * 
 * @return float Output of PID, which is used to control robot.
 */
float PID_realize(float speed_now){
    pid.err = pid.SetSpeed - speed_now;
    float incrementSpeed = pid.Kp*(pid.err - pid.err_next) + pid.Ki*pid.err + pid.Kd*(pid.err - 2 * pid.err_next + pid.err_last);
    pid.ActualSpeed += incrementSpeed;
    pid.err_last = pid.err_next;
    pid.err_next = pid.err;
    return pid.ActualSpeed;
}
 
private:
Pid pid;
};
#endif
