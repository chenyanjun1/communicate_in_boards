//
// Created by supernova on 25-10-12.
//

#ifndef COMMUNICATE_IN_BOARDS_H
#define COMMUNICATE_IN_BOARDS_H
#include "bsp_can.h"
#include "shoot_fun.h"
#include "alg_chassis_calc.h"
#include "robot_config.h"
#ifdef BOARD_UP
#include "robot_app_task.h"
#include "gimbal_fun.h"
#elif defined BOARD_DOWN
#include "IMU_task.h"
extern float IMU_angle[3];

#endif

#pragma pack(1)

#ifdef BOARD_UP
typedef struct {
    float IMU_angle_yaw;        //陀螺仪yaw轴的值 [0:16]
    float IMU_angle_roll;       //陀螺仪roll轴的值 [16:32]
}processed_receive_data;

typedef struct {
    bool robot_state;           //使能状态[0：1]
    ChassisAction chassis_state;//底盘状态[2:4]
    float Vx;                   //前后速度[8:24]
    float Vy;                   //左右速度[24:40]
    float motor_yaw_angle;      //云台偏航角度[40:56]
}processed_send_data;

#elif defined BOARD_DOWN
typedef struct {
    bool robot_state;           //使能状态[0：1]
    ChassisAction chassis_state;//底盘状态[2:4]
    float Vx;                   //前后速度[8:24]
    float Vy;                   //左右速度[24:40]
    float motor_yaw_angle;      //云台偏航角度[40:56]
}processed_receive_data;

typedef struct {
    float IMU_angle_yaw;        //陀螺仪yaw轴的值 [0:16]
    float IMU_angle_roll;       //陀螺仪roll轴的值 [16:32]
}processed_send_data;
#endif
#pragma pack()

typedef struct {
    char* topic_name;
    CanInstance_s *can_instance;
#ifdef BOARD_UP

    processed_receive_data_up receive_data;
    processed_send_data_up send_data;

    GimbalInstance_s *parent_ptr_gimbal;
    ShootInstance_s *parent_ptr_shoot;
    robot_state_t *parent_ptr_state;
#elif defined BOARD_DOWN

    processed_send_data send_data;
    processed_receive_data receive_data;

#endif
}communicateInstance_s;

typedef struct {

    char* topic_name;
    CanInitConfig_s can_config;
#ifdef BOARD_UP
    GimbalInstance_s *parent_ptr_gimbal;
    robot_state_t *parent_ptr_state;
#endif
}communicateInitConfig_s;


communicateInstance_s *Communicate_Register(communicateInitConfig_s *communicate_config);

bool Communicate_Transmit(communicateInstance_s *instance);






#endif //COMMUNICATE_IN_BOARDS_H
