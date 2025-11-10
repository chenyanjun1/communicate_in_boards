//
// Created by supernova on 25-10-12.
//


#include "communicate_in_boards.h"


static int float_to_uint(const float x_float, const float x_min, const float x_max, const int bits){
    const float span = x_max - x_min;
    const float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(const int x_int, const float x_min, const float x_max, const int bits){
    const float span = x_max - x_min;
    const float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

#if defined BOARD_UP && defined BOARD_DOWN
#error "Only one board can be defined pls"
#endif


#ifdef BOARD_UP
static bool communicate_processing(communicateInstance_s *instance){
    if(instance == NULL){
        return  false;
    }
    instance->send_data.robot_state = instance->parent_ptr_state->robot_control_mode;
    instance->send_data.motor_yaw_angle = instance->parent_ptr_gimbal->yaw_motor->message.out_position;
    instance->send_data.chassis_state =instance->parent_ptr_state->chassis_state;
    instance->send_data.Vx = instance->parent_ptr_state->Chassis_velocity.Vx;
    instance->send_data.Vy = instance->parent_ptr_state->Chassis_velocity.Vy;
    const uint8_t chassis_state_tmp = (uint8_t)instance->send_data.chassis_state;
    const uint16_t Vx_tmp = float_to_uint(instance->send_data.Vx, -5, 5 , 16);
    const uint16_t Vy_tmp = float_to_uint(instance->send_data.Vy, -5, 5 , 16);
    const uint16_t motor_yaw_angle_tmp = float_to_uint(instance->send_data.motor_yaw_angle,3.141593f, -3.141593f , 16);
    const uint8_t robot_state_tmp = instance->send_data.robot_state;
    instance->can_instance->tx_buff[0] = ((chassis_state_tmp << 1) & 0x03) | (robot_state_tmp & 0x01);
    instance->can_instance->tx_buff[1] = (Vx_tmp >> 8);
    instance->can_instance->tx_buff[2] = Vx_tmp;
    instance->can_instance->tx_buff[3] = (Vy_tmp >> 8);
    instance->can_instance->tx_buff[4] = Vy_tmp;
    instance->can_instance->tx_buff[5] = (motor_yaw_angle_tmp >> 8);
    instance->can_instance->tx_buff[6] = motor_yaw_angle_tmp;
    return true;
}
static void communicate_decode(CanInstance_s *can_instance) {
    if (can_instance == NULL) {
        return;
    }
    const uint8_t *rx_buff = can_instance->rx_buff;
    communicateInstance_s *instance = can_instance->parent_ptr;
    if (instance == NULL) {
        return;
    }
    instance->receive_data.IMU_angle_yaw = uint_to_float(rx_buff[0] << 8 | rx_buff[1], -180.0f, 180.0f, 16);
    instance->receive_data.IMU_angle_roll = uint_to_float(rx_buff[2] << 8 | rx_buff[3], -180.0f, 180.0f, 16);
return ;
}
#elif defined BOARD_DOWN
static bool communicate_processing(communicateInstance_s *instance) {
    if (instance == NULL) {
        return false;
    }
    instance->send_data.IMU_angle_yaw = IMU_angle[0];
    instance->send_data.IMU_angle_roll = IMU_angle[2];
    const uint16_t IMU_angle_yaw_tmp = float_to_uint(instance->send_data.IMU_angle_yaw, -180.0f, 180.0f,16);
    const uint16_t IMU_angle_pitch_tmp = float_to_uint(instance->send_data.IMU_angle_roll, -180.0f, 180.0f,16);
    instance->can_instance->tx_buff[0] = (IMU_angle_yaw_tmp >> 8);
    instance->can_instance->tx_buff[1] = IMU_angle_yaw_tmp;
    instance->can_instance->tx_buff[2] = (IMU_angle_pitch_tmp >> 8);
    instance->can_instance->tx_buff[3] = IMU_angle_pitch_tmp;
    instance->can_instance->tx_buff[4] = 0;
    instance->can_instance->tx_buff[5] = 0;
    instance->can_instance->tx_buff[6] = 0xff;
    instance->can_instance->tx_buff[7] = 0xff;
return true;
}

static void communicate_decode(CanInstance_s *can_instance){
    if (can_instance == NULL){
        return ;
    }
    const uint8_t *rx_buff = can_instance->rx_buff;
    communicateInstance_s *instance = can_instance->parent_ptr;
    if(instance == NULL){
        return ;
}
    instance->receive_data.robot_state = rx_buff[0] & 0x01;
    instance->receive_data.chassis_state = (ChassisAction)((rx_buff[0] << 1) & 0x0f);
    instance->receive_data.Vx = uint_to_float(rx_buff[1] << 8 | rx_buff[2], -5, 5, 16);
    instance->receive_data.Vy = uint_to_float(rx_buff[3] << 8 | rx_buff[4], -5, 5, 16);
    instance->receive_data.motor_yaw_angle = uint_to_float(rx_buff[5] << 8 | rx_buff[6], 3.141593f, -3.141593f, 16);

    return;
}
#endif



communicateInstance_s *Communicate_Register(communicateInitConfig_s *communicate_config){
    if(communicate_config == NULL){
        return NULL;
    }
    communicateInstance_s* instance = (communicateInstance_s*)user_malloc(sizeof(communicateInstance_s));
    if(instance == NULL){
        user_free(instance);
        return NULL;
    }

    instance->topic_name = communicate_config->topic_name;
    communicate_config->can_config.topic_name = communicate_config->topic_name;


    communicate_config->can_config.can_module_callback = communicate_decode;

    communicate_config->can_config.parent_ptr = instance;

    instance->can_instance = Can_Register(&communicate_config->can_config);

    if(instance->can_instance == NULL){
        user_free(instance);
        return NULL;
    }
#ifdef BOARD_UP
    instance->parent_ptr_gimbal = communicate_config->parent_ptr_gimbal;
    instance->parent_ptr_state = communicate_config->parent_ptr_state;
#endif

    return instance;
}

bool Communicate_Transmit(communicateInstance_s *instance){
    if(instance == NULL){
        return false;
    }
    communicate_processing(instance);
    return Can_Transmit(instance->can_instance);
}

