#ifndef INFANTRY_CTRL_H
#define INFANTRY_CTRL_H

#include "robot.h"
#include "user_lib.h"

#define robot_lost_control (abs(robot->chassis->imu->Pitch) > 13.0f) 

#define has_non_zero_data(data) \
    (data != NULL) && \
    (data->gimbal_receive.yaw != 0 || data->gimbal_receive.pitch != 0 || data->shoot_receive.fire_flag != 0)
/**
 * @brief Control logic for Remote Controller mode.
 * @param robot Pointer to the RobotInstance.
 */
void JoyStickCtrl(RobotInstance* robot);

/**
 * @brief Control logic for Mouse and Keyboard mode.
 * @param robot Pointer to the RobotInstance.
 */
void MouseKeyCtrl(RobotInstance* robot);

/**
 * @brief Emergency stop handler.
 * @param robot Pointer to the RobotInstance.
 */
void EmergencyHandler(RobotInstance* robot);

#endif // INFANTRY_CTRL_H
