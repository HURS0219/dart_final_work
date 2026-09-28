//
// Created by ROG on 2025/11/17.
//
/* Private includes ----------------------------------------------------------*/
#include "grab.h"

#include "daemon.h"
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
#define PULLEY_GEAR_RATIO 2.0f          // 带轮传动比
#define BEVEL_GEAR_RATIO 1.6667f        // 锥齿轮传动比 5:3
#define MOTOR2006_REDUCTION_RATIO 36.0f // 2006 ecd减速比36
/* Private variables ---------------------------------------------------------*/
static GrabInstance *grab;
static Grab_Ctrl_Cmd_s *grab_ctrl_cmd;
static float total_angle_init_L = 0;
static float total_angle_init_R = 0;
static float total_angle_init_vedio_forward = 0;
static float total_angle_init_vedio_pitch = 0;
float a[5] = {0, 0, 0, 0, 0}; // 测试用电机目标位置
/* Private function prototypes -----------------------------------------------*/
GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config); // 机械臂初始化，返回一个机械臂示例指针
void GrabTask();                                              // 机械臂任务函数
static void GrabCmdTask();                                    // 机械臂控制命令处理函数
static void MotorTask();                                      // 电机任务函数
static void Grab_Position_Calculate(GrabInstance *grab);      // 计算电机目标位置

/* Private user code ---------------------------------------------------------*/
/**
 * @brief 初始化机械臂
 */
GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config)
{
    GrabInstance *grab_instance = (GrabInstance *)zmalloc(sizeof(GrabInstance));
    grab_instance->actuator = (ActuatorInstance *)zmalloc(sizeof(ActuatorInstance));
    grab_instance->arm = (ArmInstance *)zmalloc(sizeof(ArmInstance));
    grab_instance->vedio = (VedioInstance *)zmalloc(sizeof(VedioInstance));

    grab_instance->actuator->grab_djimotor[0] = DJIMotorInit(&Grab_init_config->Grab_motor_config[3]);
    grab_instance->actuator->grab_djimotor[1] = DJIMotorInit(&Grab_init_config->Grab_motor_config[4]);

    while (grab_instance->actuator->grab_djimotor[1]->measure.real_current == 0)
    {
        osDelay(10);
    }
    // 在没有上电的情况下先不发使能帧给dm电机，即不初始化
    grab_instance->actuator->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[5]); // v2
    grab_instance->arm->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[0]);      // v3
    grab_instance->arm->grab_dmmotor[1] = DMMotorInit(&Grab_init_config->Grab_motor_config[1]);      // v4
    grab_instance->arm->grab_dmmotor[2] = DMMotorInit(&Grab_init_config->Grab_motor_config[2]);      // v4

    //
    // grab_instance->vedio->grab_djimotor[0] =
    // DJIMotorInit(&Grab_init_config->Grab_motor_config[6]);
    // grab_instance->vedio->grab_djimotor[1] =
    // DJIMotorInit(&Grab_init_config->Grab_motor_config[7]);

    // 先赋值grab指针，再访问grab_instance中的成员
    grab = grab_instance;
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;

    // 初始化电机初始角度，必须要发生在grab的赋值之后
    osDelay(10);
    total_angle_init_L = grab->actuator->grab_djimotor[1]->measure.total_angle;
    total_angle_init_R = grab->actuator->grab_djimotor[0]->measure.total_angle;
    total_angle_init_vedio_forward = grab->vedio->grab_djimotor[0]->measure.total_angle;
    total_angle_init_vedio_pitch = grab->vedio->grab_djimotor[1]->measure.total_angle;
    if (Grab_init_config->Grab_cali_mode == GRAB_CALI_MODE)
    {
        DMMotorCaliEncoder(grab->arm->grab_dmmotor[0]);
        DMMotorCaliEncoder(grab->arm->grab_dmmotor[1]);
        DMMotorCaliEncoder(grab->arm->grab_dmmotor[2]);
        DMMotorCaliEncoder(grab->actuator->grab_dmmotor[0]);
    }
    return grab_instance;
}

/**
 * @brief 机械臂任务函数
 */
void GrabTask()
{
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;
    GrabCmdTask();
    Grab_Position_Calculate(grab);
    MotorTask();
}
static void GrabCmdTask()
{
    grab->arm->base_joint = grab_ctrl_cmd->base_joint;
    grab->arm->elbow_roll = grab_ctrl_cmd->elbow_roll;
    grab->arm->elbow_pitch = grab_ctrl_cmd->elbow_pitch;
    grab->actuator->wrist_pitch = grab_ctrl_cmd->wrist_pitch;
    grab->actuator->wrist_roll = grab_ctrl_cmd->wrist_roll;
    grab->actuator->torque = grab_ctrl_cmd->torque;
    // grab->vedio->vedio_forward= grab_ctrl_cmd->vedio_forward;
    // grab->vedio->vedio_pitch= grab_ctrl_cmd->vedio_pitch;
}
static void MotorTask()
{
    if (grab_ctrl_cmd->grab_mode == GRAB_POWER_OFF)
    {
        DMMotorStop(grab->arm->grab_dmmotor[0]);
        DMMotorStop(grab->arm->grab_dmmotor[1]);
        DMMotorStop(grab->arm->grab_dmmotor[2]);

        DJIMotorStop(grab->actuator->grab_djimotor[0]);
        DJIMotorStop(grab->actuator->grab_djimotor[1]);
        DMMotorStop(grab->actuator->grab_dmmotor[0]);

        // DJIMotorStop(grab->vedio->grab_djimotor[0]);
        // DJIMotorStop(grab->vedio->grab_djimotor[1]);
    }
    else
    {
        // 循环处理所有DMMotor，只对在线的电机进行使能和PID计算
        for (int i = 0; i < 3; i++)
        {
            if (DaemonIsOnline(grab->arm->grab_dmmotor[i]->daemon))
            {
                DMMotorEnable(grab->arm->grab_dmmotor[i]);
                switch (i)
                {
                case 0:
                    DMMotorSetPIDRef(grab->arm->grab_dmmotor[i], grab->arm->base_joint * DEGREE_2_RAD);
                    break;
                case 1:
                    DMMotorSetPIDRef(grab->arm->grab_dmmotor[i], grab->arm->elbow_roll * DEGREE_2_RAD);
                    break;
                case 2:
                    DMMotorSetPIDRef(grab->arm->grab_dmmotor[i], grab->arm->elbow_pitch * DEGREE_2_RAD);
                    break;
                }
            }
        }

        // 循环处理所有DJIMotor（actuator部分）
        for (int i = 0; i < 2; i++)
        {
            if (DaemonIsOnline(grab->actuator->grab_djimotor[i]->daemon))
            {
                DJIMotorEnable(grab->actuator->grab_djimotor[i]);
                switch (i)
                {
                case 0:
                    DJIMotorSetPIDRef(grab->actuator->grab_djimotor[i], grab->actuator->R_target);
                    break;
                case 1:
                    DJIMotorSetPIDRef(grab->actuator->grab_djimotor[i], grab->actuator->L_target);
                    break;
                }
            }
        }

        // 处理actuator的DMMotor
        if (DaemonIsOnline(grab->actuator->grab_dmmotor[0]->daemon))
        {
            DMMotorEnable(grab->actuator->grab_dmmotor[0]);
            DMMotorSetRef(grab->actuator->grab_dmmotor[0], grab->actuator->T_target);
        }

        // // 循环处理所有DJIMotor（vedio部分）
        // for (int i = 0; i < 2; i++) {
        //     if (DaemonIsOnline(grab->vedio->grab_djimotor[i]->daemon)) {
        //         DJIMotorEnable(grab->vedio->grab_djimotor[i]);
        //         switch (i) {
        //             case 0:
        //                 DJIMotorSetPIDRef(grab->vedio->grab_djimotor[i], grab->vedio->F_target);
        //                 break;
        //             case 1:
        //                 DJIMotorSetPIDRef(grab->vedio->grab_djimotor[i], grab->vedio->P_target);
        //                 break;
        //         }
        //     }
        // }
    }
}

/**
 * @brief 根据pitch和roll角度解算出电机的转动角度
 * @param arm 机械臂结构体指针
 */
static void Grab_Position_Calculate(GrabInstance *grab)
{
    /**
     * L = -pitch + roll
     * R = +pitch + roll
     */

    grab->actuator->R_target =
        total_angle_init_R + (grab->actuator->wrist_pitch + grab->actuator->wrist_roll * BEVEL_GEAR_RATIO) *
                                 MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;
    grab->actuator->L_target =
        total_angle_init_L + (-grab->actuator->wrist_pitch + grab->actuator->wrist_roll * BEVEL_GEAR_RATIO) *
                                 MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;
    // grab->vedio->F_target = total_angle_init_vedio_forward + grab->vedio->vedio_forward * MOTOR2006_REDUCTION_RATIO;
    // grab->vedio->P_target = total_angle_init_vedio_pitch + grab->vedio->vedio_pitch;
    grab->actuator->T_target = grab->actuator->torque;
}
