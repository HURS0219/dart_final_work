#include "gantry.h"
#include "cmsis_os.h"
#include "main.h"
#include "math.h"
#include "remote_control.h"
#include "stdlib.h"
#include "string.h"

// 使用新的实例结构体
static GantryInstance *gantry_instance = NULL;

// 定义全局变量，解决链接错误
RC_ctrl_t controller;  // 自定义控制器数据
GantryInstance gantry; // 龙门架实例

// 初始化位置
static float init_lift_ecd[2], init_stretch_ecd[2], init_sidesway_ecd; // 电机刚上电时的位置
static float last_gantry_x;

extern RC_ctrl_t controller;                  // 自定义控制器数据
extern DJI_Motor_Measure_s camera_motor_lift; // 相机平台的抬升电机

static void Gantry_Run(GantryInstance *gantry);
static void Gantry_Position_Calculate(GantryInstance *gantry);
static void Gantry_Shut_Down(GantryInstance *gantry);
void GantryTask(void);
/**
 * @brief 龙门架任务函数
 */
void StartGantryTask(void const *argument)
{
    // 等待实例初始化完成
    while (gantry_instance == NULL)
    {
        osDelay(10);
    }

    uint32_t wait_time = xTaskGetTickCount();
    for (;;)
    {
        Gantry_Run(gantry_instance);
        osDelayUntil(&wait_time, 1);
    }
}

static void Gantry_Run(GantryInstance *gantry)
{
    Gantry_Position_Calculate(gantry); // 根据位置矢量解算出电机转动的圈数
    Gantry_Shut_Down(gantry);          // 使用pid计算结果控制电机转动
    // 【新增】调用电机模块的发送函数，将电流值发送给电机
    DJIMotorTask();
}

/**
 * @brief 龙门架初始化
 * @param init_config 初始化配置
 * @return GantryInstance* 龙门架实例指针
 */
GantryInstance *GantryInit(Gantry_Init_Config_s *init_config)
{
    if (gantry_instance != NULL)
    {
        return gantry_instance; // 已经初始化过
    }

    gantry_instance = (GantryInstance *)malloc(sizeof(GantryInstance));
    if (gantry_instance == NULL)
    {
        return NULL;
    }
    memset(gantry_instance, 0, sizeof(GantryInstance));

    // 保存参数配置
    gantry_instance->Gantry_param = init_config->Gantry_param;
    // 初始化控制命令
    gantry_instance->Gantry_ctrl_cmd.Gantry_mode = GANTRY_MODE_POWER_OFF;

    // 初始化电机
    for (int i = 0; i < 2; i++)
    {
        // 抬升电机
        gantry_instance->lift_motor[i].motor = DJIMotorInit(&init_config->lift_motor_config[i]);
        // 前伸电机
        // gantry_instance->stretch_motor[i].motor = DJIMotorInit(&init_config->stretch_motor_config[i]);
    }
    // 横移电机
    // gantry_instance->sidesway_motor.motor = DJIMotorInit(&init_config->sidesway_motor_config);

    // 等电机数据出现代表电机已经正常工作
    // for (int i = 0; i < 2; i++)
    // {
    //     // 等待抬升电机数据
    //     int timeout = 0;
    //     while (gantry_instance->lift_motor[i].motor == NULL ||
    //            gantry_instance->lift_motor[i].motor->measure.ecd == 0 )
    //     {            osDelay(10);
    //         timeout++;
    //     }
    //
    //     // 等待前伸电机数据
    //     timeout = 0;
    //     while (gantry_instance->stretch_motor[i].motor == NULL ||
    //            gantry_instance->stretch_motor[i].motor->measure.ecd == 0 )
    //     {            osDelay(10);
    //         timeout++;
    //     }
    // }
    osDelay(100);

    // 设置目标角度为当前角度，防止上电时失控
    for (int i = 0; i < 2; i++)
    {
        if (gantry_instance->lift_motor[i].motor != NULL)
        {
            init_lift_ecd[i] = gantry_instance->lift_motor[i].motor->measure.total_angle;
        }
        // if (gantry_instance->stretch_motor[i].motor != NULL) {
        //     init_stretch_ecd[i] = gantry_instance->stretch_motor[i].motor->measure.total_angle;
        // }
    }
    // if (gantry_instance->sidesway_motor.motor != NULL) {
    //     init_sidesway_ecd = gantry_instance->sidesway_motor.motor->measure.total_angle;
    // }

    // 切换到遥控器控制模式
    gantry_instance->Gantry_ctrl_cmd.Gantry_mode = GANTRY_MODE_CONTROL_REMOTE;

    return gantry_instance;
}

/**
 * @brief 使用pid计算结果控制电机转动
 * @param gantry 龙门架实例指针
 */
static void Gantry_Shut_Down(GantryInstance *gantry)
{
    uint8_t stop = (gantry->Gantry_ctrl_cmd.Gantry_mode == GANTRY_MODE_POWER_OFF);
    // 抬升电机
    for (int i = 0; i < 2; i++)
    {
        if (gantry->lift_motor[i].motor)
        {
            if (stop)
                DJIMotorStop(gantry->lift_motor[i].motor);
            else
                DJIMotorEnable(gantry->lift_motor[i].motor);
        }
    }
    // 前伸电机
    // for (int i = 0; i < 2; i++) {
    //     if (gantry->stretch_motor[i].motor) {
    //         if (stop)
    //             DJIMotorStop(gantry->stretch_motor[i].motor);
    //         else
    //             DJIMotorEnable(gantry->stretch_motor[i].motor);
    //     }
    // }
    // 横移电机
    // if (gantry->sidesway_motor.motor) {
    //     if (stop)
    //         DJIMotorStop(gantry->sidesway_motor.motor);
    //     else
    //         DJIMotorEnable(gantry->sidesway_motor.motor);
    // }
}

/**
 * @brief 根据位置矢量解算出电机转动的圈数
 * @param gantry 龙门架结构体指针
 */
static void Gantry_Position_Calculate(GantryInstance *gantry)
{
    // 使用局部变量简化访问
    const float ratio = gantry->Gantry_param.position_ecd_ratio;
    const Gantry_Ctrl_Cmd_s *cmd = &gantry->Gantry_ctrl_cmd;

    // 计算 Z 轴目标位置对应的编码器变化量
    float delta_z_ecd = cmd->z * ratio;
    if (gantry->lift_motor[0].motor)
    {
        DJIMotorSetPIDRef(gantry->lift_motor[0].motor, init_lift_ecd[0] - delta_z_ecd);
    }
    // if (gantry->lift_motor[1].motor) {
    //     // 修改lift1电机的方向，使其与lift0电机一致
    //     DJIMotorSetPIDRef(gantry->lift_motor[1].motor, init_lift_ecd[1] + delta_z_ecd);
    // }

    // 计算 Y 轴目标位置对应的编码器变化量
    // float delta_y_ecd = cmd->y * ratio;
    // if (gantry->stretch_motor[0].motor) {
    //     DJIMotorSetPIDRef(gantry->stretch_motor[0].motor, init_stretch_ecd[0] - delta_y_ecd);
    // }
    // if (gantry->stretch_motor[1].motor) {
    //     DJIMotorSetPIDRef(gantry->stretch_motor[1].motor, init_stretch_ecd[1] + delta_y_ecd);
    // }

    // 计算 X 轴目标位置对应的编码器变化量
    // float delta_x_ecd = cmd->x * ratio;
    // if (gantry->sidesway_motor.motor) {
    //     DJIMotorSetPIDRef(gantry->sidesway_motor.motor, init_sidesway_ecd - delta_x_ecd);
    // }
}

void GantryTask()
{
    // 调用龙门架的控制逻辑入口
    if (gantry_instance != NULL)
    {
        Gantry_Run(gantry_instance);
    }
}