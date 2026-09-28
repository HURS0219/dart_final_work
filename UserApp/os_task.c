/**
 ******************************************************************************
 * @file    os_task.c
 * @author  Enhao Zhang
 * @date    2025/10/9
 * @brief   None
 ******************************************************************************
 * @attention
 * None
 *
 ******************************************************************************
 */
#include "os_task.h"
// freertos
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
// tasks
#include "buzzer.h"
#include "daemon.h"
#include "motor_task.h"
#include "robot.h"
#include "ui.h"
// module
#include "dmmotor.h"
// bsp
#include "bsp_init.h"

osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId robotTaskHandle;
osThreadId uiTaskHandle;

/**
 * @brief 初始化RTOS任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit() {
  // 关闭中断,防止在初始化过程中发生中断
  // 请不要在初始化过程中使用中断和延时函数！
  // 若必须,则只允许使用DWT_Delay()
  __disable_irq();

  BSPInit();

  // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
  // 后续修改为读取传感器数据准备好的中断处理
  osThreadDef(motortask, StartMOTORTASK, osPriorityBelowNormal, 0, 768);
  motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

  osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
  daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

  osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
  robotTaskHandle = osThreadCreate(osThread(robottask), NULL);
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  osThreadDef(uitask, StartUITASK, osPriorityLow, 0, 512);
  uiTaskHandle = osThreadCreate(osThread(uitask), NULL);
#endif

  // 初始化完成,开启中断
  __enable_irq();
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument) {
  static float motor_dt;
  static float motor_start;
  LOGINFO("[freeRTOS] MOTOR Task Start");
  for (;;) {
    motor_start = DWT_GetTimeline_ms();
    MotorControlTask();
    motor_dt = DWT_GetTimeline_ms() - motor_start;
    if (motor_dt > 2) LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
    osDelay(2);
  }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument) {
  static float daemon_dt;
  static float daemon_start;
  BuzzerInit();
  LOGINFO("[freeRTOS] Daemon Task Start");
  for (;;) {
    // 100Hz
    daemon_start = DWT_GetTimeline_ms();
    DaemonTask();
    BuzzerTask();
    daemon_dt = DWT_GetTimeline_ms() - daemon_start;
    if (daemon_dt > 10) LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);
    osDelay(10);
  }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument) {
  static float robot_dt;
  static float robot_start;
  RobotInit();
  DMMotorTaskInit();
  LOGINFO("[freeRTOS] ROBOT core Task Start");
  // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
  for (;;) {
    robot_start = DWT_GetTimeline_ms();
    RobotTask();
    robot_dt = DWT_GetTimeline_ms() - robot_start;
    if (robot_dt > 1) LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%f]", &robot_dt);
    osDelay(1);
  }
}

__attribute__((noreturn)) void StartUITASK(void const *argument) {
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  RobotInstance* ui_robot = NULL;

  LOGINFO("[freeRTOS] UI Task Start");
  osDelay(100);

  for (;;) {
    ui_robot = RobotGetInstance();
    if (ui_robot != NULL && ui_robot->referee_data != NULL) {
      break;
    }
    osDelay(10);
  }

  MyUIInit(ui_robot);

  for (;;) {
    ui_robot = RobotGetInstance();
    if (ui_robot != NULL && ui_robot->referee_data != NULL) {
      UITask(ui_robot);
    }
    osDelay(30);
  }
#else
  for (;;) {
    osDelay(1000);
  }
#endif
}
