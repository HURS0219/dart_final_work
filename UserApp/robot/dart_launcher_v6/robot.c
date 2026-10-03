/*
 * robot.c — dart_launcher_v6 入口 / 编排 (组件容器风格)
 * =============================================================================
 * 参考 UserApp/robot/infantry_six_wheel_example2 与 hero_mecanum_example1:
 *   RobotInit: 创建各组件 (motor/servo/vision) + link + ctrl + store;
 *   RobotTask: 依次调用各组件 Task, 装配遥测快照交给 link。
 * 不直接调用任何 DJI/舵机接口(那是组件内部的事)。
 * =============================================================================
 */
#include "robot.h"

#include <string.h>

#include "bsp_dwt.h"
#include "ctrl.h"
#include "link.h"
#include "main.h"
#include "robot_config.h"
#include "robot_def.h"
#include "store.h"
#include "user_lib.h"

RobotInstance* robot = NULL;

/* 装配 F 帧遥测(数值单位与 v5_HIK 一致: 角度/参数 ×100, 舵机 ×10) */
static void BuildTelemetry(Launcher_Telemetry_s* t) {
  memset(t, 0, sizeof(*t));
  t->n = LM_COUNT;
  for (int i = 0; i < LM_COUNT; i++) {
    MotorAxis* a = &robot->motor->axis[i];
    DJIMotorInstance* m = a->inst;
    float ratio = (a->ratio > 0.01f) ? a->ratio : 1.0f;
    t->motor[i].slot = i;
    t->motor[i].type = (int)m->motor_type;
    t->motor[i].id = (int)m->motor_can_instance->tx_id;
    t->motor[i].online = a->online;
    t->motor[i].dir =
        (m->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? 1 : 0;
    t->motor[i].mode = (int)a->mode;
    t->motor[i].target100 = (a->mode == LMODE_STOP) ? 0 : (int)(a->target * 100.0f);
    t->motor[i].rpm = (int)(m->measure.speed_aps / 6.0f / ratio);
    t->motor[i].angle100 = (int)(MotorOutAngle(i) * 100.0f);
    t->motor[i].turns100 = (int)(MotorOutTurns(i) * 100.0f);
    t->motor[i].temp = (int)m->measure.temperature;
    t->motor[i].cur = (int)m->measure.real_current;

    float p[11];
    MotorReadParams(i, p);
    for (int j = 0; j < 11; j++) t->param[i][j] = (int)(p[j] * 100.0f);
  }
  t->servo.cur10 = (int)(robot->servo->cur_deg * 10.0f);
  t->servo.state = robot->servo->state;
  t->servo.std10 = (int)(robot->servo->std_deg * 10.0f);
  t->servo.prep10 = (int)(robot->servo->prep_deg * 10.0f);
  t->task.spring100 = (int)(CtrlGetSpringTurns() * 100.0f);
  t->task.step = CtrlGetTaskStep();
  t->task.yaw = CtrlGetYawMode();
  t->task.estop = CtrlGetEstop();
  t->task.aim100 = (int)(CtrlGetAimRpm() * 100.0f);
  t->vis.x = robot->vision->x;
  t->vis.ok = robot->vision->ok;
  t->vis.center = robot->vision->center;
  t->vis.err = robot->vision->err;
}

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
  memset(robot, 0, sizeof(RobotInstance));

  robot->motor = MotorsInit();
  robot->servo = LauncherServoInit();
  robot->vision = LauncherVisionInit();

  CtrlInit();  /* 先建 ctrl 状态 */
  LinkInit();
  StoreInit(); /* 恢复掉电参数(需要 ctrl/motor/servo 已就绪) */

  DWT_GetDeltaT(&robot->DWT_CNT);
}

void RobotTask(void) {
  robot->dt = DWT_GetDeltaT(&robot->DWT_CNT);

  LinkTask();    /* 收上位机指令(填内部 cmd) + 周期发遥测 */
  LauncherVisionTask();  /* 坐标超时判定 */
  CtrlTask();    /* 大脑 -> 填 motor/servo ctrl_cmd */
  MotorsTask();  /* 执行 */
  LauncherServoTask();
  StoreTask();

  Launcher_Telemetry_s t;
  BuildTelemetry(&t);
  LinkUpdateTelemetry(&t);

  if (StoreTakeSaved()) LinkSend("SAVED\n");
}
