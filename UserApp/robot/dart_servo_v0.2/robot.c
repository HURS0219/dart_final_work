/*
 * robot.c — 制导飞镖舵机子系统 v0.2 入口
 */
#include "robot.h"

#include "dart_axis.h"
#include "dart_cfg.h"
#include "dart_proto.h"
#include "user_lib.h"

RobotInstance *robot = NULL;

void RobotInit(void) {
  robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));
  DartAxisInit();  /* 含 DartPwmInit */
  DartCfgInit();   /* 加载标定 */
  DartProtoInit();
}

void RobotTask(void) {
  DartAxisTask();
  DartProtoTask();
  DartCfgTask();
}
