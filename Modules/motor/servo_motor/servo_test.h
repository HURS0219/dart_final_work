#ifndef SERVO_TEST_H
#define SERVO_TEST_H

/* 舵机 bring-up 测试 (临时)
 *
 * 上电后让 PWM1(PE9) / PWM2(PE11) 两个舵机在 90° 附近缓慢来回摆动,
 * 用于确认 50Hz PWM 通路、接线和供电是否正常。
 *
 * 正式控制时: 删除 servo_test.c / servo_test.h, 并去掉 motor_task.c 中的调用。
 * 仅在 STM32F407 (GIMBAL_BOARD) 生效, 其它板子为空实现。 */

void ServoTestInit(void);
void ServoTestTask(void);

#endif /* SERVO_TEST_H */
