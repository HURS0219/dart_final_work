# ins_task

neozng1@hnu.edu.cn

## 硬触发流程

image-20221113212706633

`times%10` 是固定相机的采集频率为100hz，请根据视觉算法实际能达到的最大帧率调整。

## 算法解析

介绍EKF四元数姿态解算的教程在:[四元数EKF姿态更新算法](https://zhuanlan.zhihu.com/p/454155643)

## 模块移植

由于历史遗留问题,IMU模块耦合程度高.后续实现BSP_SPI,将bmi088 middleware删除.仅保留BMI088读取的协议和寄存器定义等,单独实现IMU模块.

> 移植已经完成,请转而使用module/BMI088的模块. 当前文件夹将在beta1.2停止支持, 1.5之后删除. INS_Task届时会被放到algorithm中,以提供对不同IMU的兼容.

## 修正安装误差

使用的是IMU_Param_Correction函数，用于修正IMU安装误差与标度因数误差，即陀螺仪轴和云台轴的安装偏移。
函数需要传入一个IMU_Init_Config_s* imu_init_config参数和解算出来的float gyro[3], float accel[3]。

IMU_Init_Config_s包括：

```c
uint8_t flag;      // 参数更新标志位，当参数发生变化时置1
float scale[3];    // 三轴标度因数修正系数 [0]-X轴 [1]-Y轴 [2]-Z轴
float Yaw;         // Yaw轴安装偏角修正 单位: °
float Pitch;       // Pitch轴安装偏角修正 单位: °
float Roll;        // Roll轴安装偏角修正 单位: °
```

flag给1，表示参数更新。
scale为三轴标度因数修正系数，默认为1.0f，表示一比一。scale增加，表示传感器的测量结果比真值要大。
Yaw、Pitch、Roll为三轴安装偏角修正系数，默认为0.0f，表示无安装偏角。如果存在安装偏角，比如需要按照下图yaw、roll、pitch方向，设置坐标系旋转角度。例如板子的坐标系需要按yaw轴正向旋转90°才是车体坐标系，则Yaw设置为90.0f。下图中就是roll转到pitch位置。
imu三轴旋转正方向.png

IMU_Init_Config_s* imu_init_config写在gimbal.h或者chassis.h里。