# robot.cmake — run_motor1: 台架"转一个电机"最小 app
# 使用: -DROBOT_TYPE=run_motor1 -DBOARD_TYPE=GIMBAL_BOARD
#       或 make_one\build.ps1 -Robot run_motor1 -Board GIMBAL_BOARD
#
# 只依赖底层 module(Modules/motor/DJImotor + motor_task), 不改任何底层代码;
# 本 app 只提供 robot.h/robot.c + 一个 cfg 头文件。

if (NOT DEFINED BOARD_TYPE)
    set(BOARD_TYPE "GIMBAL_BOARD")
endif ()

if (BOARD_TYPE STREQUAL "GIMBAL_BOARD")
    set(MCU_TYPE "stm32-f4")
elseif (BOARD_TYPE STREQUAL "ONE_BOARD")
    set(MCU_TYPE "stm32-h7")
elseif (BOARD_TYPE STREQUAL "CHASSIS_BOARD")
    set(MCU_TYPE "stm32-h7")
elseif (BOARD_TYPE STREQUAL "H743_BOARD")
    set(MCU_TYPE "stm32-h743")
else ()
    message(FATAL_ERROR "Unknown BOARD_TYPE '${BOARD_TYPE}' (run_motor1 需有 CAN/DJIMotor 的板子)")
endif ()

add_compile_definitions(${BOARD_TYPE})

# 本 app 头文件(robot.h / robot_def.h / run_motor_cfg.h / ui.h)
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})

# 本 app 源文件
file(GLOB RUN_MOTOR_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/*.c")
list(APPEND SOURCES ${RUN_MOTOR_SOURCES})
