# robot.cmake — dart_final_test_app: 复用 dart_final/app 的各 app 服务 + 本目录 test.c
# 用途: 逐个黑盒测试 dart_final 的 app(fin/guidance/vision/imu)。
# 使用: build.ps1 -Robot dart_final_test_app -Board GIMBAL_BOARD

if (NOT DEFINED BOARD_TYPE)
    set(BOARD_TYPE "GIMBAL_BOARD")
endif ()

if (BOARD_TYPE STREQUAL "ONE_BOARD")
    set(MCU_TYPE "stm32-h7")
elseif (BOARD_TYPE STREQUAL "GIMBAL_BOARD")
    set(MCU_TYPE "stm32-f4")
elseif (BOARD_TYPE STREQUAL "CHASSIS_BOARD")
    set(MCU_TYPE "stm32-h7")
else ()
    message(FATAL_ERROR "Unknown BOARD_TYPE '${BOARD_TYPE}'")
endif ()

add_compile_definitions(${BOARD_TYPE})

# 头文件: 本目录 + 复用 dart_final 与 dart_final/app (robot.h/ui.h/robot_def.h/dart_final_cfg.h/app)
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_directories(
        ${CMAKE_CURRENT_LIST_DIR}/../dart_final
        ${CMAKE_CURRENT_LIST_DIR}/../dart_final/app
)

# 源文件: 本目录 test.c + 复用 dart_final/app/*.c (不含 dart_final/robot.c, 避免 RobotInit 冲突)
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/*.c")
file(GLOB APP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/../dart_final/app/*.c")
list(APPEND SOURCES ${TEST_SOURCES} ${APP_SOURCES})
