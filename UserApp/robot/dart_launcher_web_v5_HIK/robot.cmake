# robot.cmake — 制导飞镖发射架 · 网页控制版 v2
# 使用: -DROBOT_TYPE=dart_launcher_web_v2 -DBOARD_TYPE=GIMBAL_BOARD

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

include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})

file(GLOB_RECURSE ROBOT_SOURCES
        CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

list(APPEND SOURCES ${ROBOT_SOURCES})
