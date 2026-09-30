# robot.cmake — 制导飞镖发射架 整机飞控 (dart_launcher_final)
# 使用: -DROBOT_TYPE=dart_launcher_final -DBOARD_TYPE=GIMBAL_BOARD
#
# 本 app 目标板为 GIMBAL_BOARD (STM32F407, C 板)。

if (NOT DEFINED BOARD_TYPE)
    set(BOARD_TYPE "GIMBAL_BOARD")
endif ()

if (BOARD_TYPE STREQUAL "ONE_BOARD")
    set(MCU_TYPE "stm32-h7")
elseif (BOARD_TYPE STREQUAL "GIMBAL_BOARD")
    set(MCU_TYPE "stm32-f4")
elseif (BOARD_TYPE STREQUAL "CHASSIS_BOARD")
    set(MCU_TYPE "stm32-h7")
elseif (BOARD_TYPE STREQUAL "H743_BOARD")
    set(MCU_TYPE "stm32-h743")
elseif (BOARD_TYPE STREQUAL "DART_F405_BOARD")
    set(MCU_TYPE "stm32-f405-dart")
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
