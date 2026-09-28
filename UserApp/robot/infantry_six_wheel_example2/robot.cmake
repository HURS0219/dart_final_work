# Set the type of chassis, gimbal and shooter modules for infantry wheel legged
set(CHASSIS_TYPE chassis_six_wheel)
set(GIMBAL_TYPE gimbal_standard)
set(SHOOT_TYPE shoot_standard)

# 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新CMake&编译,只能存在一个定义!
# ONE_BOARD GIMBAL_BOARD CHASSIS_BOARD
# add_compile_definitions(ONE_BOARD)
add_compile_definitions(GIMBAL_BOARD)
# add_compile_definitions(CHASSIS_BOARD)

add_compile_definitions(FRICTION_NUM=2)

# 遥控器选择
# add_compile_definitions(USE_RC_CTRL)
add_compile_definitions(USE_OCD_CTRL)


# Include directories for header file searching
include_sub_directories_recursively(${CMAKE_CURRENT_LIST_DIR})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/shoot/${SHOOT_TYPE})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/gimbal/${GIMBAL_TYPE})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/UserApp/components/chassis/${CHASSIS_TYPE})

# Define source files for the robot application
file(GLOB_RECURSE ROBOT_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/chassis/${CHASSIS_TYPE}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/gimbal/${GIMBAL_TYPE}/*.c"
        "${CMAKE_SOURCE_DIR}/UserApp/components/shoot/${SHOOT_TYPE}/*.c"
)

# Add the robot source files to the global SOURCES list
list(APPEND SOURCES ${ROBOT_SOURCES})