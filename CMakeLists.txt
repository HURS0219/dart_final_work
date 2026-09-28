cmake_minimum_required(VERSION 3.16)
# 指定编译平台/架构与语言标准, 推荐指定Ninja为构建工具,可以加快编译速度(相比make)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
# 指定工具链
set(CMAKE_C_COMPILER_FORCED TRUE) # skip compiler test
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 指定工程名称和语言类型
project(control-2026 C ASM)

# 选择构建类型
if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif() # Debug Release RelWithDebInfo MinSizeRel

#set(ROBOT_TYPE "steering_infantry_gimbal")
#set(ROBOT_TYPE "sentry_mecanum")
#set(ROBOT_TYPE "engineering")
#set(ROBOT_TYPE "infantry_wheel_legged_heu")
# 选择机器人类型: 可在命令行用 -DROBOT_TYPE=xxx 覆盖 (VS Code 一键烧录用)
if(NOT DEFINED ROBOT_TYPE)
    set(ROBOT_TYPE "infantry_wheel_legged_sjtu")
endif()
#set(ROBOT_TYPE "infantry_wheel_legged_demo")
#set(ROBOT_TYPE "hero_mecanum")
#set(ROBOT_TYPE "infantry_steering")
#set(ROBOT_TYPE "infantry_mecanum")
#set(ROBOT_TYPE "dart")
#set(ROBOT_TYPE "dart_launcher")
#set(ROBOT_TYPE "wifi_gm6020")
#set(ROBOT_TYPE "run_motor1")
#set(ROBOT_TYPE "run_motor2")
#set(ROBOT_TYPE "debug")；

# 递归包含头文件的函数
function(include_sub_directories_recursively root_dir)
    if (IS_DIRECTORY ${root_dir})               # 当前路径是一个目录吗，是的话就加入到包含目录
        message("include dir: " ${root_dir})
        include_directories(${root_dir})
    endif ()

    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*) # 获得当前目录下的所有文件，让如ALL_SUB列表中
    foreach (sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})
            include_sub_directories_recursively(${root_dir}/${sub}) # 对子目录递归调用，包含
        endif ()
    endforeach ()
endfunction()

# robot.cmake 根据 BOARD_TYPE 设置 MCU_TYPE，因此必须在 hardware cmake 之前 include
include(${CMAKE_SOURCE_DIR}/UserApp/robot/${ROBOT_TYPE}/robot.cmake)

include(${CMAKE_SOURCE_DIR}/Hardware/${MCU_TYPE}/${MCU_TYPE}.cmake)

# Generic compiler settings for optimization and basic link lib
add_compile_options(-pipe ${MCU_FLAGS} -Wall -fmessage-length=0 # basic options
        -ffunction-sections -fdata-sections -fno-common # optimize options
)
add_link_options(-pipe ${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,--no-warn-rwx-segments # close RWX warning
        -lm -lc # lib options
        -Wl,--gc-sections -flto -specs=nano.specs # optimize options
        -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map -Wl,--cref -Wl,--print-memory-usage # map options
) # if your executable is too large , try option '-s' to strip symbols

include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Hardware/${MCU_TYPE})
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Bsp)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Modules)
include_directories(${CMAKE_SOURCE_DIR}/UserApp)

file(GLOB_RECURSE BASE_SOURCES
        "Hardware/${MCU_TYPE}/*.c"
        "Bsp/*.c"
        "Modules/*.c"
        "UserApp/os_task.c"
)
list(APPEND SOURCES ${BASE_SOURCES})

# Build types
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g -gdwarf-2)
    add_definitions(-DESC_DEBUG) # ESC Debug
endif ()

set_source_files_properties(${ASM_SOURCES} PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

# build binary and hex file
add_executable(${PROJECT_NAME}.elf ${SOURCES} ${ASM_SOURCES} ${LINKER_SCRIPT})
target_link_libraries(${PROJECT_NAME}.elf ${DSP_NAME} m) # link DSP lib and math lib

add_custom_command(
        TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
        COMMENT "Building hex & bin file...\nEXCUTABLE SIZE:"
        COMMAND ${SIZE} ${PROJECT_NAME}.elf
)
