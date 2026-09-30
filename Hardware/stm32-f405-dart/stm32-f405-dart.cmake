# stm32-f405-dart board settings  (STM32F405RGT6 / LQFP64, dart control board)
set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)
set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/STM32F405RGTX_FLASH.ld")
set(DSP_NAME "libCMSISDSP.a")
link_directories(${CMAKE_CURRENT_LIST_DIR}/Middlewares/ST/ARM/DSP/Lib)

# assembly sources (CubeMX generated startup + SEGGER RTT)
set(ASM_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/Startup/startup_stm32f405rgtx.s
        ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
)

add_definitions(
        -DUSE_HAL_DRIVER
        -DSTM32F405xx
        -DARM_MATH_CM4
        -D__FPU_PRESENT=1U
)

# 本板(LQFP64 精简)不含总线/外设: 裁剪 dart_final 用不到的 Bsp/Modules 源, 避免引用缺失句柄
set(BOARD_EXCLUDE
        "Bsp/can/"
        "Bsp/iic/"
        "Modules/alarm/"
        "Modules/can_comm/"
        "Modules/display/"
        "Modules/oled/"
        "Modules/referee/"
        "Modules/remote/"
        "Modules/super_cap/"
        "Modules/master_machine/"
        "Modules/TFmini/"
        "Modules/vofa/"
        "Modules/VT13/"
        "Modules/motor/DJImotor/"
        "Modules/motor/DMmotor/"
        "Modules/motor/motor_task.c"
        "Modules/motor/run_motor1/"
)
