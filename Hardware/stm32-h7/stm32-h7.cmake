# board specific settings, arch/fpu/instruction
set(MCU_FLAGS -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16)
set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/STM32H723VGTX_FLASH.ld") # 指定链接脚本
set(DSP_NAME "libCMSISDSP.a") # 指定DSP库名称
link_directories(${CMAKE_CURRENT_LIST_DIR}/Middlewares/ST/ARM/DSP/Lib)

# 汇编文件路径
set(ASM_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/Startup/startup_stm32h723vgtx.s
        ${CMAKE_CURRENT_LIST_DIR}/Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
)

# add_compile_definitions() works for compile stage
# while add_definitions() works for both compile and link stage
add_definitions(
        -DUSE_HAL_DRIVER
        -DSTM32H723xx
        -DARM_MATH_CM7
        -D__FPU_PRESENT=1U
) # need -D<macro> to define macro