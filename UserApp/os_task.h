/**
 ******************************************************************************
 * @file    os_task.c
 * @author  Enhao Zhang
 * @date    2025/10/9
 * @brief   None
 ******************************************************************************
 * @attention
 * None
 *
 ******************************************************************************
 */
#pragma once


void OSTaskInit();

void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);

