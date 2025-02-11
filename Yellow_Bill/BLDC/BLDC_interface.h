/*
 * @file     BLDC_interface.h
 * @date     Dec 16, 2024
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains
 */

#ifndef BLDC_INTERFACE_H_
#define BLDC_INTERFACE_H_

#include "../../Core/Inc/main.h"



extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;



void BLDC_T200_PWM_Init();
void BLDC_T200_Init();
void BLDC_T200_StopAll();
void BLDC_T200_SetMotor(uint8_t MotorChannel, uint16_t usSpeed);

#endif /* BLDC_INTERFACE_H_ */
