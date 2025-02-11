/*
 * @file     BLDC_interface.c
 * @date     Dec 16, 2024
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains
 */

#include "BLDC_interface.h"

/***/
void BLDC_T200_PWM_Init(){
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

}
void BLDC_T200_Init(){
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 1502);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 1502);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 1502);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, 1502);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, 1502);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 1502);
  HAL_Delay(4000);
}

void BLDC_T200_StopAll(){
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 1502);
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 1502);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 1502);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 1502);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, 1502);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, 1502);

}

void BLDC_T200_SetMotor(uint8_t MotorChannel, uint16_t usSpeed){
	if((usSpeed < 1000)|| (usSpeed > 2000)){
		/*Log Warning : Wrong microsecond speed */
	}else{
		switch (MotorChannel){
			case 0:
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,usSpeed+2);
				break;
			case 1:
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, usSpeed+2);
				break;
			case 2:
				 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, usSpeed+2);
				break;
			case 3:
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,usSpeed+2);
				break;
			case 4:
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,usSpeed+2);
				break;
			case 5:
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,usSpeed+2);
				break;

			default:
				/*Log Warning : Wrong MotorChannel */
				break;
		}

	}

}

