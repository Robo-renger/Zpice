/*
 * @file     i2c.c
 * @date     Feb 11, 2025
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains source file of i2c interface between 
 *           stm32f401rtc and Raspberry Pi 4, functionally like PCA Servo Driver
 */



#include "i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define I2C_SLAVE_ADDRESS 0x08  // Match the address in the master code

uint8_t receivedData[10] = {0};  // Variable to store received data
uint8_t counter = 0;

extern uint8_t Channel_Received_Rasp;
extern uint16_t usPulse_Speed_Received_Rasp;



/**
 * @brief  Get the channel and the pulse width from the received string
 * @param  ReceivedStr: The received string from the master
 * @param  Ret_channel: The channel number (Return value)
 * @param  Ret_usPulse: The pulse width in microseconds (Return value)
*/
void GetChannels_Value(uint8_t * ReceivedStr , uint8_t *Ret_channel , uint16_t * Ret_usPulse){
	int channel, usPulse;

   // Use strtok to split the string by the delimiter '-'
   char *token = strtok(ReceivedStr, "-");
   if (token != NULL) {
	   *Ret_channel = atoi(token);
   }

   token = strtok(NULL, "-");
   if (token != NULL) {
	   *Ret_usPulse = atoi(token);
   }
}


/**
 * @brief Restart listening for the next transmission
*/
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_EnableListen_IT(hi2c);
}


/**
 * @brief  Callback function when the slave address is matched
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
  if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
    // Master wants to send data to the slave
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, &receivedData, 7, I2C_FIRST_AND_LAST_FRAME);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
}

/**
 * @brief  Callback function when data reception is complete and Data received successfully
*/
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  // Data received successfully
  // Process the received data (e.g., print it or store it)
  receivedData[0]='0';
  GetChannels_Value(receivedData, &Channel_Received_Rasp,&usPulse_Speed_Received_Rasp);

  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  counter++;
}


/**
 * @brief  Callback function when an error occurs
*/
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef * hi2c){
	 HAL_I2C_EnableListen_IT(hi2c);
}

