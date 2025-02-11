/*
 * @file     i2c.h
 * @date     Feb 11, 2025
 * @author   Ahmed Samy
 * @link     https://github.com/AhmedSamymoh
 *
 * @brief    This file contains
 */

#ifndef I2C_I2C_H_
#define I2C_I2C_H_


#include "../../Core/Inc/main.h"



extern I2C_HandleTypeDef hi2c1;


void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef * hi2c);



#endif /* I2C_I2C_H_ */
