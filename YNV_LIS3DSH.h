/*
 * YNV_LIS3DSH.h
 *
 *  Created on: Jan 11, 2021
 *      Author: peyob
 */

#ifndef INC_YNV_LIS3DSH_H_
#define INC_YNV_LIS3DSH_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"


// Register Adress Define
#define LIS3DSH_CTRL_REG3			0x23	//Interruptions
#define LIS3DSH_CTRL_REG4			0X20
#define LIS3DSH_CTRL_REG5			0x24
#define LIS3DSH_OUT_X 				0x28
#define LIS3DSH_OUT_Y				0x2A
#define LIS3DSH_OUT_Z				0x2C
#define LIS3DSH_STATUS				0x27

// Parameters Config Define

	//CTRL_REG4

		//Axis Enable/Disable (ENB/DSB)

#define LIS3DSH_X_AXIS_ENB					((uint8_t)0x01)
#define LIS3DSH_X_AXIS_DSB					((uint8_t)0x00)
#define LIS3DSH_Y_AXIS_ENB					((uint8_t)0x02)
#define LIS3DSH_Y_AXIS_DSB					((uint8_t)0x00)
#define LIS3DSH_Z_AXIS_ENB					((uint8_t)0x04)
#define LIS3DSH_Z_AXIS_DSB					((uint8_t)0x00)

		// Output Data Rate (ODR)

#define LIS3DSH_ODR_PD						((uint8_t)0x00)
#define LIS3DSH_ODR_3_125_HZ				((uint8_t)0x10)
#define LIS3DSH_ODR_6_25_HZ					((uint8_t)0x20)
#define LIS3DSH_ODR_12_5_HZ					((uint8_t)0x30)
#define LIS3DSH_ODR_25_HZ					((uint8_t)0x40)
#define LIS3DSH_ODR_50_HZ					((uint8_t)0x50)
#define LIS3DSH_ODR_100_HZ					((uint8_t)0x60)
#define LIS3DSH_ODR_400_HZ					((uint8_t)0x70)
#define LIS3DSH_ODR_800_HZ					((uint8_t)0x80)
#define LIS3DSH_ODR_1600_HZ					((uint8_t)0x90)

	//CTRL_REG5

		// Anti-Aliasing Bandwidth

#define LIS3DSH_BW_800_HZ					((uint8_t)0x00)
#define LIS3DSH_BW_200_HZ					((uint8_t)0x40)
#define LIS3DSH_BW_400_HZ					((uint8_t)0x80)
#define LIS3DSH_BW_50_HZ					((uint8_t)0xC0)

		//Full-Scale Selection

#define LIS3DSH_FSCALE_2G					((uint8_t)0x00)
#define LIS3DSH_FSCALE_4G					((uint8_t)0x08)
#define LIS3DSH_FSCALE_6G					((uint8_t)0x10)
#define LIS3DSH_FSCALE_8G					((uint8_t)0x18)
#define LIS3DSH_FSCALE_16G					((uint8_t)0x20)

// Return Status Define

#define LIS3DSH_STATUS_OK					"LIS3DSH_OK !\n\r"
#define LIS3DSH_STATUS_ERROR				"LIS3DSH_ERROR !\n\r"

// Init Struct

typedef struct
{
		uint8_t XEnable;
		uint8_t YEnable;
		uint8_t ZEnable;
		uint8_t OutputDataRate;
		uint8_t AntiAliasing;
		uint8_t FullScale;
}LIS3DSH_init_t;

// Status Enum

typedef enum
{
	LIS3DSH_OK,
	LIS3DSH_ERROR,
}LIS3DSH_Status_t;

// Write Function

LIS3DSH_Status_t LIS3DSH_Write_Reg(SPI_HandleTypeDef *posSPI,

                                    uint8_t reg_addr,
                                    uint8_t * dataW,
                                    uint8_t size);

// Read Function

LIS3DSH_Status_t LIS3DSH_Read_Reg(SPI_HandleTypeDef *posSPI,

                                    uint8_t reg_addr,
                                    uint8_t * dataR,
                                    uint8_t size);
// Init Function

LIS3DSH_Status_t LIS3DSH_Init(SPI_HandleTypeDef *posSPI,

                            LIS3DSH_init_t PosInitDef);

// Get Position

	// X Axis Pos

LIS3DSH_Status_t LIS3DSH_Get_X(SPI_HandleTypeDef *posSPI,
							   uint8_t reg_addr,
							   uint16_t *posX);

	// Y Axis Pos

LIS3DSH_Status_t LIS3DSH_Get_Y(SPI_HandleTypeDef *posSPI,
							   uint8_t reg_addr,
							   uint16_t *posY);

	// Z Axis Pos

LIS3DSH_Status_t LIS3DSH_Get_Z(SPI_HandleTypeDef *posSPI,
							   uint8_t reg_addr,
							   uint16_t *posZ);

#endif /* INC_YNV_LIS3DSH_H_ */
