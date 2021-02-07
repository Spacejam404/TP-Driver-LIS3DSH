/*
 * YNV_LIS3DSH.c
 *
 *  Created on: Jan 11, 2021
 *      Author: peyob
 */

#include "YNV_LIS3DSH.h"

LIS3DSH_Status_t LIS3DSH_Write_Reg(SPI_HandleTypeDef *posSPI,

                                    uint8_t reg_addr,
                                    uint8_t * dataW,
                                    uint8_t size)
{
	dataW[0] = reg_addr;

	if(HAL_SPI_Transmit(posSPI, dataW, size, 10) == HAL_OK)
	{
		return LIS3DSH_OK;
	}
	return LIS3DSH_ERROR;
}
LIS3DSH_Status_t LIS3DSH_Read_Reg(SPI_HandleTypeDef *posSPI,
								  uint8_t reg_addr,
								  uint8_t * dataR,
								  uint8_t size)
{
	dataR[1] = reg_addr;
	if(HAL_SPI_Transmit(posSPI, dataR, size, 10) == HAL_OK)
	{
		if(HAL_SPI_Receive(posSPI, dataR, size, 10) == HAL_OK)
		{
			return LIS3DSH_OK;
		}
		return LIS3DSH_ERROR;
	}
}

LIS3DSH_Status_t LIS3DSH_Init(SPI_HandleTypeDef *posSPI,
							  LIS3DSH_init_t PosInitDef)
{
	 uint8_t SPIData[1] = {0x00};
	 uint8_t SPICheckData[1] = {0x00};

	    /*Data register config*/
/*	    SPIData[1] |= (PosInitDef -> XEnable = LIS3DSH_X_AXIS_ENB);
	    SPIData[1] |= (PosInitDef -> YEnable = LIS3DSH_Y_AXIS_ENB);
	    SPIData[1] |= (PosInitDef -> OutputDataRate = LIS3DSH_ODR_PD);
	    SPIData[2] |= (PosInitDef -> AntiAliasing = LIS3DSH_BW_800_HZ);
	    SPIData[2] |= (PosInitDef -> FullScale = LIS3DSH_FSCALE_2G);*/

	    if(LIS3DSH_Write_Reg(posSPI, LIS3DSH_CTRL_REG4, SPIData, 2) == LIS3DSH_OK)
	    {
	        if (LIS3DSH_Read_Reg(posSPI, LIS3DSH_CTRL_REG4, SPICheckData, 1) == LIS3DSH_OK)
	        {
	            if(SPICheckData[0] == SPIData[1])
	            {
	                return LIS3DSH_OK;
	            }
	        }
	    }
	    return LIS3DSH_ERROR;

	    if(LIS3DSH_Write_Reg(posSPI, LIS3DSH_CTRL_REG5, SPIData, 2) == LIS3DSH_OK)
	    	    {
	    	        if (LIS3DSH_Read_Reg(posSPI, LIS3DSH_CTRL_REG5, SPICheckData, 1) == LIS3DSH_OK)
	    	        {
	    	            if(SPICheckData[0] == SPIData[2])
	    	            {
	    	                return LIS3DSH_OK;
	    	            }
	    	        }
	    	    }
	    	    return LIS3DSH_ERROR;
}
