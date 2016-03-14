/**
  ******************************************************************************
  * @file    INC_I2C.h
  * @author  Inc
  * @version V0.0.1
  * @date    14-Sep-2015
  * @brief   This file contains the I2C function prototype.
  *
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef   __INC_I2C_H
#define   __INC_I2C_H

//------------------------------------------------------------------------------
//Include
//------------------------------------------------------------------------------
#ifndef _OWN_I2C_SOFT
    #define I2C_EXT extern
#else
    #define I2C_EXT
#endif

    
#include "stm32f4xx_hal.h"


//------------------------------------------------------------------------------
//Typedef
//------------------------------------------------------------------------------
typedef enum{
    I2C_ERR = 0,
    I2C_SUCCESS = !I2C_ERR,
}i2c_t;

//------------------------------------------------------------------------------
//Extern
//------------------------------------------------------------------------------
I2C_EXT void i2c_soft_init(void);
I2C_EXT i2c_t i2c_single_write( uint8_t chSlaveAddr, uint8_t chRegAddr, uint8_t chByte);
I2C_EXT i2c_t i2c_single_write16( uint8_t chSlaveAddr, uint16_t hwRegAddr, uint8_t chByte);
I2C_EXT uint8_t i2c_single_read(uint8_t chSlaveAddr, uint8_t chRegAddr);
I2C_EXT uint8_t i2c_single_read16(uint8_t chSlaveAddr, uint16_t hwRegAddr);
I2C_EXT i2c_t i2c_multi_read(uint8_t chSlaveAddr, uint8_t chRegAddr, uint8_t *pBuf, uint16_t hwLen);
I2C_EXT i2c_t i2c_multi_read16(uint8_t chSlaveAddr, uint16_t hwRegAddr, uint8_t *pBuf, uint16_t hwLen);








#endif
/******************************End Of File(INC)***********************************/
