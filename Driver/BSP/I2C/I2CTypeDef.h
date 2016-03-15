/**
  ******************************************************************************
  * @file    I2CTypeDef.h
  * @author  Inc
  * @version V0.0.1
  * @date    14-3-2016
  * @brief   This file is header for I2C Typedef.
  *
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
#ifndef   __I2C_TYPEDEF_H
#define   __I2C_TYPEDEF_H

//------------------------------------------------------------------------------
//Include
//------------------------------------------------------------------------------
#include "stdint.h"

//------------------------------------------------------------------------------
//Typedef
//------------------------------------------------------------------------------
/* !status of i2c tranmit
 */
typedef enum{
    I2C_SUCCESS = 0,
    I2C_ERR = !I2C_SUCCESS,
}i2c_status_t;


typedef struct{
    i2c_status_t (*dev_on)(void);
    i2c_status_t (*dev_off)(void);
    i2c_status_t (*single_write_byte)( uint8_t chSlaveAddr, uint8_t chRegAddr, uint8_t chByte);
    i2c_status_t (*single_write_word)( uint8_t chSlaveAddr, uint16_t hwRegAddr, uint8_t chByte);
    uint8_t      (*single_read_byte)(uint8_t chSlaveAddr, uint8_t chRegAddr);
    uint8_t      (*single_read_word)(uint8_t chSlaveAddr, uint16_t hwRegAddr);
    i2c_status_t (*multi_write_byte)( uint8_t chSlaveAddr, uint8_t chRegAddr, uint8_t *pByte, uint16_t hwLen);
    i2c_status_t (*multi_write_word)( uint8_t chSlaveAddr, uint16_t hwRegAddr, uint8_t *pByte, uint16_t hwLen);
    i2c_status_t (*multi_read_byte)(uint8_t chSlaveAddr, uint8_t chRegAddr, uint8_t *pBuf, uint16_t hwLen);
    i2c_status_t (*multi_read_word)(uint8_t chSlaveAddr, uint16_t hwRegAddr, uint8_t *pBuf, uint16_t hwLen);
}i2c_dev_t;

#endif /*__I2C_TYPEDEF_H */
/******************************End Of File(INC)***********************************/
