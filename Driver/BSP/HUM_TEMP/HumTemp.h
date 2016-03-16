/**
  ******************************************************************************
  * @file    HumTemp.h
  * @author  Inc
  * @version V0.0.1
  * @date    15-3-2016
  * @brief   This file is header file for hum_temp sensor Typedef
  *
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */


#ifndef __HUM_TMEP_H
#define __HUM_TMEP_H


#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
//Include
//------------------------------------------------------------------------------
#include "stdint.h"
#include "I2CTypeDef.h"
//------------------------------------------------------------------------------
//Typedef
//------------------------------------------------------------------------------
/* !status of sensor tranmit
 */
typedef enum{
    HUM_TEMP_SUCCESS = 0,
    HUM_TEMP_ERR,
    HUM_TEMP_TIMTOUT,
    HUM_TEMP_NOT_IMPLEMENTED,
}ht_status_t;


typedef struct{
    ht_status_t (*dev_on)(i2c_dev_t *);
    ht_status_t (*dev_off)(void);
    ht_status_t (*read_id)(uint8_t *pHtsID);
    ht_status_t (*reset)(void);
    ht_status_t (*get_humidity)(float *);
    ht_status_t (*get_temperature)(float *);
}ht_dev_t;


#ifdef __cplusplus
}
#endif

#endif /*__HUM_TMEP_H */
/******************************End Of File(INC)***********************************/
