/**
  ******************************************************************************
  * @file    Project/Drive/HUM_TEMP/HTS221.c
  * @author  Inc
  * @version V0.0.1
  * @date    15-3-2016
  * @brief   The Module of HTS221
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

#define _HTS221_MODULE

//==============================================================================
//Include
//==============================================================================
#include "HTS221.h"
#include "I2CSoft.h"
#include <math.h>
// !User Include

//==============================================================================
//Private variables
//==============================================================================

//private save calibration of Humidity and Temperature
static struct {
    /* Temperature in degree for calibration  */
    float T0DegC;
    float T1DegC;
    /* Humidity for calibration  */
    float H0Rh;
    float H1Rh;
}s_tHumTempCal;

//private save output date of Humidity and Temperature
static struct {
    int16_t T0Out;
    int16_t T1Out;
    int16_t H0T0Out;
    int16_t H1T0Out;
}s_tHumTempOut;

// i2c_transmit function point
i2c_dev_t *pHTSTransmit = NULL;

hts221_info_t hts221_info = {
    0,
    0,
    0,
    0,
    0,
    HTS221_ODR_12_5Hz,
};

//==============================================================================
//private function prototypes
//==============================================================================
static ht_status_t calibration(void);
static ht_status_t power_on(void);
static ht_status_t power_off(void);

//==============================================================================
//Static Function
//==============================================================================
ht_status_t hts221_init(i2c_dev_t *pI2cDev)
{
    uint8_t tmp = 0x00;

    pHTSTransmit = pI2cDev;

    pHTSTransmit->dev_on();

    if(HUM_TEMP_SUCCESS != power_on()){
        return HUM_TEMP_ERR;
    }

    if(HUM_TEMP_SUCCESS != calibration()){
        return HUM_TEMP_ERR;
    }
    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, &tmp, 1))
    {
        return HUM_TEMP_ERR;
    }

    /* Output Data Rate selection */
    tmp &= ~(HTS221_ODR_MASK);
    tmp |= hts221_info.OutputDataRate;

    if(pHTSTransmit->single_write_byte(HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, tmp))
    {
        return HUM_TEMP_ERR;
    }


    return HUM_TEMP_SUCCESS;
}

ht_status_t hts221_deinit(void)
{
    if(power_off()){
        return HUM_TEMP_ERR;
    }
    //---------<<<
    return HUM_TEMP_SUCCESS;
}

//------------------------------------------------------------------------------
//HTS221 Calibration procedure
//Param:  None
//Return: ht_status_t: it will return HUM_TEMP_SUCCESS
//                                    HUM_TEMP_ERR
//                                    HUM_TEMP_TIMTOUT
//                                    HUM_TEMP_NOT_IMPLEMENTED
ht_status_t calibration(void)
{
    /* Temperature Calibration */
    /* Temperature in degree for calibration ( "/8" to obtain float) */
    uint16_t T0_degC_x8_L, T0_degC_x8_H, T1_degC_x8_L, T1_degC_x8_H;
    uint8_t H0_rh_x2, H1_rh_x2;
    uint8_t tempReg[2] = {0, 0};

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_T0_degC_X8_ADDR, tempReg, 1)){
        return HUM_TEMP_ERR;
    }

    T0_degC_x8_L = (uint16_t)tempReg[0];

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_T1_T0_MSB_X8_ADDR, tempReg, 1)){
        return HUM_TEMP_ERR;
    }

    T0_degC_x8_H = (uint16_t) (tempReg[0] & 0x03);
    s_tHumTempCal.T0DegC = ((float)((T0_degC_x8_H << 8) | (T0_degC_x8_L))) / 8;

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_T1_degC_X8_ADDR, tempReg, 1)){
        return HUM_TEMP_ERR;
    }

    T1_degC_x8_L = (uint16_t)tempReg[0];

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_T1_T0_MSB_X8_ADDR, tempReg, 1)){
        return HUM_TEMP_ERR;
    }

    T1_degC_x8_H = (uint16_t) (tempReg[0] & 0x0C);
    T1_degC_x8_H = T1_degC_x8_H >> 2;
    s_tHumTempCal.T1DegC = ((float)((T1_degC_x8_H << 8) | (T1_degC_x8_L))) / 8;

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS,
                                     (HTS221_T0_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD),
                                     tempReg,
                                     2)){
        return HUM_TEMP_ERR;
    }

    s_tHumTempOut.T0Out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS,
                                     (HTS221_T1_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD),
                                     tempReg,
                                     2)){
        return HUM_TEMP_ERR;
    }

    s_tHumTempOut.T1Out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);

    /* Humidity Calibration */
    /* Humidity in degree for calibration ( "/2" to obtain float) */

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS,
                                     HTS221_H0_RH_X2_ADDR,
                                     &H0_rh_x2,
                                     1)){
        return HUM_TEMP_ERR;
    }

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS,
                                     HTS221_H1_RH_X2_ADDR,
                                     &H1_rh_x2,
                                     1)){
        return HUM_TEMP_ERR;
    }

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS,
                                     (HTS221_H0_T0_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD),
                                     &tempReg[0],
                                     2)){
        return HUM_TEMP_ERR;
    }

    s_tHumTempOut.H0T0Out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS,
                                     (HTS221_H1_T0_OUT_L_ADDR  | HTS221_I2C_MULTIPLEBYTE_CMD),
                                     &tempReg[0],
                                     2)){
        return HUM_TEMP_ERR;
    }

    s_tHumTempOut.H1T0Out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);

    s_tHumTempCal.H0Rh = ((float)H0_rh_x2) / 2;
    s_tHumTempCal.H1Rh = ((float)H1_rh_x2) / 2;

    return HUM_TEMP_SUCCESS;
}


//------------------------------------------------------------------------------
//breaf: power_on
//Param:  None
//Return: ht_status_t: it will return HUM_TEMP_SUCCESS
//                                    HUM_TEMP_ERR
//                                    HUM_TEMP_TIMTOUT
//                                    HUM_TEMP_NOT_IMPLEMENTED
ht_status_t power_on(void)
{
    uint8_t tmpReg = 0;
    i2c_status_t status;

    /* Read the register content */
    status = pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, &tmpReg, 1);
    if(I2C_SUCCESS != status){
        return HUM_TEMP_ERR;
    }

    /* Set the power down bit */
    tmpReg |= HTS221_MODE_ACTIVE;

    /* Write register */
    if(pHTSTransmit->single_write_byte(HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, tmpReg)){
        return HUM_TEMP_ERR;
    }

    return HUM_TEMP_SUCCESS;
}

//breaf: power_off
//Param:  None
//Return: ht_status_t: it will return HUM_TEMP_SUCCESS
//                                    HUM_TEMP_ERR
//                                    HUM_TEMP_TIMTOUT
//                                    HUM_TEMP_NOT_IMPLEMENTED
ht_status_t power_off(void)
{
    uint8_t tmpReg;

    /* Read the register content */
    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, &tmpReg, 1)){
        return HUM_TEMP_ERR;
    }

    /* Reset the power down bit */
    tmpReg &= ~(HTS221_MODE_ACTIVE);

    /* Write register */
    if(pHTSTransmit->single_write_byte(HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, tmpReg))
    {
        return HUM_TEMP_ERR;
    }

    return HUM_TEMP_SUCCESS;
}

//breaf: hts221_read_id
//Param:  uint8_t *ht_id
//Return: ht_status_t: it will return HUM_TEMP_SUCCESS
//                                    HUM_TEMP_ERR
//                                    HUM_TEMP_TIMTOUT
//                                    HUM_TEMP_NOT_IMPLEMENTED
ht_status_t hts221_read_id(uint8_t *pHtsID)
{
    if(!pHtsID)
    {
        return HUM_TEMP_ERR;
    }

    if(I2C_SUCCESS != pHTSTransmit->multi_read_byte(HTS221_ADDRESS,
                                HTS221_WHO_AM_I_ADDR,
                                pHtsID, 1)){
        return HUM_TEMP_ERR;
    }

    return HUM_TEMP_SUCCESS;
}

//breaf: hts221_reboot
//Param:  none
//Return: ht_status_t: it will return HUM_TEMP_SUCCESS
//                                    HUM_TEMP_ERR
//                                    HUM_TEMP_TIMTOUT
//                                    HUM_TEMP_NOT_IMPLEMENTED
ht_status_t hts221_reboot(void)
{
    uint8_t tmpreg;

    /* Read CTRL_REG2 register */
    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, &tmpreg, 1)
                    != HUM_TEMP_SUCCESS){
        return HUM_TEMP_ERR;
    }

    /* Enable or Disable the reboot memory */
    tmpreg |= HTS221_BOOT_REBOOTMEMORY;

    /* Write value to MEMS CTRL_REG2 regsister */
    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, &tmpreg, 1)
                    != HUM_TEMP_SUCCESS){
        return HUM_TEMP_ERR;
    }

    return HUM_TEMP_SUCCESS;
}

//breaf: hts221_get_hum
//Param:  float *pfData
//Return: ht_status_t: it will return HUM_TEMP_SUCCESS
//                                    HUM_TEMP_ERR
//                                    HUM_TEMP_TIMTOUT
//                                    HUM_TEMP_NOT_IMPLEMENTED
ht_status_t hts221_get_hum(float* pfData)
{
    int16_t H_T_out, humidity_t;
    uint8_t tempReg[2] = {0, 0};
    uint8_t tmp = 0x00;
    float H_rh;

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, &tmp, 1) != I2C_SUCCESS)
    {
        return HUM_TEMP_ERR;
    }

    /* Output Data Rate selection */
    tmp &= (HTS221_ODR_MASK);

    if(tmp == 0x00)
    {
        if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, &tmp, 1)!= I2C_SUCCESS)
        {
            return HUM_TEMP_ERR;
        }

        /* Serial Interface Mode selection */
        tmp &= ~(HTS221_ONE_SHOT_MASK);
        tmp |= HTS221_ONE_SHOT_START;

        if(pHTSTransmit->single_write_byte(HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, tmp)!= I2C_SUCCESS)
        {
            return HUM_TEMP_ERR;
        }

        do
        {

            if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_STATUS_REG_ADDR, &tmp, 1) != I2C_SUCCESS)
            {
                return HUM_TEMP_ERR;
            }

        }
        while(!(tmp & 0x02));
    }


    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, (HTS221_HUMIDITY_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD),
                      &tempReg[0], 2) != I2C_SUCCESS)
    {
        return HUM_TEMP_ERR;
    }

    H_T_out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);

    H_rh = ( float )(((( H_T_out - s_tHumTempOut.H0T0Out ) *
                    ( s_tHumTempCal.H1Rh - s_tHumTempCal.H0Rh )) /
                    ( s_tHumTempOut.H1T0Out - s_tHumTempOut.H0T0Out ))
                    + s_tHumTempCal.H0Rh );

    // Truncate to specific number of decimal digits
    humidity_t = (uint16_t)(H_rh * pow(10, HUM_DECIMAL_DIGITS));
    *pfData = ((float)humidity_t) / pow(10, HUM_DECIMAL_DIGITS);

    // Prevent data going below 0% and above 100% due to linear interpolation
    if ( *pfData <   0.0f ) *pfData =   0.0f;
    if ( *pfData > 100.0f ) *pfData = 100.0f;

    return HUM_TEMP_SUCCESS;
}

//breaf: hts221_get_temp
//Param:  float *pfData
//Return: ht_status_t: it will return HUM_TEMP_SUCCESS
//                                    HUM_TEMP_ERR
//                                    HUM_TEMP_TIMTOUT
//                                    HUM_TEMP_NOT_IMPLEMENTED
ht_status_t hts221_get_temp(float* pfData)
{
    int16_t T_out, temperature_t;
    uint8_t tempReg[2] = {0, 0};
    uint8_t tmp = 0x00;
    float T_degC;

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_CTRL_REG1_ADDR, &tmp, 1) != HUM_TEMP_SUCCESS)
    {
        return HUM_TEMP_ERR;
    }

    /* Output Data Rate selection */
    tmp &= (HTS221_ODR_MASK);

    if(tmp == 0x00)
    {
        if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, &tmp, 1) != HUM_TEMP_SUCCESS)
        {
            return HUM_TEMP_ERR;
        }

        /* Serial Interface Mode selection */
        tmp &= ~(HTS221_ONE_SHOT_MASK);
        tmp |= HTS221_ONE_SHOT_START;

        if(pHTSTransmit->single_write_byte(HTS221_ADDRESS, HTS221_CTRL_REG2_ADDR, tmp) != HUM_TEMP_SUCCESS)
        {
            return HUM_TEMP_ERR;
        }

        do
        {

            if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, HTS221_STATUS_REG_ADDR, &tmp, 1) != HUM_TEMP_SUCCESS)
            {
                return HUM_TEMP_ERR;
            }

        }
        while(!(tmp & 0x01));
    }

    if(pHTSTransmit->multi_read_byte(HTS221_ADDRESS, (HTS221_TEMP_OUT_L_ADDR | HTS221_I2C_MULTIPLEBYTE_CMD),
                      &tempReg[0], 2) != HUM_TEMP_SUCCESS)
    {
        return HUM_TEMP_ERR;
    }

    T_out = ((((int16_t)tempReg[1]) << 8) + (int16_t)tempReg[0]);

    T_degC = ((float)(T_out - s_tHumTempOut.T0Out))
                     /(s_tHumTempOut.T1Out - s_tHumTempOut.T0Out)
                     *(s_tHumTempCal.T1DegC- s_tHumTempCal.T0DegC)
                     +s_tHumTempCal.T0DegC;

    temperature_t = (int16_t)(T_degC * pow(10, TEMP_DECIMAL_DIGITS));

    *pfData = ((float)temperature_t) / pow(10, TEMP_DECIMAL_DIGITS);

    return HUM_TEMP_SUCCESS;
}


//----------------------ExPort-----------------------------------------------------
ht_dev_t hts221 = {
    hts221_init,
    hts221_deinit,
    hts221_read_id,
    hts221_reboot,
    hts221_get_hum,
    hts221_get_temp,
};

/*************************End Of File(Inc)*************************************/


