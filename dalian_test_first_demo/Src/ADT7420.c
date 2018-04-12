/***************************************************************************//**
 *   @file   ADT7420.c
 *   @brief  Implementation of ADT7420 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 798
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "ADT7420.h"
#include "stm32l4xx_hal.h"

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
uint8_t resolutionSetting = 0;    // Current resolution setting

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 *
 * @return registerValue - Value of the register.
*******************************************************************************/
uint8_t ADT7420_GetRegisterValue(unsigned char registerAddress)
{
    uint8_t registerValue = 0;

    registerValue = I2C_Read(ADT7420_ADDRESS, registerAddress);
  	return registerValue;
}

/***************************************************************************//**
 * @brief Sets the value of a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue - Value of the register.
 *
 * @return None.
*******************************************************************************/


/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the device is
 *        present.
 *
 * @return status - The result of the initialization procedure.
 *                  Example: 0x0 - I2C peripheral was not initialized or the
 *                                 device is not present.
 *                           0x1 - I2C peripheral was initialized and the
 *                                 device is present.
*******************************************************************************/
uint8_t ADT7420_Init(void)
{
    uint8_t status = 0;
    uint8_t test   = 0;
    
    I2C_Init();
    test = ADT7420_GetRegisterValue(ADT7420_REG_ID);
    if(test != ADT7420_DEFAULT_ID)
    {
        status = 1;
    }
    
    return status;
}

/***************************************************************************//**
 * @brief Resets the ADT7420.
 *        The ADT7420 does not respond to I2C bus commands while the default
 *        values upload (approximately 200 us).
 *
 * @return None.
*******************************************************************************/


/***************************************************************************//**
 * @brief Sets the operational mode for ADT7420.
 *
 * @param mode - Operation mode.
 *               Example: ADT7420_OP_MODE_CONT_CONV - continuous conversion;
 *                        ADT7420_OP_MODE_ONE_SHOT - one shot;
 *                        ADT7420_OP_MODE_1_SPS - 1 SPS mode;
 *                        ADT7420_OP_MODE_SHUTDOWN - shutdown.
 *
 * @return None.
*******************************************************************************/


/***************************************************************************//**
 * @brief Sets the resolution for ADT7420.
 *
 * @param resolution - Resolution.
 *                     Example: 0 - 13-bit resolution;
 *                              1 - 16-bit resolution.
 *
 * @return None.
*******************************************************************************/

/***************************************************************************//**
 * @brief Reads the temperature data and converts it to Celsius degrees.
 *
 * @return temperature - Temperature in degrees Celsius.
*******************************************************************************/
void ADT7420_GetTemperature(float* tempC)
{
    uint8_t  msbTemp = 0;
    uint8_t  lsbTemp = 0;
    uint_least16_t temp    = 0;

    msbTemp = ADT7420_GetRegisterValue(ADT7420_REG_TEMP_MSB);
    lsbTemp = ADT7420_GetRegisterValue(ADT7420_REG_TEMP_LSB);
    temp = ((uint_least16_t)msbTemp << 8) + lsbTemp;

     if(temp & 0x8000)
     {
           /*! Negative temperature */
           *tempC = (float)((signed long)temp - 65536) / 128;
     }
     else
     {
           /*! Positive temperature */
           *tempC = (float)temp / 128;
     }
}
