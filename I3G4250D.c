/*
Library: Accelerometer - I3G4250D
Written by: Fatih Ertikin
Date: 02/10/2020
Description: An STM32 driver library for the I3G4250D MEMS gyroscope included in the STM32F429I-DISC1 board.
References:
    1) I3G4250D Datasheet:
    https://www.st.com/resource/en/datasheet/i3g4250d.pdf

* Copyright (C) 2020 - F. Ertikin
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.
	
   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

#include "I3G4250D.h"

//CS Pin used in my STM32F429ZI, change according to needs
#define _CS_PIN GPIO_PIN_1
#define _I3G4250D_CS_ENABLE HAL_GPIO_WritePin(GPIOC, _CS_PIN, GPIO_PIN_RESET)
#define _I3G4250D_CS_DISABLE HAL_GPIO_WritePin(GPIOC, _CS_PIN, GPIO_PIN_SET)

// SPI Handle
static SPI_HandleTypeDef SPI_Handle;

// Sensitivty
static float I3G4250D_Sensitivity = I3G4250D_SENSITIVTY_17_50;

// Bias and scaling
static float X_Bias = 0.0f;
static float Y_Bias = 0.0f;
static float Z_Bias = 0.0f;

static float X_Scale = 0.0f;
static float Y_Scale = 0.0f;
static float Z_Scale = 0.0f;

void I3G4250D_WriteIO(uint8_t registerAddress, uint8_t *writeData, uint8_t size)
{
    uint32_t timeOut = 10;
    uint8_t spiRegisterAddress = registerAddress;
    // Enable Chip Select or Slave Select (CS / SS)
    _I3G4250D_CS_ENABLE;
    // Set register values
    HAL_SPI_Transmit(&SPI_Handle, &spiRegisterAddress, 1, timeOut);
    // Transmite write data
    HAL_SPI_Transmit(&SPI_Handle, writeData, size, timeOut);
    // Disable Chip select
    _I3G4250D_CS_DISABLE;
}

void I3G4250D_ReadIO(uint8_t registerAddress, uint8_t *readData, uint8_t size)
{
    uint32_t msTimeOut = 10;
    uint8_t spiBuffer[4];
    spiBuffer[0] = registerAddress | 0x80;
    _I3G4250D_CS_ENABLE;
    HAL_SPI_Transmit(&SPI_Handle, spiBuffer, 1, msTimeOut);
    HAL_SPI_Receive(&SPI_Handle, spiBuffer, size, msTimeOut);
    _I3G4250D_CS_DISABLE;

    for (uint8_t i = 0; i < (size & 0x3); i++)
    {
        readData[i] = spiBuffer[i];
    }
}

void I3G4250D_Init(SPI_HandleTypeDef *accelerometerSPI, I3G4250D_InitTypeDef *accelerometerInit)
{
    uint8_t spiData = 0;
    memcpy(&SPI_Handle, accelerometerSPI, sizeof(*accelerometerSPI));

    //** 1. Enable all axis on the gyroscope and set output datarate and bandwidth preset**//
    spiData |= (accelerometerInit->ENABLED_AXIS & 0x0F);
    spiData |= (accelerometerInit->ODR_BW_PRESET & 0xF0);
    // set values of CTRL_REG1 of the I3G4250D
    I3G4250D_WriteIO(I3G4250D_CTRL_REG1, &spiData, 1);

    //** 2. Set the High Pass filter mode and High pass filter frequency cutoff **//
    // Reset SpiData
    spiData = 0;
    spiData |= (accelerometerInit->HPF_MODE & 0x30);
    spiData |= (accelerometerInit->HPCF_MODE & 0x0F);
    // set values of CTRL_REG2 of the I3G4250D
    I3G4250D_WriteIO(I3G4250D_CTRL_REG2, &spiData, 1);

    //** 3. Set 4-wire SPI, Full scale selection and self test mode
    spiData = 0;
    spiData |= (accelerometerInit->FULLSCALE_SELECTION & 0x30);
    // Set the self test mode to normal and the SPI mode to 4 wire.
    spiData |= (0x00 & 0x00);
    // set values of CTRL_REG4 of the I3G4250D
    I3G4250D_WriteIO(I3G4250D_CTRL_REG4, &spiData, 1);

    switch (accelerometerInit->FULLSCALE_SELECTION)
    {
    case I3G4250D_SCALE_245:
        I3G4250D_Sensitivity = I3G4250D_SENSITIVTY_8_75;
        break;

    case I3G4250D_SCALE_500:
        I3G4250D_Sensitivity = I3G4250D_SENSITIVTY_17_50;
        break;

    case I3G4250D_SCALE_2000:
        I3G4250D_Sensitivity = I3G4250D_SENSITIVTY_70;
        break;

    case I3G4250D_SCALE_2000_2:
        I3G4250D_Sensitivity = I3G4250D_SENSITIVTY_70;
        break;
    }

    // Disable Chip select
    _I3G4250D_CS_DISABLE;
}

I3G4250D_DataRaw I3G4250D_GetRawData(void)
{
    uint8_t spiBuffer[2];
    I3G4250D_DataRaw tempRawData;

    //read X axis data
    I3G4250D_ReadIO(I3G4250D_OUT_X_L_ADDR, spiBuffer, 2);
    tempRawData.x = ((spiBuffer[1] << 8) + spiBuffer[0]);

    //read Y axis data
    I3G4250D_ReadIO(I3G4250D_OUT_Y_L_ADDR, spiBuffer, 2);
    tempRawData.x = ((spiBuffer[1] << 8) + spiBuffer[0]);

    //read Z axis data
    I3G4250D_ReadIO(I3G4250D_OUT_Z_L_ADDR, spiBuffer, 2);
    tempRawData.x = ((spiBuffer[1] << 8) + spiBuffer[0]);

    return tempRawData;
}

I3G4250D_DataScaled I3G4250D_GetScaledData(void)
{
    I3G4250D_DataRaw tempRawData = I3G4250D_GetRawData();

    // Scaling and return
    // TODO: Check if these values are applicable to the I3G4250D
    I3G4250D_DataScaled scaledData;
    scaledData.x = (tempRawData.x * I3G4250D_Sensitivity * X_Scale) + 0.0f - X_Bias;
    scaledData.y = (tempRawData.x * I3G4250D_Sensitivity * Y_Scale) + 0.0f - Y_Bias;
    scaledData.z = (tempRawData.x * I3G4250D_Sensitivity * Z_Scale) + 0.0f - Z_Bias;

    return scaledData;
}

bool I3G4250D_DataReady(uint32_t msTimeOut)
{
    uint8_t Acc_status;
    uint8_t startTick = HAL_GetTick();
    do
    {
        I3G4250D_ReadIO(I3G4250D_STATUS_ADDR, &Acc_status, 1);
    } while ((Acc_status & 0x07)==0 && (HAL_GetTick() - startTick < msTimeOut));
    
    if(Acc_status & 0x07)
    {
        return true;
    }
    return false;
}

void I3G4250D_X_Calibrate(float x_min, float x_max)
{
    X_Bias = (x_max + x_min) / 2.0f;
    X_Scale = (2*1000) / (x_max - x_min);
}

void I3G4250D_Y_Calibrate(float y_min, float y_max)
{
    Y_Bias = (y_max + y_min) / 2.0f;
    Y_Scale = (2*1000) / (y_max - y_min);
}

void I3G4250D_Z_Calibrate(float z_min, float z_max)
{
    Z_Bias = (z_max + z_min) / 2.0f;
    Z_Scale = (2*1000) / (z_max - z_min);
}