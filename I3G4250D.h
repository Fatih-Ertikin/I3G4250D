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

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>

// Register adresses
#define I3G4250D_WHO_AM_I_ADDR           0x0F
#define I3G4250D_CTRL_REG1               0x20
#define I3G4250D_CTRL_REG2               0x21
#define I3G4250D_CTRL_REG3               0x22
#define I3G4250D_CTRL_REG4               0x23
#define I3G4250D_CTRL_REG5               0x24

#define I3G4250D_STATUS_ADDR             0x27

#define I3G4250D_OUT_X_L_ADDR            0x28
#define I3G4250D_OUT_X_H_ADDR            0x29
#define I3G4250D_OUT_Y_L_ADDR            0x2A
#define I3G4250D_OUT_Y_H_ADDR            0x2B
#define I3G4250D_OUT_Z_L_ADDR            0x2C
#define I3G4250D_OUT_Z_H_ADDR            0x2D


// Datarate
#define I3G4250D_DATARATE_100            ((uint8_t)0x00)       // 100 HZ
#define I3G4250D_DATARATE_200            ((uint8_t)0x40)       // 200 HZ
#define I3G4250D_DR_200_BW_12_5          ((uint8_t)0x80)       // 400 HZ
#define I3G4250D_DR_400_BW_25            ((uint8_t)0xC0)       // 800 HZ

// Bandwidth
#define I3G4250D_BANDWIDTH_12_5          ((uint8_t)0x00)       // 12.5 Cutoff
#define I3G4250D_BANDWIDTH_25            ((uint8_t)0x40)       // 25 Cutoff
#define I3G4250D_BANDWIDTH_50            ((uint8_t)0x80)       // 50 Cutoff
#define I3G4250D_BANDWIDTH_70            ((uint8_t)0x80)       // 70 Cutoff

// Datarate and bandwidth presets
#define I3G4250D_ODR_BW_LOW              ((uint8_t)0x10)       // 100HZ and a cutoff of 12.5
#define I3G4250D_ODR_BW_MEDIUM           ((uint8_t)0x60)       // 200HZ and a cuttoff of 50
#define I3G4250D_ODR_BW_HIGH             ((uint8_t)0xB0)       // 400HZ and a cuttoff of 110
#define I3G4250D_ODR_BW_ULTRA            ((uint8_t)0xF0)       // 800HZ and a cuttoff of 110

// High pass filter mode
#define I3G4250D_HPF_MODE_NORMAL        ((uint8_t)0x00)
#define I3G4250D_HPF_MODE_REFERENCE     ((uint8_t)0x01)
#define I3G4250D_HPF_MODE_NORMAL_2      ((uint8_t)0x02)
#define I3G4250D_HPF_MODE_AUTORESET     ((uint8_t)0x03)

// Highpass filter cutoff frequencies (See reference table 27 for more information.)
#define I3G4250D_HPCF_MODE_1             ((uint8_t)0x00)       // 100HZ > 8 | 200HZ > 15 | 400HZ > 30 | 800HZ > 56
#define I3G4250D_HPCF_MODE_2             ((uint8_t)0x10)       // 100HZ > 4 | 200HZ > 8 | 400HZ > 15 | 800HZ > 30
#define I3G4250D_HPCF_MODE_3             ((uint8_t)0x20)       // 100HZ > 2 | 200HZ > 4 | 400HZ > 8 | 800HZ > 15
#define I3G4250D_HPCF_MODE_4             ((uint8_t)0x30)       // 100HZ > 1 | 200HZ > 2 | 400HZ > 4 | 800HZ > 8
#define I3G4250D_HPCF_MODE_5             ((uint8_t)0x40)       // 100HZ > 0.5 | 200HZ > 1 | 400HZ > 2 | 800HZ > 4
#define I3G4250D_HPCF_MODE_6             ((uint8_t)0x50)       // 100HZ > 0.2 | 200HZ > 0.5 | 400HZ > 1 | 800HZ > 2
#define I3G4250D_HPCF_MODE_7             ((uint8_t)0x60)       // 100HZ > 0.1 | 200HZ > 0.2 | 400HZ > 0.5 | 800HZ > 1
#define I3G4250D_HPCF_MODE_8             ((uint8_t)0x70)       // 100HZ > 0.05 | 200HZ > 0.1 | 400HZ > 0.2 | 800HZ > 0.5
#define I3G4250D_HPCF_MODE_9             ((uint8_t)0x80)       // 100HZ > 0.02 | 200HZ > 0.05 | 400HZ > 0.1 | 800HZ > 0.2
#define I3G4250D_HPCF_MODE_10            ((uint8_t)0x90)       // 100HZ > 0.01 | 200HZ > 0.02 | 400HZ > 0.05 | 800HZ > 0.1


// Fullscale selections
#define I3G4250D_SCALE_245               ((uint8_t)0x00)       // 245 DPS
#define I3G4250D_SCALE_500               ((uint8_t)0x40)       // 500 DPS
#define I3G4250D_SCALE_2000              ((uint8_t)0x80)       // 2000 DPS
#define I3G4250D_SCALE_2000_2            ((uint8_t)0xC0)       // 2000 DPS 
/* NOTE:
The reference datasheet gives 2 options for setting the scale to 2000 DPS:
1) By setting FS1-FS0 bits of the CTRL_REG4 register to 1 and 0 respectively. This is what I3G4250D_SCALE_2000 does.
2) By setting FS1-FS0 bits of the CTRL_REG4 register to 1 and 1 respectively. This is what I3G4250D_SCALE_2000_2 does.
At the time of writing this library I am unsure why the manufacturer put 2 options for setting the scale to 2000DPS (Could not find anything in the datasheet), 
I chose to include the option anyways should anyone need it. If any developer knows more about this, please let me know by opening a issue on github.
*/

/* Sensitivity
Values are measured in MDPS/digit. (See referenced datasheet table 4. for more information)
*/
#define I3G4250D_SENSITIVTY_8_75         8.75                  // Typical value read when using 245 DPS
#define I3G4250D_SENSITIVTY_17_50        17.50                 // Typical value read when using 500 DPS
#define I3G4250D_SENSITIVTY_70           70                    // Typical value read when using 2000 DPS 

// Enabled Axis
#define I3G4250D_ENABLE_ALL_AXIS         ((uint8_t)0x0F)        // Enables all gyroscope Axis
#define I3G4250D_ENABLE_Z                ((uint8_t)0x0C)       // Enables ONLY the Z axis
#define I3G4250D_ENABLE_Y                ((uint8_t)0x0A)       // Enables ONLY the Y axis
#define I3G4250D_ENABLE_X                ((uint8_t)0x09)       // Enables ONLY the X axis

//Typedefs
typedef struct
{
    uint8_t ENABLED_AXIS;                                       // Enabled presets
    uint8_t ODR_BW_PRESET;                                      // Output datarate & Bandwidth preset
    uint8_t HPF_MODE;                                           // Highpass filter mode
    uint8_t HPCF_MODE;                                          // Highpass filter cutoff frequency
    uint8_t FULLSCALE_SELECTION;                                   
} I3G4250D_InitTypeDef;

// Accelerometer data
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} I3G4250D_DataRaw;

typedef struct
{
    float x;
    float y;
    float z;
} I3G4250D_DataScaled;

// Function prototypes
// Read IO
void I3G4250D_WriteIO(uint8_t registerAddress, uint8_t *writeData, uint8_t size);
// Write IO
void I3G4250D_ReadIO(uint8_t registerAddress, uint8_t *readData, uint8_t size);

// Initialization
void I3G4250D_Init(SPI_HandleTypeDef *accelerometerSPI, I3G4250D_InitTypeDef *accelerometerInit);

I3G4250D_DataRaw I3G4250D_GetRawData(void);
I3G4250D_DataScaled I3G4250D_GetScaledData(void);
bool I3G4250D_DataReady(uint32_t msTimeOut);

// Calibration

void I3G4250D_X_Calibrate(float x_min, float x_max);
void I3G4250D_Y_Calibrate(float y_min, float y_max);
void I3G4250D_Z_Calibrate(float z_min, float z_max);
