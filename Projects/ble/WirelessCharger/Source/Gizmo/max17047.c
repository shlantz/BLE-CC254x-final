/******************************************************************************

 @file  max17047.c

 @brief This module defines the HAL I2C API for the CC2541ST. It implements the
        I2C master.

 Group: Gizmo Eagle
 Target Device: CC2540, CC2541

 ******************************************************************************
 Release Name: max17047 for ble_sdk_1.4.2.2
 Release Date: 2018-02-06 14:57:22
 *****************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "max17047.h"


/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */



/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */


/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */
//static uint8 i2cAddr;  // Target Slave address pre-shifted up by one leaving RD/WRn LSB as zero.
static float  MAX17047_SENSE = 0.01;
static uint8 buffer[24];



bool max_ReadReg(uint8 reg, uint8 *pBuf, uint8 nBytes)  
{  
  uint8 i = 0;  
  
  /* Send address we're reading from */  
  if (HalI2CWrite(1,&reg) == 1)  
  {  
    /* Now read data */  
    i = HalI2CRead(nBytes,pBuf);  
  }  
  
  return i == nBytes;  
}  

bool max_WriteReg(uint8 reg, uint8 *pBuf, uint8 nBytes)  
{  
  uint8 i;  
  uint8 *p = buffer;  
  
  /* Copy address and data to local buffer for burst write */  
  *p++ = reg;  
  for (i = 0; i < nBytes; i++)  
  {  
    *p++ = *pBuf++;  
  }  
  nBytes++;  
  
  /* Send address and data */  
  i = HalI2CWrite(nBytes, buffer);  
  
  return (i == nBytes);  
}  


    // basic initialization
    void max17047_init(void)
    {
        uint8  uvalue[2];
        
        HalI2CInit(MAX17047_ADDR, i2cClock_533KHZ);

        // Programm Alert to SOC_AV
        max_ReadReg( MAX17047_MISC_CFG, uvalue, 2);
        uvalue[0] &= 0xfc; //0b11111100;
        uvalue[0] |= 0x01; //0b00000001;
        max_WriteReg( MAX17047_MISC_CFG, uvalue, 2);

        // MAX17047_CONFIG
        max_ReadReg( MAX17047_CONFIG, uvalue, 2);
        // Enable Alarms
        uvalue[0] |= 0x04; //0b00000100; // Aen
        uvalue[1] &= 0xf7; //0b11110111; // ALRTp

        // disable Thermistor
        uvalue[0] &= 0xef; //0b11101111; // ETHRM
        max_WriteReg( MAX17047_CONFIG, uvalue, 2);
        
        setAlarmVoltage(3300, 4200);  // setup min / max voltge
        setCapacityDesign(2200);
    };

     // datasheet says to wait ~600ms before working with the IC if POR=1
    bool getPORStatus(void)
    {
        // Check POR-Status
        uint8 value[2];
        max_ReadReg( MAX17047_STATUS, value, 2);
        if ((value[0] & 2))     return true;
        else                    return false;
    }

    void clearPORStatus(void)
    {
        // clear POR-bit
        uint8 value[2];
        //value[0] &= 0xfd; //0b11111101; // POR
        value[0] = 0xfd; //0b11111101; // POR
        max_WriteReg( MAX17047_STATUS, value, 2);
    }
    
    
    // application specific (only set on POR)
    void setCapacityDesign(const uint16 capacity_mA)
    {
        uint16 uvalue = (uint16)((float)capacity_mA * (MAX17047_SENSE * 200.0f)); // mAh * R / uVh
        uint8  value[2];
        value[0] = uvalue & 0xFF;
        value[1] = (uvalue>>8) & 0xFF;
        max_WriteReg( MAX17047_DESIGN_CAP, value, 2);
    };

    // application specific (only set on POR)
    void setFullThreshold(const uint8 percent_soc)
    {
        uint8  value[2];
        value[0] = 0;
        value[1] = (percent_soc) & 0xFF;
        max_WriteReg( MAX17047_FULL_SOC_THR, value, 2);
    };

    // application specific (only set on POR)
    void setTerminationChargeCurrent(const uint16 current_mA)
    {
        uint8  value[2];
        uint16 uvalue = (uint16)((float)(current_mA) * (MAX17047_SENSE * 640.0f));
        value[0] = uvalue & 0xFF;
        value[1] = (uvalue>>8) & 0xFF;
        max_WriteReg( MAX17047_I_CHG_TERM, value, 2);
    };

    // application specific (only set on POR)
    void setEmptyVoltage(const uint16 empty_mV, uint16 recovery_mV)
    {
        uint8  value[2];
        value[0] = (recovery_mV / 40) & 0x7f; //0b01111111;
        value[1] = (empty_mV / 20) & 0xFF;
        max_WriteReg( MAX17047_V_EMPTY, value, 2);
    };

    // alarm turns the alarm-led on
    void setAlarmVoltage(const uint16 lowVolt_mV, uint16 highVolt_mV)
    {
        uint8  value[2];
        value[0] = (lowVolt_mV / 20) & 0xFF;       // low Voltage
        value[1] = (highVolt_mV / 20) & 0xFF; // high Voltage
        max_WriteReg( MAX17047_V_ALRT_THRESHOLD, value, 2);
    };

    // do this at end-of-charge or end-of-discharge, gives back 20 byte, contains memory / INFO about cells
    void backupData(uint8 registers[])
    {
        max_ReadReg( MAX17047_FULL_CAP,       &registers[0], 2);
        max_ReadReg( MAX17047_CYCLES,         &registers[2], 2);
        max_ReadReg( MAX17047_RCOMP_0,        &registers[4], 2);
        max_ReadReg( MAX17047_TEMP_CO,        &registers[6], 2);
        max_ReadReg( MAX17047_Q_RESIDUAL_00,  &registers[8], 2);
        max_ReadReg( MAX17047_Q_RESIDUAL_10,  &registers[10], 2);
        max_ReadReg( MAX17047_Q_RESIDUAL_20,  &registers[12], 2);
        max_ReadReg( MAX17047_Q_RESIDUAL_30,  &registers[14], 2);
        max_ReadReg( MAX17047_D_QACC,         &registers[16], 2);
        max_ReadReg( MAX17047_D_PACC,         &registers[18], 2);
    };

    // after POR you have can restore registers and don't have to relearn the cell
    void restoreData(uint8 registers[])
    {
        // restore app-specific-register
        // restore gauge-learned register
        max_WriteReg( MAX17047_FULL_CAP,      &registers[0], 2);
        max_WriteReg( MAX17047_CYCLES,        &registers[2], 2);
        max_WriteReg( MAX17047_RCOMP_0,       &registers[4], 2);
        max_WriteReg( MAX17047_TEMP_CO,       &registers[6], 2);
        max_WriteReg( MAX17047_Q_RESIDUAL_00, &registers[8], 2);
        max_WriteReg( MAX17047_Q_RESIDUAL_10, &registers[10], 2);
        max_WriteReg( MAX17047_Q_RESIDUAL_20, &registers[12], 2);
        max_WriteReg( MAX17047_Q_RESIDUAL_30, &registers[14], 2);
        max_WriteReg( MAX17047_D_QACC,        &registers[16], 2);
        max_WriteReg( MAX17047_D_PACC,        &registers[18], 2);
    };

    // return measured cell voltage in mV
    uint16 getCellVoltage_mV(void)
    {
        uint8 value[2];
        max_ReadReg( MAX17047_V_CELL, value, 2);
        return (uint16)(((uint16)(value[1]<<5) + (value[0]>>3)) / 1.6); // mV
    };

    // return measured / drawn cell current in mA
    uint16 getCellCurrent_mA(void)
    {
        uint8 value[2];
        int16 temp;
        
        max_ReadReg( MAX17047_CURRENT, value, 2);
        
        temp = (int16)(value[1]<<8) + (value[0]);
        if(temp < 0) temp = ~temp + 1;
        return (uint16)((float)temp * (0.001 * (1.5625 / MAX17047_SENSE)));
        //return (uint16)((float)((uint16)(value[1]<<8) + (value[0])) * (0.001 * (1.5625 / MAX17047_SENSE)));
    };

    // return cell current in mA
    int16 getCellAverageCurrent_fmA(void)
    {
	uint8 value[2];
        int16 temp;
        
	max_ReadReg( MAX17047_AVERAGE_CURRENT, value, 2);
        
        temp = (int16)(value[1]<<8) + (value[0]);
        if(temp < 0) temp = ~temp + 1;
        
        return (int16)((float)temp * (0.001 * (1.5625 / MAX17047_SENSE)));
	//return (int16)( (float)((uint16)(value[1]<<8) + (value[0])) * (0.001 * (1.5625 / MAX17047_SENSE)));
    };
	
    // State of Charge in percent
    float getStateOfCharge_f(void)
    {
        uint8 value[2];
        max_ReadReg( MAX17047_SOC_AV, value, 2);
        return ((float)((uint16)(value[1]<<8) + (value[0])) / 256.0f);
    }

    // State of Charge in percent
    uint8 getStateOfCharge(void)
    {
        uint8 value[2];
        max_ReadReg( MAX17047_SOC_AV, value, 2);
        return (value[1]);
    }

    // returns 1 if cell is empty
    bool getEmptyStatus(void) 
    {
        // empty-Status
        uint8 value[2];
        max_ReadReg( MAX17047_F_STAT, value, 2);
        if (value[1] & 1)  return true;
        else               return false;
    }

    // gives remaining time in minutes
    float getTimeToEmpty_fmin(void)
    {
        // time to empty
        uint8 value[2];
        max_ReadReg( MAX17047_TTE, value, 2);
        return (float)((uint16)(value[1]<<8) + (value[0])) * (5.625f / 60.0f);
    }

    // gives remaining time in minutes
    uint16 getTimeToEmpty_min(void)
    {
        // time to empty
        uint8 value[2];
        max_ReadReg( MAX17047_TTE, value, 2);
        return ((uint16)(value[1]<<8) + (value[0])) / 11; // only an approximation to avoid float
    }

    // Remaining capacity in Ah
    uint16 getRemainingCapacity_mAh(void)
    {
        uint8 value[2];
        max_ReadReg( MAX17047_REM_CAP_AV, value, 2);
        float fvalue = (float)((uint16)(value[1]<<8) + (value[0])) * (0.005 / MAX17047_SENSE);
        return (uint16)fvalue;
    }


    // full capacity
    uint16 getFullCapacity_mAh(void)
    {
        uint8 value[2];
        max_ReadReg( MAX17047_FULL_CAP, value, 2);
        float fvalue = (float)((uint16)(value[1] << 8) + (value[0])) * (0.005 / MAX17047_SENSE);
        return (uint16)(fvalue);
    }

    // MAX17047_CYCLES in percent
    uint16 getChargingCycles_per(void)
    {
        uint8 value[2];
        max_ReadReg( MAX17047_CYCLES, value, 2);
        return ((uint16)(value[1] << 8) + (value[0]));
    }

    // MAX17047_AGE in percent (float)
    float getCellAge_fper(void)
    {
        uint8 value[2];
        max_ReadReg( MAX17047_AGE, value, 2);
        return (float)((uint16)(value[1] << 8) + (value[0])) / 256.0f;
    }

    // MAX17047_AGE in percent (truncated int)
    uint16 getCellAge_per(void)
    {
        uint8 value[2];
        max_ReadReg( MAX17047_AGE, value, 2);
        return (value[1]);
    }

   // MAX17047_TEMPERATURE in Celcius
    float getTemperature_fc(void)
    {
        uint8 value[2];
        max_ReadReg( MAX17047_TEMPERATURE, value, 2);
        return (float)((uint16)(value[1] << 8) + (value[0])) / 256.0f;
    }

    // MAX17047_TEMPERATURE in Celcius
    uint16 getTemperature_c(void)
    {
        uint8 value[2];
        max_ReadReg( MAX17047_TEMPERATURE, value, 2);
        return (value[1]);
    }

    // MAX17047_DESIGN_CAP in microvolt hours (mAh capacity * sense resistor = microvolt hours
    float getDesignCap_fmAh(void)
    {
        uint8 value[2];
        max_ReadReg( MAX17047_DESIGN_CAP, value, 2);
        float fvalue = (float)((uint16)(value[1]<<8) + (value[0])) * (0.005 / MAX17047_SENSE);
        return (fvalue);
    }
    
    // return the instance's sense resistor value
    float getSense(void)
    {
        return (MAX17047_SENSE);
    }
    
    // set the instance's sense resistor value
    void setSense(const float fvalue) 
    {
        MAX17047_SENSE = fvalue;
    }

/*********************************************************************
*********************************************************************/
