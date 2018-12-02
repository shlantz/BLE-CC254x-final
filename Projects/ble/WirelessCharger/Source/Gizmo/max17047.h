/******************************************************************************

 @file  max17047.h

 @brief HAL I2C API for the CC2541ST. It implements the I2C master only.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:09
 *****************************************************************************/

#ifndef MAX17047_H
#define MAX17047_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_i2c.h" 
#include "hal_sensor.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */
#define MAX17047_ADDR                   0x36
//#define MAX17047_ADDR                   0x6C

#define MAX17047_STATUS 		0x00  // Full, Empty, Normal
#define MAX17047_V_ALRT_THRESHOLD       0x01
#define MAX17047_T_ALRT_THRESHOLD       0x02
#define MAX17047_SOC_ALRT_THRESHOLD     0x03
#define MAX17047_AT_RATE	        0x04
#define MAX17047_REM_CAP_REP	        0x05  // Capacity in uVh / filtered -AV
#define MAX17047_SOC_REP	        0x06  // State of Charge in % (Highbyte) Filtered -AV
#define MAX17047_AGE		        0x07  // FULL_CAP divided by DESIGN_CAP * 100%. Also known as state of health.
#define MAX17047_TEMPERATURE            0x08
#define MAX17047_V_CELL		        0x09
#define MAX17047_CURRENT	        0x0A
#define MAX17047_AVERAGE_CURRENT        0x0B  // Average over a user-defined period
#define MAX17047_SOC_MIX	        0x0D  // State of Charge in % (Highbyte)
#define MAX17047_SOC_AV		        0x0E    // State of Charge in % (Highbyte) considering all information
#define MAX17047_REM_CAP_MIX	        0x0F    // Capacity in uVh (div Senseresistor for mAh)
#define MAX17047_FULL_CAP	        0x10    // best case full capacity in uVh
#define MAX17047_TTE		        0x11    // Time to Empty 5.625s/LSB
#define MAX17047_Q_RESIDUAL_00	        0x12
#define MAX17047_FULL_SOC_THR	        0x13
#define MAX17047_AVERAGE_TEMP	        0x16
#define MAX17047_CYCLES		        0x17    // accumulate total percent in Change in %
#define MAX17047_DESIGN_CAP	        0x18   // Application-specific input
#define MAX17047_AVERAGE_V_CELL	        0x19
#define MAX17047_MAX_MIN_TEMP	        0x1A
#define MAX17047_MAX_MIN_VOLTAGE        0x1B
#define MAX17047_MAX_MIN_CURRENT        0x1C
#define MAX17047_CONFIG		        0x1D
#define MAX17047_I_CHG_TERM	        0x1E
#define MAX17047_REM_CAP_AV	        0x1F
#define MAX17047_VERSION	        0x21
#define MAX17047_Q_RESIDUAL_10	        0x22
#define MAX17047_FULL_CAP_NOM	        0x23
#define MAX17047_TEMP_NOM	        0x24
#define MAX17047_TEMP_LIM	        0x25
#define MAX17047_AIN		        0x27
#define MAX17047_LEARN_CFG	        0x28
#define MAX17047_FILTER_CFG	        0x29
#define MAX17047_RELAX_CFG	        0x2A
#define MAX17047_MISC_CFG	        0x2B
#define MAX17047_T_GAIN		        0x2C
#define MAX17047_T_OFF		        0x2D
#define MAX17047_C_GAIN		        0x2E
#define MAX17047_C_OFF		        0x2F
#define MAX17047_Q_RESIDUAL_20	        0x32
#define MAX17047_I_AVG_EMPTY	        0x36
#define MAX17047_F_CTC		        0x37
#define MAX17047_RCOMP_0	        0x38
#define MAX17047_TEMP_CO	        0x39
#define MAX17047_V_EMPTY	        0x3A    // Empty Voltage
#define MAX17047_F_STAT		        0x3D
#define MAX17047_TIMER		        0x3E
#define MAX17047_SHDN_TIMER	        0x3F
#define MAX17047_Q_RESIDUAL_30	        0x42
#define MAX17047_D_QACC		        0x45
#define MAX17047_D_PACC		        0x46
#define MAX17047_QH		        0x4D
#define MAX17047_V_FOCV		        0xFB
#define MAX17047_SOC_VF		        0xFF    // State of Charge according to voltage fuel gauge

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */



/* ------------------------------------------------------------------------------------------------
 *                                       Global Functions
 * ------------------------------------------------------------------------------------------------
 */
void max17047_init(void);

bool getPORStatus(void);
void clearPORStatus(void);

void setCapacityDesign(const uint16 capacity_mA);
void setFullThreshold(const uint8 percent_soc);
void setTerminationChargeCurrent(const uint16 current_mA);
void setEmptyVoltage(const uint16 empty_mV, uint16 recovery_mV);
void setAlarmVoltage(const uint16 lowVolt_mV, uint16 highVolt_mV);

void backupData(uint8 registers[]);
void restoreData(uint8 registers[]);

uint16 getCellVoltage_mV(void);
uint16 getCellCurrent_mA(void);

int16 getCellAverageCurrent_fmA(void);
float getStateOfCharge_f(void);
uint8 getStateOfCharge(void);
bool getEmptyStatus(void);
float getTimeToEmpty_fmin(void);
uint16 getTimeToEmpty_min(void);
uint16 getRemainingCapacity_mAh(void);
uint16 getFullCapacity_mAh(void);
uint16 getChargingCycles_per(void);
float getCellAge_fper(void);
uint16 getCellAge_per(void);
float getTemperature_fc(void);
uint16 getTemperature_c(void);
float getDesignCap_fmAh(void);
float getSense(void);
void setSense(const float fvalue);

#endif
/**************************************************************************************************
 */
