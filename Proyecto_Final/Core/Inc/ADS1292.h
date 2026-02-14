/**
  ******************************************************************************
  * @file           : ads1292.h
  * @brief          : ads1292 lib definitions
  ******************************************************************************
  * @attention Copyright (c) 2023  All rights reserved.
  * @author : Martín Alexandro De Luca
  * @email  : martindeluca@frba.utn.edu.ar
  ******************************************************************************
**/
#ifndef ADS1292_H
#define ADS1292_H

#include "stdint.h"
#include "stm32f1xx_hal.h"

#define USE_ADS1292 1
/**************************************************************************
    ADS1292 Hardware Ports & SPI Handler
**************************************************************************/
SPI_HandleTypeDef hspi_ads;
GPIO_TypeDef ADS_CH_PORT;
uint16_t ADS_CH_PIN;
IRQn_Type IRQ_Add;
/**************************************************************************
    ADS1292 Commands
**************************************************************************/
typedef enum ads1292_Commands{
	// System Commands
	ADS1292_WAKEUP  = 0x02, // Wake-UP from standby mode
	ADS1292_STANDBY = 0x04, // Enter standby mode
	ADS1292_RESET   = 0x06, // Reset the device
	ADS1292_START   = 0x08, // Start/restart (synchronize) conversions
	ADS1292_STOP    = 0x0A, // Stop conversion
	ADS1292_OFFCAL  = 0X1A, // Offset channel calibration
	// Data Read Commands
	ADS1292_RDATAC  = 0x10, // Enable Read Data Continuous mode. This mode is the default mode at power-up
	ADS1292_SDATAC  = 0x11, // Stop Read Data Continuously mode
	ADS1292_RDATA   = 0x12, // Read data by command; supports multiple read back
	// Register Read Commands
	ADS1292_RREG    = 0x20, // Read  n nnnn registers starting at address r rrrr
	ADS1292_WREG    = 0x40, // Write n nnnn registers starting at address r rrrr
} ads1292_Commands;
/**************************************************************************
    ADS1292 Register MAP
**************************************************************************/
typedef enum ads1292_RegisterMap{
	// Device Settings (Read-Only Registers)
	ADS1292_ID         = 0x00, // ID Control Register (Read-Only)
	// Global Settings Across Channels
	ADS1292_CONFIG1    = 0x01, // Configuration Register 1
	ADS1292_CONFIG2    = 0x02, // Configuration Register 2
	ADS1292_LOFF       = 0x03, // Lead OFF Control Register
	// Channel-Specific Settings
	ADS1292_CH1SET     = 0x04, // Channel SET n = 1
	ADS1292_CH2SET     = 0x05, // Channel SET n = 2
	ADS1292_RLD_SENS   = 0x06, // Register Control Selecion - Right Led Drive Derivation
	ADS1292_LOFF_SENSN = 0x07, // Register Control Selecion - Lead-Off Detection
	ADS1292_LOFF_STAT  = 0x08, // Register Control Selecion - Lead-Off Detection STAT
	// GPIO and OTHER Registers
	ADS1292_RESP1      = 0x09, // Respiration Control Register 1 This register applies to the ADS1292R version only
	ADS1292_RESP2      = 0x0A, // Respiration Control Register 2 This register applies to the ADS1292R version only
	ADS1292_GPIO       = 0x0B, // General-Purpose I/O Register
} ads1292_RegisterMap;
/**************************************************************************
    ADS1292 Register Configuration Structure
    "struct ADS_REG also uses ADS_Default_REG()" >Function to populate default values
**************************************************************************/
typedef struct ADS_REG{
	uint8_t CONV_MODE;
	uint8_t VREF;
	uint8_t CFG2_CLK;
	uint8_t SP_RATE;
	uint8_t PDB_REFBUF;
	uint8_t PDB_LOFF_COMP;
	uint8_t GPIO;
	uint8_t RESP1;
	uint8_t RESP2;

	struct  ADS_TEST{
		uint8_t ENABLED;
		uint8_t FREC;
	}TEST;

	struct ADS_LOFF{
		uint8_t COMP;
		uint8_t ILEAD;
		uint8_t FLEAD;
		uint8_t SENSE;
		uint8_t STAT;
	}LOFF;
	struct ADS_RLD{
		uint8_t CHOP;
		uint8_t PDB_RLD;
		uint8_t LOFF_SE;
		struct {
				uint8_t N;
				uint8_t P;
			}CH1, CH2;
	}RLD;
	struct CHSET{
		uint8_t ENABLED;
		uint8_t GAIN;
		uint8_t MUX;
	}CH1, CH2;
}ADS_REG ;

typedef struct ADS_DATA{
	int32_t status;
	int32_t ch[2];
}ADS_DATA;
/**************************************************************************
    ADS1292 DEFINE BITS
**************************************************************************/
/* CONFIG1 */
#define ADS_1292_ID                  ((uint8_t)0x53)  // Factory ID for ADS1292
#define CFG1_SINGLE_SHOT             ((uint8_t)0x80)  // SINGLE_SHOT
#define CFG1_CONT_MODE               ((uint8_t)0x00)  // Continuous conversion mode (default)

#define CFG1_MASK                    ((uint8_t)0x87)  // MASK DEFAULT - BIT7='1' MSB -
#define CFG1_DR_FMOD_DIV1024         ((uint8_t)0x00)  // DATA RATE  fMOD/1024 125SPS
#define CFG1_DR_FMOD_DIV512          ((uint8_t)0x01)  // DATA RATE  fMOD/512  250SPS
#define CFG1_DR_FMOD_DIV256          ((uint8_t)0x02)  // DATA RATE  fMOD/256  500SPS (DEFAULT)
#define CFG1_DR_FMOD_DIV128          ((uint8_t)0x03)  // DATA RATE  fMOD/128  01kSPS
#define CFG1_DR_FMOD_DIV64           ((uint8_t)0x04)  // DATA RATE  fMOD/64   02kSPS
#define CFG1_DR_FMOD_DIV32           ((uint8_t)0x05)  // DATA RATE  fMOD/32   04kSPS
#define CFG1_DR_FMOD_DIV16           ((uint8_t)0x06)  // DATA RATE  fMOD/16   08kSPS
/* CONFIG2 */
#define CFG2_MASK                    ((uint8_t)0x80)  // MASK DEFAULT - BIT7 = '1' MSB
#define CFG2_MASK_2                  ((uint8_t)0xFB)  // MASK DEFAULT - BIT3 = '0'
#define CFG2_PDB_LOFF_COMP           ((uint8_t)0x40)  // 1 = Lead-off comparators enabled
#define CFG2_PDB_LOFF_COMP_OFF       ((uint8_t)0x00)  // 0 = Lead-off comparators disabled (default)
#define CFG2_PDB_REFBUF              ((uint8_t)0x20)  // 1 = Reference buffer is enabled
#define CFG2_PDB_REFBUF_OFF          ((uint8_t)0x00)  // 0 = Reference buffer is powered down (default)
#define CFG2_VREF_4V                 ((uint8_t)0x10)  // 4.033-V reference
#define CFG2_VREF_2V                 ((uint8_t)0x00)  // 2.42-V reference (default)
#define CFG2_CLK_EN                  ((uint8_t)0x08)  // 1 = Oscillator clock output enabled
#define CFG2_CLK_DIS                 ((uint8_t)0x00)  // 0 = Oscillator clock output disabled (default)
#define CFG2_TEST_OFF                ((uint8_t)0x00)  // AMPLITUDE SIGNAL  OFF
#define CFG2_TEST_ON                 ((uint8_t)0x02)  // AMPLITUDE SIGNAL  (VREFP - VREFN)/2.4mV
#define CFG2_TEST_FREQ_1K            ((uint8_t)0x01)  // FREQUENCY SIGNAL  PULSED AT 1KHZ
#define CFG2_TEST_FREQ_DC            ((uint8_t)0x00)  // FREQUENCY SIGNAL  AT DC
/* LOFF */
#define LOFF_MASK                    ((uint8_t)0x10)  // MASK LEAD OFF BIT4 = 1
#define LOFF_MASK2                   ((uint8_t)0xFD)  // MASK LEAD OFF BIT4 = 0
#define LOFF_COMP_TH_95_5            ((uint8_t)0x00)  // COMPARATOR THRESHOLD - POSITIVE  95%   - NEGATIVE  5% (DEFAULT)
#define LOFF_COMP_TH_925_7           ((uint8_t)0x20)  // COMPARATOR THRESHOLD - POSITIVE  92.5% - NEGATIVE  7.5%
#define LOFF_COMP_TH_90_10           ((uint8_t)0x40)  // COMPARATOR THRESHOLD - POSITIVE  90%   - NEGATIVE  10%
#define LOFF_COMP_TH_875_125         ((uint8_t)0x60)  // COMPARATOR THRESHOLD - POSITIVE  87.5% - NEGATIVE  12.5%
#define LOFF_COMP_TH_85_15           ((uint8_t)0x80)  // COMPARATOR THRESHOLD - POSITIVE  85%   - NEGATIVE  15%
#define LOFF_COMP_TH_80_20           ((uint8_t)0xA0)  // COMPARATOR THRESHOLD - POSITIVE  80%   - NEGATIVE  20%
#define LOFF_COMP_TH_75_25           ((uint8_t)0xC0)  // COMPARATOR THRESHOLD - POSITIVE  75%   - NEGATIVE  25%
#define LOFF_COMP_TH_70_30           ((uint8_t)0xE0)  // COMPARATOR THRESHOLD - POSITIVE  70%   - NEGATIVE  30%
#define LOFF_SENSE_MASK              ((uint8_t)0x3F)  // MASK LEAD OFF 0011 1111
#define LOFF_STAT_MASK               ((uint8_t)0x5F)  // MASK LEAD OFF 0101 1111
#define LOFF_SENSE_OFF               ((uint8_t)0x00)  // MASK LEAD OFF
#define LOFF_STAT_OFF                ((uint8_t)0x00)  //
#define LOFF_VLEAD_OFF_DET_CURRENT   ((uint8_t)0x00)  // CURRENT SOURCE MODE LEAD-OFF (DEFAULT)
#define LOFF_VLEAD_OFF_DET_PULL      ((uint8_t)0x10)  // PULL-UP/PULL-DOWN RESISTOR MODE LEAD-OFF
#define LOFF_ILEAD_OFF_6NA           ((uint8_t)0x00)  // MAGNITUDE OF CURRENT  6nA (DEFAULT)
#define LOFF_ILEAD_OFF_12NA          ((uint8_t)0x04)  // MAGNITUDE OF CURRENT  12nA
#define LOFF_ILEAD_OFF_18NA          ((uint8_t)0x08)  // MAGNITUDE OF CURRENT  18nA
#define LOFF_ILEAD_OFF_24NA          ((uint8_t)0x0C)  // MAGNITUDE OF CURRENT  24nA
#define LOFF_FLEAD_OFF_ALL           ((uint8_t)0x00)  // ALL BITS LOFF_SENSP AND LOFF_SENSN TURNED OFF (DEFAULT)
#define LOFF_FLEAD_OFF_AC            ((uint8_t)0x01)  // AC LEAD-OFF DETECTION AT Fdr/4
#define LOFF_FLEAD_OFF_DC            ((uint8_t)0x00)  // DC LEAD-OFF DETECTION TURNED ON
/* CHnSET */
#define CHNSET_POWER_ENABLED         ((uint8_t)0x00)  // NORMAL OPERATION
#define CHNSET_POWER_DISABLED        ((uint8_t)0x80)  // CHANNEL POWER-DOWN
#define CHNSET_GAIN_1                ((uint8_t)0x10)  // GAIN SETTING  1
#define CHNSET_GAIN_2                ((uint8_t)0x20)  // GAIN SETTING  2
#define CHNSET_GAIN_3                ((uint8_t)0x30)  // GAIN SETTING  3
#define CHNSET_GAIN_4                ((uint8_t)0x40)  // GAIN SETTING  4
#define CHNSET_GAIN_6                ((uint8_t)0x00)  // GAIN SETTING  6 (DEFAULT)
#define CHNSET_GAIN_8                ((uint8_t)0x60)  // GAIN SETTING  8
#define CHNSET_GAIN_12               ((uint8_t)0x80)  // GAIN SETTING  12
#define CHNSET_MUXN_NORMAL           ((uint8_t)0x00)  // NORMAL ELECTRODE INPUT (DEFAULT)
#define CHNSET_MUXN_INPUTSHORTED     ((uint8_t)0x01)  // INPUT SHORTED (FOR OFFSET OR NOISE MEASUREMENTS)
#define CHNSET_MUXN_USERLD           ((uint8_t)0x02)  // USED IN CONJUNCTION WITH RLD_MEAS BIT FOR RLD MEASUREMENTS
#define CHNSET_MUXN_MVDD             ((uint8_t)0x03)  // MVDD FOR SUPPLY MEASUREMENT
#define CHNSET_MUXN_TEMPERATURE      ((uint8_t)0x04)  // TEMPERATURE SENSOR
#define CHNSET_MUXN_TEST_SIGNAL      ((uint8_t)0x05)  // TEST SIGNAL
#define CHNSET_MUXN_RLD_DRP          ((uint8_t)0x06)  // RLD_DRP (POSITIVE ELECTRODE IS THE DRIVER)
#define CHNSET_MUXN_RLD_DRN          ((uint8_t)0x07)  // RLD_DRN (NEGATIVE ELECTRODE IS THE DRIVER)
/* RLD_SENSP/N */
#define RLD_CHOP_F16                 ((uint8_t)0x00)  // 00 = fMOD/ 16 CHOP[1:0]: Chop frequency
//#define RLD_CHOP_RES               ((uint8_t)0x40)  // 01 = Reserved CHOP[1:0]: Chop frequency
#define RLD_CHOP_F2                  ((uint8_t)0x80)  // 10 = fMOD / 2 CHOP[1:0]: Chop frequency
#define RLD_CHOP_F4                  ((uint8_t)0xC0)  // 11 = fMOD / 4 CHOP[1:0]: Chop frequency
#define RLD_PDB_ON                   ((uint8_t)0x20)  // PDB_RLD: RLD buffer power ON
#define RLD_LOFF_SENSE               ((uint8_t)0x10)  // RLD_LOFF_SENSE: RLD lead-off sense function
#define RLD2N_CON                    ((uint8_t)0x08)  // RLD connected to IN2N
#define RLD2P_CON                    ((uint8_t)0x04)  // RLD connected to IN2P
#define RLD1N_CON                    ((uint8_t)0x02)  // RLD connected to IN1N
#define RLD1P_CON                    ((uint8_t)0x01)  // RLD connected to IN1P
#define RLD_OFF                      ((uint8_t)0x00)  // UNSET ALL CHANNELS - RIGHT LEG DRIVE DERIVATION - DISABLED (DEFAULT)
/* GPIO */
#define GPIO_MASK                    ((uint8_t)0x0F)  // MASK LEAD OFF BIT4=1
#define GPIOC1_IN                    ((uint8_t)0x04)  // GPIO 1 IN
#define GPIOC2_IN                    ((uint8_t)0x08)  // GPIO 2 IN
#define GPIOC_OUT                    ((uint8_t)0x00)  // GPIO 1 & 2 OUT (DEFAULT)
/* PACE */
#define PACE_EVEN_CHN_2              ((uint8_t)0x00)  // CHANNEL 2 (DEFAULT)
#define PACE_ODD_CHN_1               ((uint8_t)0x00)  // CHANNEL 1 (DEFAULT)
#define PACE_DETECT_DISABLED         ((uint8_t)0x00)  // PACE DETECT BUFFER TURNED OFF (DEFAULT)
#define PACE_DETECT_ENABLED          ((uint8_t)0x01)  // PACE DETECT BUFFER TURNED ON
/* RESP */
#define RESP1_OFF                    ((uint8_t)0x02)  // RESP OFF (DEFAULT)
#define RESP1_MASK                   ((uint8_t)0x02)  // RESP MASK

#define RESP2_OFF                    ((uint8_t)0x01)  // RESP OFF (DEFAULT)
#define RESP2_CALIB                  ((uint8_t)0x80)  // RESP OFF (DEFAULT)
#define RESP2_MASK                   ((uint8_t)0x87)  // RESP MASK
#define RESP2_MASK2                  ((uint8_t)0x01)  // RESP MASK
/**************************************************************************
    ADS1292 Functions definitions
**************************************************************************/
void ADS1292_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, IRQn_Type IRQ);
uint8_t ADS1292_read_init_state();
void ADS1292_Suspend();
void ADS1292_Resume();
void ADS1292_PW_OFF();
void ADS1292_Config(ADS_REG );
void ADS1292_Write_Reg(uint8_t, uint8_t);
uint8_t ADS1292_Read_Reg(uint8_t );
void ADS1292_SPI_Command_Data(uint8_t);
void ADS1292_On_Interrupt();
ADS_REG ADS1292_Default_REG();
uint8_t ADS1292_Interrupt_Flag();
uint8_t _ADS1292_Read_Data(uint32_t *ch1, uint32_t *ch2);
ADS_DATA ADS1292_Read_Data();
int32_t ADS1292_Read_Int_Data(uint32_t *ch1, uint32_t *ch2);

#endif
