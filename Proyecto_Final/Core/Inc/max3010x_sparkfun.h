

#ifndef UOS_DRIVERS_MAX3010X_SPARKFUN_H_
#define UOS_DRIVERS_MAX3010X_SPARKFUN_H_

#include "main.h"

#define MEAN_FILTER_SIZE        15

#define PPG_SAMPLE_RATE 3200 //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
#define PPG_PULSE_WIDTH 69  //Options: 69, 118, 215, 411
#define PPG_AVG_COUNT 2	     //Options: 1, 2, 4, 8, 16, 32
#define PPG_LED_CURRENT 0x2F //Options: 0=Off to 255=50mA
#define PPG_ADC_RANGE 4096   //Options: 2048, 4096, 8192, 16384

extern int32_t IRcw;
extern int32_t REDcw;
extern uint32_t IR;
extern uint32_t RED;

typedef struct{
   int32_t value[2];
}Filter_Data;
uint8_t max3010x_init_state();
void max3010x_initialization(I2C_HandleTypeDef *i2c);
void max3010x_off(I2C_HandleTypeDef *i2cPort);
void begin(I2C_HandleTypeDef *i2cPort, uint8_t i2caddr);
void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
uint8_t readRegister8(uint8_t address, uint8_t reg);
uint8_t readRegisterMany(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t size);
void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);
void max3010x_softReset(void);
void setFIFOAverage(uint8_t numberOfSamples);
void enableFIFORollover(void);
void disableFIFORollover(void);
void setLEDMode(uint8_t mode);
void setADCRange(uint8_t adcRange);
void setSampleRate(uint8_t sampleRate);
void setPulseWidth(uint8_t pulseWidth);
void setPulseAmplitudeRed(uint8_t amplitude);
void setPulseAmplitudeIR(uint8_t amplitude);
void enableSlot(uint8_t slotNumber, uint8_t device);
void disableSlots(void);
void clearFIFO(void);
void setFIFOAlmostFull(uint8_t numberOfSamples);
void setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange);
uint8_t max3010x_read_data(int16_t *ir_data, int16_t *red_data);
void getLastReadData(uint16_t *ir_data, uint16_t *red_data);
void max3010x_on_interrupt();
uint8_t max3010x_has_interrupt();
uint16_t max3010x_has_data();

uint8_t checkForBeat(int32_t sample);
int16_t averageDCEstimator(int32_t *p, uint16_t x);
int16_t lowPassFIRFilter(int16_t din);
int32_t mul16(int16_t x, int16_t y);

int32_t DCRemove(int32_t value,int32_t *cw);
int32_t MeanDiff(int32_t M);
int32_t LowPassButterworthFilter(int32_t value,Filter_Data *filter_data);

#endif /* UOS_DRIVERS_MAX3010X_SPARKFUN_H_ */
