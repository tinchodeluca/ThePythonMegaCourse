
#include "max3010x_sparkfun.h"

// Status Registers
static const uint8_t MAX30105_INTSTAT1 =		0x00;
static const uint8_t MAX30105_INTSTAT2 =		0x01;
static const uint8_t MAX30105_INTENABLE1 =		0x02;
static const uint8_t MAX30105_INTENABLE2 =		0x03;

// FIFO Registers
static const uint8_t MAX30105_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX30105_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX30105_FIFOREADPTR = 	0x06;
static const uint8_t MAX30105_FIFODATA =		0x07;

// Configuration Registers
static const uint8_t MAX30105_FIFOCONFIG = 		0x08;
static const uint8_t MAX30105_MODECONFIG = 		0x09;
static const uint8_t MAX30105_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30105_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30105_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30105_LED3_PULSEAMP = 	0x0E;
static const uint8_t MAX30105_LED_PROX_AMP = 	0x10;
static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30105_DIETEMPINT = 		0x1F;
static const uint8_t MAX30105_DIETEMPFRAC = 	0x20;
static const uint8_t MAX30105_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers
static const uint8_t MAX30105_PROXINTTHRESH = 	0x30;

// Part ID Registers
static const uint8_t MAX30105_REVISIONID = 		0xFE;
static const uint8_t MAX30105_PARTID = 			0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX30105_INT_A_FULL_MASK =		(uint8_t)~0b10000000;
static const uint8_t MAX30105_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX30105_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX30105_INT_DATA_RDY_MASK = (uint8_t)~0b01000000;
static const uint8_t MAX30105_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX30105_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_INT_ALC_OVF_MASK = (uint8_t)~0b00100000;
static const uint8_t MAX30105_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX30105_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX30105_INT_PROX_INT_MASK = (uint8_t)~0b00010000;
static const uint8_t MAX30105_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30105_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX30105_INT_DIE_TEMP_RDY_MASK = (uint8_t)~0b00000010;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30105_SAMPLEAVG_MASK =	(uint8_t)~0b11100000;
static const uint8_t MAX30105_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30105_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30105_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30105_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30105_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30105_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30105_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX30105_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30105_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30105_A_FULL_MASK = 	0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX30105_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX30105_SHUTDOWN = 		0x80;
static const uint8_t MAX30105_WAKEUP = 			0x00;

static const uint8_t MAX30105_RESET_MASK = 		0xBF;
static const uint8_t MAX30105_RESET = 			0x40;

static const uint8_t MAX30105_MODE_MASK = 		0xF8;
static const uint8_t MAX30105_MODE_REDONLY = 	0x02;
static const uint8_t MAX30105_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30105_MODE_MULTILED = 	0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30105_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30105_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30105_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30105_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30105_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30105_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30105_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30105_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30105_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30105_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30105_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30105_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30105_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30105_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30105_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30105_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30105_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30105_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30105_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30105_SLOT1_MASK = 		0xF8;
static const uint8_t MAX30105_SLOT2_MASK = 		0x8F;
static const uint8_t MAX30105_SLOT3_MASK = 		0xF8;
static const uint8_t MAX30105_SLOT4_MASK = 		0x8F;

static const uint8_t SLOT_NONE = 				0x00;
static const uint8_t SLOT_RED_LED = 			0x01;
static const uint8_t SLOT_IR_LED = 				0x02;
static const uint8_t SLOT_GREEN_LED = 			0x03;
static const uint8_t SLOT_NONE_PILOT = 			0x04;
static const uint8_t SLOT_RED_PILOT =			0x05;
static const uint8_t SLOT_IR_PILOT = 			0x06;
static const uint8_t SLOT_GREEN_PILOT = 		0x07;

static const uint8_t MAX_30105_EXPECTEDPARTID = 0x15;
static uint8_t _initialization_finished_MAX = 0;

I2C_HandleTypeDef *_i2cPort;
uint8_t _i2caddr;
uint8_t _interrupt_flag=0;
uint16_t last_ir_data;
uint16_t last_red_data;
uint16_t _dataAvailable=0;
void max3010x_off(I2C_HandleTypeDef *i2cPort){
	max3010x_softReset();
	begin(i2cPort, MAX30102_I2C_ADDR);
	//Setup to sense a nice looking saw tooth on the plotter
	uint8_t ledBrightness = 0; //Options: 0=Off to 255=50mA
	uint8_t sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
	uint8_t ledMode       = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
	int sampleRate        = PPG_SAMPLE_RATE; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
	int pulseWidth        = PPG_PULSE_WIDTH; //Options: 69, 118, 215, 411
	int adcRange          = PPG_ADC_RANGE; //Options: 2048, 4096, 8192, 16384
	setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}
uint8_t max3010x_init_state(){
	return _initialization_finished_MAX;
}
void max3010x_initialization(I2C_HandleTypeDef *i2cPort){
	begin(i2cPort, MAX30102_I2C_ADDR);
	//Setup to sense a nice looking saw tooth on the plotter
	uint8_t ledBrightness = PPG_LED_CURRENT; //Options: 0=Off to 255=50mA
	uint8_t sampleAverage = PPG_AVG_COUNT; //Options: 1, 2, 4, 8, 16, 32
	uint8_t ledMode       = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
	int sampleRate        = PPG_SAMPLE_RATE; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
	int pulseWidth        = PPG_PULSE_WIDTH; //Options: 69, 118, 215, 411
	int adcRange          = PPG_ADC_RANGE; //Options: 2048, 4096, 8192, 16384
	setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void begin(I2C_HandleTypeDef *i2cPort, uint8_t i2caddr){
	_i2cPort = i2cPort;
	_i2caddr = i2caddr;
}

void bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = readRegister8(_i2caddr, reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeRegister8(_i2caddr, reg, originalContents | thing);
}

uint8_t readRegister8(uint8_t address, uint8_t reg) {
	uint8_t ret=0;

	//HAL_I2C_Master_Transmit(_i2cPort, address, &reg, 1, 10);

	//HAL_I2C_Master_Receive(_i2cPort, address, &ret, 1, 10);

	HAL_StatusTypeDef stat = HAL_I2C_Mem_Read(_i2cPort, address << 1, reg, 1, &ret, 1, 10);
	UNUSED(stat);
	return ret;
}

uint8_t readRegisterMany(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t size) {

	if(_i2cPort->State != HAL_I2C_STATE_READY) return 0;

	HAL_I2C_Master_Transmit(_i2cPort, address, &reg, 1, 10);

	if(HAL_OK == HAL_I2C_Master_Receive(_i2cPort, address << 1, buffer, size, 10))
		return 1;

	return 0;
}

void writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(_i2cPort, address << 1, reg, 1, &value, 1, 10);
	UNUSED(ret);
}

void max3010x_softReset(void) {
  bitMask(MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  uint32_t startTime = HAL_GetTick();
  while (HAL_GetTick() - startTime < 100)
  {
    uint8_t response = readRegister8(_i2caddr, MAX30105_MODECONFIG);
    if ((response & MAX30105_RESET) == 0) break; //We're done!
    HAL_Delay(1); //Let's not over burden the I2C bus
  }
  return;
}

//Set sample average (Table 3, Page 18)
void setFIFOAverage(uint8_t numberOfSamples) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numberOfSamples);
}

//Enable roll over if FIFO over flows
void enableFIFORollover(void) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

//Disable roll over if FIFO over flows
void disableFIFORollover(void) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_DISABLE);
}

void setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  bitMask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

void setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

void setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

void setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
	bitMask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void setPulseAmplitudeRed(uint8_t amplitude){
	writeRegister8(_i2caddr, MAX30105_LED1_PULSEAMP, amplitude);
}

void setPulseAmplitudeIR(uint8_t amplitude) {
	writeRegister8(_i2caddr, MAX30105_LED2_PULSEAMP, amplitude);
}

//Given a slot number assign a thing to it
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
//Assigning a SLOT_RED_LED will pulse LED
//Assigning a SLOT_RED_PILOT will ??
void enableSlot(uint8_t slotNumber, uint8_t device) {

	switch (slotNumber) {
		case (1):
		  bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
		  break;
		case (2):
		  bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
		  break;
		case (3):
		  bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
		  break;
		case (4):
		  bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
		  break;
		default:
		  //Shouldn't be here!
		  break;
  }
}

//Clears all slot assignments
void disableSlots(void) {
	writeRegister8(_i2caddr, MAX30105_MULTILEDCONFIG1, 0);
	writeRegister8(_i2caddr, MAX30105_MULTILEDCONFIG2, 0);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void clearFIFO(void) {
	writeRegister8(_i2caddr, MAX30105_FIFOWRITEPTR, 0);
	writeRegister8(_i2caddr, MAX30105_FIFOOVERFLOW, 0);
	writeRegister8(_i2caddr, MAX30105_FIFOREADPTR , 0);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void setFIFOAlmostFull(uint8_t numberOfSamples) {
	bitMask(MAX30105_FIFOCONFIG, MAX30105_A_FULL_MASK, numberOfSamples);
}


//Setup the sensor
//The MAX30105 has many settings. By default we select:
// Sample Average = 4
// Mode = MultiLED
// ADC Range = 16384 (62.5pA per LSB)
// Sample rate = 50
//Use the default setup if you are just getting started with the MAX30105 sensor
void setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange) {
	max3010x_softReset(); //Reset all configuration, threshold, and data registers to POR values

	//FIFO Configuration
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	//The chip will average multiple samples of same type together if you wish
	if (sampleAverage == 1      ) setFIFOAverage(MAX30105_SAMPLEAVG_1); //No averaging per FIFO record
	else if (sampleAverage == 2 ) setFIFOAverage(MAX30105_SAMPLEAVG_2);
	else if (sampleAverage == 4 ) setFIFOAverage(MAX30105_SAMPLEAVG_4);
	else if (sampleAverage == 8 ) setFIFOAverage(MAX30105_SAMPLEAVG_8);
	else if (sampleAverage == 16) setFIFOAverage(MAX30105_SAMPLEAVG_16);
	else if (sampleAverage == 32) setFIFOAverage(MAX30105_SAMPLEAVG_32);
	else setFIFOAverage(MAX30105_SAMPLEAVG_4);

	//setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
	enableFIFORollover(); //Allow FIFO to wrap/roll over
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	//Mode Configuration
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	if (ledMode == 3) setLEDMode(MAX30105_MODE_MULTILED); //Watch all three LED channels
	else if (ledMode == 2) setLEDMode(MAX30105_MODE_REDIRONLY); //Red and IR
	else setLEDMode(MAX30105_MODE_REDONLY); //Red only
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	//Particle Sensing Configuration
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	if(adcRange < 4096        ) setADCRange(MAX30105_ADCRANGE_2048); //7.81pA per LSB
	else if(adcRange < 8192   ) setADCRange(MAX30105_ADCRANGE_4096); //15.63pA per LSB
	else if(adcRange < 16384  ) setADCRange(MAX30105_ADCRANGE_8192); //31.25pA per LSB
	else if(adcRange == 16384 ) setADCRange(MAX30105_ADCRANGE_16384); //62.5pA per LSB
	else setADCRange(MAX30105_ADCRANGE_2048);

	if (sampleRate < 100       ) setSampleRate(MAX30105_SAMPLERATE_50); //Take 50 samples per second
	else if (sampleRate < 200  ) setSampleRate(MAX30105_SAMPLERATE_100);
	else if (sampleRate < 400  ) setSampleRate(MAX30105_SAMPLERATE_200);
	else if (sampleRate < 800  ) setSampleRate(MAX30105_SAMPLERATE_400);
	else if (sampleRate < 1000 ) setSampleRate(MAX30105_SAMPLERATE_800);
	else if (sampleRate < 1600 ) setSampleRate(MAX30105_SAMPLERATE_1000);
	else if (sampleRate < 3200 ) setSampleRate(MAX30105_SAMPLERATE_1600);
	else if (sampleRate == 3200) setSampleRate(MAX30105_SAMPLERATE_3200);
	else setSampleRate(MAX30105_SAMPLERATE_50);

	//The longer the pulse width the longer range of detection you'll have
	//At 69us and 0.4mA it's about 2 inches
	//At 411us and 0.4mA it's about 6 inches
	if (pulseWidth < 118) setPulseWidth(MAX30105_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
	else if (pulseWidth < 215) setPulseWidth(MAX30105_PULSEWIDTH_118); //16 bit resolution
	else if (pulseWidth < 411) setPulseWidth(MAX30105_PULSEWIDTH_215); //17 bit resolution
	else if (pulseWidth == 411) setPulseWidth(MAX30105_PULSEWIDTH_411); //18 bit resolution
	else setPulseWidth(MAX30105_PULSEWIDTH_69);
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	//LED Pulse Amplitude Configuration
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	//Default is 0x1F which gets us 6.4mA
	//powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
	//powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
	//powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
	//powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

	setPulseAmplitudeRed(powerLevel);
	setPulseAmplitudeIR(powerLevel);

	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	//Multi-LED Mode Configuration, Enable the reading of the three LEDs
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	enableSlot(1, SLOT_RED_LED);
	if (ledMode > 1) enableSlot(2, SLOT_IR_LED);
	if (ledMode > 2) enableSlot(3, SLOT_GREEN_LED);
	//enableSlot(1, SLOT_RED_PILOT);
	//enableSlot(2, SLOT_IR_PILOT);
	//enableSlot(3, SLOT_GREEN_PILOT);
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	clearFIFO(); //Reset the FIFO before we begin checking the sensor

	// Leo los registros de las interrupciones para borrar algun bit si estuviese prendido
	readRegister8(_i2caddr, MAX30105_INTSTAT1);
	readRegister8(_i2caddr, MAX30105_INTSTAT2);

	// Activo la interrupcion para cuando tengo un dato listo
	writeRegister8(_i2caddr, MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_ENABLE);
	writeRegister8(_i2caddr, MAX30105_INTENABLE2, 0x00);
	_initialization_finished_MAX = 1;

}

void getLastReadData(uint16_t *ir_data, uint16_t *red_data){
	_dataAvailable = 0;
	*ir_data = last_ir_data;
	*red_data = last_red_data;
	return;
}

uint8_t max3010x_read_data(int16_t *ir_data, int16_t *red_data){
	uint8_t sample[6];

	if(readRegisterMany(_i2caddr, MAX30105_FIFODATA , sample, 6)){
		uint32_t _ir_sample = ((uint32_t)(sample[0] << 16) | (uint32_t)(sample[1] << 8) | (uint32_t)(sample[2])) & 0x3ffff;
		uint32_t _red_sample = ((uint32_t)(sample[3] << 16) | (uint32_t)(sample[4] << 8) | (uint32_t)(sample[5])) & 0x3ffff;

		*ir_data = (int16_t)_ir_sample;
		*red_data = (int16_t)_red_sample;
		_interrupt_flag = 0;
		return 1;
	}
	return 0;
}

void max3010x_on_interrupt()
{
	_interrupt_flag = 1;
	_dataAvailable = 1;
	max3010x_read_data(&last_ir_data, &last_red_data);
}

uint8_t max3010x_has_interrupt(){
	return _interrupt_flag;
}

uint16_t max3010x_has_data(){
	return _dataAvailable;
}
int16_t IR_AC_Max = 20;
int16_t IR_AC_Min = -20;

int16_t IR_AC_Signal_Current = 0;
int16_t IR_AC_Signal_Previous;
int16_t IR_AC_Signal_min = 0;
int16_t IR_AC_Signal_max = 0;
int16_t IR_Average_Estimated;

int16_t positiveEdge = 0;
int16_t negativeEdge = 0;
int32_t ir_avg_reg = 0;

int16_t cbuf[32];
uint8_t offset = 0;

static const uint16_t FIRCoeffs[12] = { 172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};

//  Heart Rate Monitor functions takes a sample value and the sample number
//  Returns true if a beat is detected
//  A running average of four samples is recommended for display on the screen.
uint8_t checkForBeat(int32_t sample){
	uint8_t beatDetected = 0;

  //  Save current state
  IR_AC_Signal_Previous = IR_AC_Signal_Current;

  //This is good to view for debugging
  //Serial.print("Signal_Current: ");
  //Serial.println(IR_AC_Signal_Current);

  //  Process next data sample
  IR_Average_Estimated = averageDCEstimator(&ir_avg_reg, sample);
  IR_AC_Signal_Current = lowPassFIRFilter(sample - IR_Average_Estimated);

  //  Detect positive zero crossing (rising edge)
  if ((IR_AC_Signal_Previous < 0) & (IR_AC_Signal_Current >= 0)){

    IR_AC_Max = IR_AC_Signal_max; //Adjust our AC max and min
    IR_AC_Min = IR_AC_Signal_min;

    positiveEdge = 1;
    negativeEdge = 0;
    IR_AC_Signal_max = 0;

    //if ((IR_AC_Max - IR_AC_Min) > 100 & (IR_AC_Max - IR_AC_Min) < 1000)
    if ((IR_AC_Max - IR_AC_Min) > 20 & (IR_AC_Max - IR_AC_Min) < 1000){
      //Heart beat!!!
      beatDetected = 1;
    }
  }

  //  Detect negative zero crossing (falling edge)
  if ((IR_AC_Signal_Previous > 0) & (IR_AC_Signal_Current <= 0)){
    positiveEdge = 0;
    negativeEdge = 1;
    IR_AC_Signal_min = 0;
  }

  //  Find Maximum value in positive cycle
  if (positiveEdge & (IR_AC_Signal_Current > IR_AC_Signal_Previous)){
    IR_AC_Signal_max = IR_AC_Signal_Current;
  }

  //  Find Minimum value in negative cycle
  if (negativeEdge & (IR_AC_Signal_Current < IR_AC_Signal_Previous)){
    IR_AC_Signal_min = IR_AC_Signal_Current;
  }

  return(beatDetected);
}

//  Average DC Estimator
int16_t averageDCEstimator(int32_t *p, uint16_t x){
	*p += ((((long) x << 15) - *p) >> 4);
	return (*p >> 15);
}

//  Low Pass FIR Filter
int16_t lowPassFIRFilter(int16_t din){
	cbuf[offset] = din;

	int32_t z = mul16(FIRCoeffs[11], cbuf[(offset - 11) & 0x1F]);

	for (uint8_t i = 0 ; i < 11 ; i++){
		z += mul16(FIRCoeffs[i], cbuf[(offset - i) & 0x1F] + cbuf[(offset - 22 + i) & 0x1F]);
	}

	offset++;
	offset %= 32; //Wrap condition
	return(z >> 15);
}

//  Integer multiplier
int32_t mul16(int16_t x, int16_t y){
  return((long)x * (long)y);
}

int32_t IRcw = 0;
int32_t REDcw = 0;
uint32_t IR = 0;
uint32_t RED = 0;
uint8_t redLEDCurrent = 5;
int32_t msum = 0;
int32_t mvalues[MEAN_FILTER_SIZE];
int32_t mindex = 0;
int32_t mcount = 0;

int32_t LowPassButterworthFilter(int32_t value,Filter_Data *filter_data){
	filter_data->value[0] = filter_data->value[1];

	//Fs = 100Hz and Fc = 10Hz
//	filter_data->value[1] = (2.452372752527856026e-1 * value) + (0.50952544949442879485 * filter_data->value[0]);
	//Fs = 100Hz and Fc = 4Hz
	filter_data->value[1] = (1.367287359973195227e-1 * value) + (0.72654252800536101020 * filter_data->value[0]); //Very precise butterworth filter

	return filter_data->value[0] + filter_data->value[1];
}
int32_t DCRemove(int32_t value,int32_t *cw){
	int32_t oldcw = *cw;
	*cw = value + 0.94 * *cw;
	return *cw - oldcw;
}

int32_t MeanDiff(int32_t M){
	int32_t avg = 0;

	msum -= mvalues[mindex];
	mvalues[mindex] = M;
	msum += mvalues[mindex];

	mindex++;
	mindex = mindex % MEAN_FILTER_SIZE;

	if(mcount < MEAN_FILTER_SIZE){
	  mcount++;
	}

	avg = msum / mcount;
	return avg - M;
}
