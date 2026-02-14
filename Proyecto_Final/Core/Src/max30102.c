#include "max30102.h"

max30102_data _READDATA;

void max30102_init(I2C_HandleTypeDef *hi2c){
	_ui2c_max       = *hi2c;
	_interrupt_flag = 0;
}
void max30102_config(MAX30102_REG _MAX30102){
	max30102_reset();
	max30102_clear_fifo();

	max30102_set_fifo_config( max30102_smp_ave_2, 1, 7 );	// SMP_AVE , FIFO_ROLLOVER_EN ,  FIFO Almost Full Value - MAX 30102	- FIFO Configuration - REG 0x08
	max30102_set_led_pulse_width( max30102_pw_16_bit );		// 0x01	=  01 LED Pulse Width Control 	- MAX 30102 - MAX30102_SPO2_CONFIG - REG 0x0A
//	max30102_set_adc_resolution( max30102_adc_2048 );
	max30102_set_sampling_rate( max30102_sr_800);			// 0x03 = 100 SpO2 Sample Rate Control 	- MAX 30102 - MAX30102_SPO2_CONFIG - REG 0x0A
	max30102_set_led_current_1( 6.2 );						// 25.4mA - Table 8. LED Current Control - MAX 30102 - MAX30102_SPO2_CONFIG - REG 0x0C
	max30102_set_led_current_2( 6.2 );						// 25.4mA - Table 8. LED Current Control - MAX 30102 - MAX30102_SPO2_CONFIG - REG 0x0D
	max30102_set_mode( max30102_spo2 );						// 0x03	= 011 SpO2 mode

//	max30102_set_fifo_config( max30102_smp_ave_2, 1, 7 );	// SMP_AVE , FIFO_ROLLOVER_EN ,  FIFO Almost Full Value - MAX 30102	- FIFO Configuration - REG 0x08
//	max30102_set_led_pulse_width( max30102_pw_16_bit );		// 0x01	=  01 LED Pulse Width Control 	- MAX 30102 - MAX30102_SPO2_CONFIG - REG 0x0A
////	max30102_set_adc_resolution( max30102_adc_2048 );
//	max30102_set_sampling_rate( max30102_sr_200 );			// 0x03 = 100 SpO2 Sample Rate Control 	- MAX 30102 - MAX30102_SPO2_CONFIG - REG 0x0A
//	max30102_set_led_current_1( 6.2 );						// 25.4mA - Table 8. LED Current Control - MAX 30102 - MAX30102_SPO2_CONFIG - REG 0x0C
//	max30102_set_led_current_2( 6.2 );						// 25.4mA - Table 8. LED Current Control - MAX 30102 - MAX30102_SPO2_CONFIG - REG 0x0D
//	max30102_set_mode( max30102_spo2);

//	max30102_set_fifo_config( _MAX30102.SAMP_AVER, _MAX30102.FIFO_ROLLOVER, _MAX30102.FIFO_A_FULL);	// SMP_AVE , FIFO_ROLLOVER_EN ,  FIFO Almost Full Value - MAX 30102	- FIFO Configuration - REG 0x08
//	//SET SENSOR
//	max30102_set_led_pulse_width( _MAX30102.PULSE_WIDH );		// 0x01	=  01 LED Pulse Width Control 	- MAX 30102 - MAX30102_SPO2_CONFIG - REG 0x0A
//	max30102_set_adc_resolution( _MAX30102.RESOLUTION );
//	max30102_set_sampling_rate( _MAX30102.LED_RATE );			// 0x03 = 100 SpO2 Sample Rate Control 	- MAX 30102 - MAX30102_SPO2_CONFIG - REG 0x0A
//	max30102_set_led_current_1( _MAX30102.LED_IR_I);						// 25.4mA - Table 8. LED Current Control - MAX 30102 - MAX30102_SPO2_CONFIG - REG 0x0C
//	max30102_set_led_current_2( _MAX30102.LED_RED_I );						// 25.4mA - Table 8. LED Current Control - MAX 30102 - MAX30102_SPO2_CONFIG - REG 0x0D
//	// Enter MODE
//	max30102_set_mode( _MAX30102.MODE);						// 0x03	= 011 SpO2 mode					- MAX 30102 -  MAX30102_MODE_CONFIG - REG 0x09
	HAL_Delay(100);
}

void max30102_write_byte( uint8_t reg, uint8_t data){
	uint8_t payload[2];
	payload[0] = reg;
	payload[1] = data;
	HAL_StatusTypeDef ret =  HAL_I2C_Master_Transmit(&_ui2c_max, MAX30102_I2C_ADDR << 1, payload, 2, MAX30102_I2C_TIMEOUT);
	UNUSED(ret);
}

/**
 * @brief Read buflen bytes from a register of the MAX30102 and store to buffer.
 * @param obj Pointer to max30102_t object instance.
 * @param reg Register address to read from.
 * @param buf Pointer to the array to write to.
 * @param buflen Number of bytes to read.
 * @retval 1 OK; 0 ERROR
 */
uint8_t max30102_read( uint8_t reg, uint8_t *buf, uint16_t buflen){
	uint8_t reg_addr = reg;
	HAL_StatusTypeDef ret;
	if((&_ui2c_max)->State != HAL_I2C_STATE_READY)
		return 0;
	ret =  HAL_I2C_Master_Transmit(&_ui2c_max, MAX30102_I2C_ADDR << 1, &reg_addr, 1, MAX30102_I2C_TIMEOUT);
	ret =  HAL_I2C_Master_Receive( &_ui2c_max, MAX30102_I2C_ADDR << 1, buf, buflen, MAX30102_I2C_TIMEOUT);
	UNUSED(ret);
	return 1;
}

void max30102_reset(){
	uint8_t val = 0x40;
	max30102_write_byte(MAX30102_MODE_CONFIG, val);
}
/**
 * @brief Enable A_FULL interrupt.
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_a_full( uint8_t enable){
	uint8_t reg = 0;
	max30102_read(MAX30102_INTERRUPT_ENABLE_1, &reg, 1);
	reg &= ~(0x01 << MAX30102_INTERRUPT_A_FULL);
	reg |= ((enable & 0x01) << MAX30102_INTERRUPT_A_FULL);
	max30102_write_byte(MAX30102_INTERRUPT_ENABLE_1, reg);
}

/**
 * @brief Enable PPG_RDY interrupt.
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_ppg_rdy( uint8_t enable){
	uint8_t reg = 0;
	max30102_read(MAX30102_INTERRUPT_ENABLE_1, &reg, 1);
	reg &= ~(0x01 << MAX30102_INTERRUPT_PPG_RDY);
	reg |= ((enable & 0x01) << MAX30102_INTERRUPT_PPG_RDY);
	max30102_write_byte(MAX30102_INTERRUPT_ENABLE_1, reg);
}

/**
 * @brief Enable ALC_OVF interrupt.
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_alc_ovf( uint8_t enable){
	uint8_t reg = 0;
	max30102_read(MAX30102_INTERRUPT_ENABLE_1, &reg, 1);
	reg &= ~(0x01 << MAX30102_INTERRUPT_ALC_OVF);
	reg |= ((enable & 0x01) << MAX30102_INTERRUPT_ALC_OVF);
	max30102_write_byte(MAX30102_INTERRUPT_ENABLE_1, reg);
}
/**
 * @brief Enable DIE_TEMP_RDY interrupt.
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_die_temp_rdy( uint8_t enable){
	uint8_t reg = (enable & 0x01) << MAX30102_INTERRUPT_DIE_TEMP_RDY;
	max30102_write_byte(MAX30102_INTERRUPT_ENABLE_2, reg);
}
/**
 * @brief Enable temperature measurement.
 * @param obj Pointer to max30102_t object instance.
 * @param enable Enable (1) or disable (0).
 */
void max30102_set_die_temp_en( uint8_t enable){
	uint8_t reg = (enable & 0x01) << MAX30102_DIE_TEMP_EN;
	max30102_write_byte(MAX30102_DIE_TEMP_CONFIG, reg);
}
/**
 * @brief Set interrupt flag on interrupt. To be called in the corresponding external interrupt handler.
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_on_interrupt(){
	_interrupt_flag = 1;
}
/**
 * @brief Check whether the interrupt flag is active.
 * @param obj Pointer to max30102_t object instance.
 * @return uint8_t Active (1) or inactive (0).
 */

uint8_t max30102_interrupt_flag(){
	return _interrupt_flag;
}
/**
 * @brief Read interrupt status registers (0x00 and 0x01) and perform corresponding tasks.
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_interrupt_handler(){
	uint8_t reg[2] = {0x00};
	// Interrupt flag in registers 0x00 and 0x01 are cleared on read
	max30102_read(MAX30102_INTERRUPT_STATUS_1, reg, 2);

	if ((reg[0] >> MAX30102_INTERRUPT_A_FULL) & 0x01)// FIFO almost full
		max30102_read_fifo();
	if ((reg[0] >> MAX30102_INTERRUPT_PPG_RDY) & 0x01){
		max30102_read_fifo();
	}
	if ((reg[0] >> MAX30102_INTERRUPT_ALC_OVF) & 0x01){
		// Ambient light overflow
	}
	if ((reg[1] >> MAX30102_INTERRUPT_DIE_TEMP_RDY) & 0x01){// Temperature data ready
		int8_t temp_int;
		uint8_t temp_frac;
		max30102_read_temp(&temp_int, &temp_frac);
		// float temp = temp_int + 0.0625f * temp_frac;
	}
}
/**
 * @brief Shutdown the sensor.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param shdn Shutdown bit.
 */
void max30102_shutdown( uint8_t shdn){
	uint8_t config;
	max30102_read(MAX30102_MODE_CONFIG, &config, 1);
	config = (config & 0x7f) | (shdn << MAX30102_MODE_SHDN);
	max30102_write_byte(MAX30102_MODE_CONFIG, config);
}
/**
 * @brief Set measurement mode.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param mode Measurement mode enum (max30102_mode_t).
 */
void max30102_set_mode( max30102_mode_t mode){
	uint8_t config;
	max30102_read(MAX30102_MODE_CONFIG, &config, 1);
	config = (config & 0xf8) | mode;
	max30102_write_byte(MAX30102_MODE_CONFIG, config);
	max30102_clear_fifo();
}
/**
 * @brief Set sampling rate.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param sr Sampling rate enum (max30102_spo2_st_t).
 */
void max30102_set_sampling_rate( max30102_sr_t sr){
	uint8_t config;
	max30102_read(MAX30102_SPO2_CONFIG, &config, 1);
	config = (config & 0x63) << MAX30102_SPO2_SR;
	max30102_write_byte(MAX30102_SPO2_CONFIG, config);
}
/**
 * @brief Set led pulse width.
 * @param obj Pointer to max30102_t object instance.
 * @param pw Pulse width enum (max30102_led_pw_t).
 */
void max30102_set_led_pulse_width( max30102_led_pw_t pw){
	uint8_t config;
	max30102_read(MAX30102_SPO2_CONFIG, &config, 1);
	config = (config & 0x7c) | (pw << MAX30102_SPO2_LEW_PW);
	max30102_write_byte(MAX30102_SPO2_CONFIG, config);
}

/**
 * @brief Set ADC resolution.
 * @param obj Pointer to max30102_t object instance.
 * @param adc ADC resolution enum (max30102_adc_t).
 */
void max30102_set_adc_resolution( max30102_adc_t adc){
	uint8_t config;
	max30102_read(MAX30102_SPO2_CONFIG, &config, 1);
	config = (config & 0x1f) | (adc << MAX30102_SPO2_ADC_RGE);
	max30102_write_byte(MAX30102_SPO2_CONFIG, config);
}

/**
 * @brief Set LED current.
 * @param obj Pointer to max30102_t object instance.
 * @param ma LED current float (0 < ma < 51.0).
 */
void max30102_set_led_current_1( float ma){
	uint8_t pa = ma / 0.2;
	max30102_write_byte(MAX30102_LED_IR_PA1, pa);
}

/**
 * @brief Set LED current.
 * @param obj Pointer to max30102_t object instance.
 * @param ma LED current float (0 < ma < 51.0).
 */
void max30102_set_led_current_2( float ma){
	uint8_t pa = ma / 0.2;
	max30102_write_byte(MAX30102_LED_RED_PA2, pa);
}

/**
 * @brief Set slot mode when in multi-LED mode.
 * @param obj Pointer to max30102_t object instance.
 * @param slot1 Slot 1 mode enum (max30102_multi_led_ctrl_t).
 * @param slot2 Slot 2 mode enum (max30102_multi_led_ctrl_t).
 */
void max30102_set_multi_led_slot_1_2( max30102_multi_led_ctrl_t slot1, max30102_multi_led_ctrl_t slot2){
	uint8_t val = 0;
	val |= ((slot1 << MAX30102_MULTI_LED_CTRL_SLOT1) | (slot2 << MAX30102_MULTI_LED_CTRL_SLOT2));
	max30102_write_byte(MAX30102_MULTI_LED_CTRL_1, val);
}

/**
 * @brief Set slot mode when in multi-LED mode.
 * @param obj Pointer to max30102_t object instance.
 * @param slot1 Slot 1 mode enum (max30102_multi_led_ctrl_t).
 * @param slot2 Slot 2 mode enum (max30102_multi_led_ctrl_t).
 */
void max30102_set_multi_led_slot_3_4( max30102_multi_led_ctrl_t slot3, max30102_multi_led_ctrl_t slot4){
	uint8_t val = 0;
	val |= ((slot3 << MAX30102_MULTI_LED_CTRL_SLOT3) | (slot4 << MAX30102_MULTI_LED_CTRL_SLOT4));
	max30102_write_byte(MAX30102_MULTI_LED_CTRL_2, val);
}

/**
 * @brief
 * @param obj Pointer to max30102_t object instance.
 * @param smp_ave
 * @param roll_over_en Roll over enabled(1) or disabled(0).
 * @param fifo_a_full Number of empty samples when A_FULL interrupt issued (0 < fifo_a_full < 15).
 */
void max30102_set_fifo_config( max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full){
	uint8_t config = 0x00;
	config |= smp_ave << MAX30102_FIFO_CONFIG_SMP_AVE;
	config |= ((roll_over_en & 0x01) << MAX30102_FIFO_CONFIG_ROLL_OVER_EN);
	config |= ((fifo_a_full & 0x0f) << MAX30102_FIFO_CONFIG_FIFO_A_FULL);
	max30102_write_byte(MAX30102_FIFO_CONFIG, config);
}

/**
 * @brief Clear all FIFO pointers in the sensor.
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_clear_fifo(){
	uint8_t val = 0x00;
	max30102_write_byte(MAX30102_FIFO_WR_PTR, val);
	max30102_write_byte(MAX30102_FIFO_RD_PTR, val);
	max30102_write_byte(MAX30102_OVF_COUNTER, val);
}

/**
 * @brief Read FIFO content and store to buffer in max30102_t object instance.
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_read_fifo(){
	// First transaction: Get the FIFO_WR_PTR
	uint8_t wr_ptr = 0, rd_ptr = 0;
	int8_t num_samples;

	max30102_read(MAX30102_FIFO_WR_PTR, &wr_ptr, 1);
	max30102_read(MAX30102_FIFO_RD_PTR, &rd_ptr, 1);

	num_samples = (int8_t)wr_ptr - (int8_t)rd_ptr;
	if (num_samples < 1)
		num_samples += 32;
	// Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
	for (int8_t i = 0; i < num_samples; i++){
		uint8_t sample[6];
		max30102_read(MAX30102_FIFO_DATA, sample, 6);
		uint32_t ir_sample  = ((uint32_t)(sample[0] << 16) | (uint32_t)(sample[1] << 8) | (uint32_t)(sample[2])) & 0x3FFFF;
		uint32_t red_sample = ((uint32_t)(sample[3] << 16) | (uint32_t)(sample[4] << 8) | (uint32_t)(sample[5])) & 0x3FFFF;
//		obj->_ir_samples[i]  = ir_sample;
//		obj->_red_samples[i] = red_sample;
		_READDATA._ir_samples[i]  = ir_sample;
		_READDATA._red_samples[i] = red_sample;

//		max30102_plot(ir_sample, red_sample);
	}
}
max30102_data max30102_getFIFO (){
	return _READDATA;
}
/**
 * @brief Read die temperature.
 * @param temp_int Pointer to store the integer part of temperature. Stored in 2's complement format.
 * @param temp_frac Pointer to store the fractional part of temperature. Increments of 0.0625 deg C.
 */

void max30102_read_temp( int8_t *temp_int,
						uint8_t *temp_frac){
	max30102_read(MAX30102_DIE_TINT, (uint8_t *)temp_int, 1);
	max30102_read(MAX30102_DIE_TFRAC, temp_frac, 1);
}

uint8_t max30102_read_data( uint16_t *ir_data, uint16_t *red_data){
	_interrupt_flag = 0;
	uint8_t sample[6];

	if( 0 == max30102_read(MAX30102_FIFO_DATA, sample, 6) )
		return 0;
	uint32_t _ir_sample  = ((uint32_t)(sample[0] << 16) | (uint32_t)(sample[1] << 8) | (uint32_t)(sample[2])) & 0x3FFFF;
	uint32_t _red_sample = ((uint32_t)(sample[3] << 16) | (uint32_t)(sample[4] << 8) | (uint32_t)(sample[5])) & 0x3FFFF;

	*ir_data  = (uint16_t) _ir_sample;
	*red_data = (uint16_t) _red_sample;

	return 1;
}

void MAX30102_PlotIrToUART(UART_HandleTypeDef *uuart, uint16_t *samples, uint8_t sampleSize){
	char data[10];
	for(uint8_t i = 0; i< sampleSize; i++){
		samples[i] = 65536 - samples[i];
		HAL_UART_Transmit(uuart, samples, 2, HAL_MAX_DELAY);
	}
}

void MAX30102_PlotBothToUART(UART_HandleTypeDef *uuart, uint16_t *samplesRed, uint16_t *samplesIr, uint8_t sampleSize){
	char data[30];
	for(uint8_t i = 0; i< sampleSize; i++){
		sprintf(data, "red:%d\t ir:%d\n", samplesRed[i], samplesIr[i]);
		HAL_UART_Transmit(uuart, data, strlen(data), HAL_MAX_DELAY);
	}
	//sprintf(data, "red:%d \r\n ir:%d \r\n", samplesRed, samplesIr);
	//HAL_UART_Transmit(uuart, data, strlen(data), HAL_MAX_DELAY);
}

MAX30102_REG max30102_default_regs(){
	MAX30102_REG result;
	result.SAMP_AVER     = max30102_smp_ave_2;
	result.FIFO_A_FULL   = 7;
	result.FIFO_ROLLOVER = 1;
	result.LED_RATE		 = max30102_sr_1600;
	result.LED_IR_I		 = 2;//6.2;
	result.LED_RED_I	 = 2;//6.2;
	result.PULSE_WIDH	 = max30102_pw_16_bit;
	result.RESOLUTION	 = max30102_adc_2048;
	result.MODE			 = max30102_spo2;
	return result;
}
