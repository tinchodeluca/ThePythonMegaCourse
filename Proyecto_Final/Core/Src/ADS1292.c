/**
  ******************************************************************************
  * @file           : ads1292.c
  * @brief          : ads1292 lib functions
  ******************************************************************************
  * @attention Copyright (c) 2023  All rights reserved.
  * @author : Martín Alexandro De Luca
  * @email  : martindeluca@frba.utn.edu.ar
  ******************************************************************************
**/
#include "ADS1292.h"
#include "stm32f1xx_hal_spi.h"

static uint8_t _interrupt_flag = 0; // flag global para marcar la interrupcion
static uint8_t _initialization_finished = 0;

void ADS1292_Init(SPI_HandleTypeDef *hspi,GPIO_TypeDef *cs_port, uint16_t cs_pin, IRQn_Type IRQ){
	hspi_ads    = *hspi;
	ADS_CH_PORT = *cs_port;
	ADS_CH_PIN  = cs_pin;
	IRQ_Add     = IRQ; //EXTI0_IRQn;
}

void ADS1292_SPI_Command_Data(uint8_t data_out){
	HAL_SPI_Transmit(&hspi_ads,  &data_out, 1, HAL_MAX_DELAY);
}

void ADS1292_Write_Reg(uint8_t READ_WRITE_ADDRESS, uint8_t DATA){

	switch (READ_WRITE_ADDRESS){
		case ADS1292_CONFIG1:
			DATA = DATA & CFG1_MASK;       // 1000 0111
			break;
		case ADS1292_CONFIG2:
			DATA = DATA & CFG2_MASK_2;     // 1111 1011
			DATA |= CFG2_MASK;             // 1000 0000
			break;
		case ADS1292_LOFF:
			DATA = DATA & LOFF_MASK2;      // 1111 1101
			DATA |= LOFF_MASK;             // 0001 0000
			break;
		case ADS1292_LOFF_SENSN:
			DATA = DATA & LOFF_SENSE_MASK; // 0011 1111
			break;
		case ADS1292_LOFF_STAT:
			DATA = DATA & LOFF_STAT_MASK;  // 0101 1111
			break;
		case ADS1292_RESP1:
			DATA = DATA & RESP1_MASK;      // 0000 0010
			break;
		case ADS1292_RESP2:
			DATA = DATA & RESP2_MASK;      // 1000 0111
			DATA |= RESP2_MASK2;           // 0000 0001
			break;
		case ADS1292_GPIO:
			DATA = DATA & GPIO_MASK;       // 0000 1111
			break;
		case ADS1292_ID:
			return;
		default:
			break;
	}
	// now combine the register address and the command into one byte:
	uint8_t RADD = READ_WRITE_ADDRESS | ADS1292_WREG; // WREG = 0x40 = 0100 0000

	uint8_t buffer[3];
	buffer[0] = RADD;  //OPCODE 1 010r rrrr, where r rrrr is the starting register address.
	buffer[1] = (1)-1; //OPCODE 2 000n nnnn, where n nnnn is the number of registers to write – 1.
	buffer[2] = DATA;  //DATA to write in the reg

	HAL_SPI_Transmit(&hspi_ads, buffer, sizeof(buffer), HAL_MAX_DELAY);
}

void ADS1292_Config(ADS_REG _ADS1292){
	uint8_t REG_CONFIG2, REG_CONFIG1;
	uint8_t REG_LOFF, REG_LOFF_SENSE, REG_LOFF_STAT;
	uint8_t REG_CHSET1, REG_CHSET2;
	uint8_t REG_RESP1, REG_RESP2;
	uint8_t REG_RLD;
	uint8_t REG_GPIO;

	REG_RESP1      = _ADS1292.RESP1;
	REG_RESP2      = _ADS1292.RESP2;
	REG_RLD        = _ADS1292.RLD.CHOP | _ADS1292.RLD.LOFF_SE | _ADS1292.RLD.PDB_RLD | _ADS1292.RLD.CH1.N | _ADS1292.RLD.CH2.N | _ADS1292.RLD.CH1.P | _ADS1292.RLD.CH2.P;

	REG_LOFF_SENSE = _ADS1292.LOFF.SENSE;
	REG_LOFF_STAT  = _ADS1292.LOFF.STAT;
	REG_GPIO       = _ADS1292.GPIO;

	REG_CONFIG1    = _ADS1292.CONV_MODE | _ADS1292.SP_RATE;
	REG_LOFF       = _ADS1292.LOFF.COMP | _ADS1292.LOFF.ILEAD;
	REG_CHSET1     = _ADS1292.CH1.ENABLED | _ADS1292.CH1.GAIN | _ADS1292.CH1.MUX;
	REG_CHSET2     = _ADS1292.CH2.ENABLED | _ADS1292.CH2.GAIN | _ADS1292.CH2.MUX;
	REG_CONFIG2    = _ADS1292.PDB_LOFF_COMP | _ADS1292.PDB_REFBUF | _ADS1292.VREF | _ADS1292.CFG2_CLK | _ADS1292.TEST.ENABLED | _ADS1292.TEST.FREC;

	HAL_GPIO_WritePin(&ADS_CH_PORT, ADS_CH_PIN, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(&ADS_CH_PORT, ADS_CH_PIN, GPIO_PIN_RESET);
	HAL_Delay(500);

	ADS1292_SPI_Command_Data(ADS1292_RESET); // Reset the device
	HAL_Delay(1000);
	ADS1292_SPI_Command_Data(ADS1292_SDATAC); // Stop Read Data Continuously mode
	HAL_Delay(100);

	if (ADS_1292_ID != ADS1292_Read_Reg(ADS1292_ID)){ // TODO: Inconsistencia el ADS DEVUELVE 101001 -> CONSISTENTE CON ADS1192
//		return 0;//TODO: ERROR ADS
	}

	ADS1292_Write_Reg(ADS1292_CONFIG1, REG_CONFIG1); //Seteo 0x03: 1000 muestras por segundo
	HAL_Delay(10);

	ADS1292_Write_Reg(ADS1292_CONFIG2, REG_CONFIG2);
	HAL_Delay(10);

	ADS1292_Write_Reg(ADS1292_CH1SET, REG_CHSET1);
	HAL_Delay(10);
	ADS1292_Write_Reg(ADS1292_CH2SET, REG_CHSET2);
	HAL_Delay(10);

	ADS1292_Write_Reg(ADS1292_LOFF, REG_LOFF); //Lead-off defaults
	HAL_Delay(10);

	ADS1292_Write_Reg(ADS1292_RESP2, RESP2_CALIB);
	HAL_Delay(10);
	ADS1292_SPI_Command_Data(ADS1292_OFFCAL);
	HAL_Delay(10);

	ADS1292_Write_Reg(ADS1292_RLD_SENS, REG_RLD); //RLD settings:
	HAL_Delay(10);
	ADS1292_Write_Reg(ADS1292_LOFF_SENSN, REG_LOFF_SENSE); //LOFF settings: all disabled
	HAL_Delay(10);
	ADS1292_Write_Reg(ADS1292_LOFF_STAT, REG_LOFF_STAT); //LOFF settings: all disabled
	HAL_Delay(10);

	ADS1292_Write_Reg(ADS1292_RESP1, REG_RESP1);
	HAL_Delay(10);
	ADS1292_Write_Reg(ADS1292_RESP2, REG_RESP2);
	HAL_Delay(10);

	ADS1292_Write_Reg(ADS1292_GPIO, REG_GPIO); //LOFF settings: all disabled
	HAL_Delay(10);

	ADS1292_Read_Reg(ADS1292_CONFIG1);
	ADS1292_Read_Reg(ADS1292_CH1SET);

	ADS1292_SPI_Command_Data(ADS1292_RDATAC);
	HAL_Delay(100);
	ADS1292_SPI_Command_Data(ADS1292_START);
	HAL_Delay(100);

	HAL_NVIC_EnableIRQ(IRQ_Add); //

	_initialization_finished = 1;
}
uint8_t ADS1292_read_init_state(){
	return _initialization_finished;
}
void ADS1292_Suspend(){
	ADS1292_SPI_Command_Data(ADS1292_STANDBY);
	HAL_NVIC_DisableIRQ(IRQ_Add);
}

void ADS1292_Resume(){
	ADS1292_SPI_Command_Data(ADS1292_WAKEUP);
	HAL_NVIC_EnableIRQ(IRQ_Add); //
}
void ADS1292_PW_OFF(){
	_initialization_finished = 0;
	HAL_NVIC_DisableIRQ(IRQ_Add);

	HAL_GPIO_WritePin(&ADS_CH_PORT, ADS_CH_PIN, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(&ADS_CH_PORT, ADS_CH_PIN, GPIO_PIN_RESET);
	HAL_Delay(500);
	ADS1292_SPI_Command_Data(ADS1292_RESET); // Reset the device
	HAL_Delay(1000);
	ADS1292_SPI_Command_Data(ADS1292_SDATAC); // Stop Read Data Continuously mode
	HAL_Delay(100);
	ADS1292_SPI_Command_Data(ADS1292_STANDBY);
	HAL_Delay(100);
	ADS1292_SPI_Command_Data(ADS1292_STOP);
	HAL_Delay(100);
}
uint8_t ADS1292_Read_Reg(uint8_t ADS_RREG){
	uint8_t write_buffer[2];
	uint8_t read_buffer[4];

	write_buffer[0] = ADS1292_RREG | ADS_RREG;  // OPCODE 1 -- Reg Read Mode + Reg ADD to read
	write_buffer[1] = (1)-1;					// OPCODE 2 -- To read 1 Reg
//	write_buffer[2] = 0;

	read_buffer[0]  = 0xFF; // RDATA //TODO: METER UN VECTOR PARA COMPROBAR CUANTOS DATOS "LEE"
	read_buffer[1]  = 0xFF; // RDATA //TODO: METER UN VECTOR PARA COMPROBAR CUANTOS DATOS "LEE"
	read_buffer[2]  = 0xFF; // RDATA //TODO: METER UN VECTOR PARA COMPROBAR CUANTOS DATOS "LEE"
	read_buffer[3]  = 0xFF; // RDATA //TODO: METER UN VECTOR PARA COMPROBAR CUANTOS DATOS "LEE"

	if ( GPIO_PIN_SET == HAL_GPIO_ReadPin(&ADS_CH_PORT, ADS_CH_PIN) ){
		HAL_GPIO_WritePin(&ADS_CH_PORT, ADS_CH_PIN, GPIO_PIN_RESET);
		HAL_Delay(500);
	}

//	HAL_SPI_TransmitReceive(&hspi_ads, write_buffer, read_buffer, 3, 10);
	HAL_SPI_Transmit(&hspi_ads, write_buffer, 2, 10); //TODO: SEPARAR LAS FUNCIONES RECIVE Y TRANSMIT?
	HAL_SPI_Transmit(&hspi_ads, write_buffer, 2, 10); //TODO: SEPARAR LAS FUNCIONES RECIVE Y TRANSMIT?
	HAL_Delay(10);
	HAL_SPI_Receive(&hspi_ads, read_buffer, 4, 10); //TODO: SEPARAR LAS FUNCIONES RECIVE Y TRANSMIT?

	return read_buffer;
}

uint32_t ads_stat = 0;

ADS_DATA _READDATA;

void ADS1292_On_Interrupt(){
	uint8_t buffer[9] = {0,0,0,0,0,0,0,0,0};

	if (1 != _initialization_finished){
		HAL_NVIC_DisableIRQ(IRQ_Add);
		return;
	}else{
		_interrupt_flag = 1;

		HAL_SPI_Receive(&hspi_ads, buffer, 9, HAL_MAX_DELAY); // A Package of 9 Bytes to be read, "SATUS x3 + CHANNEL1 x3 + CHANNEL2 x3"
		_READDATA.status = 0;
		_READDATA.status = ( (buffer[0] << 16) | (buffer[1] << 8) | (buffer[2]) ) & 0x00FFFFFF; // Guardo estado
		_READDATA.ch[0]  = 0;
		_READDATA.ch[0]  = ( (buffer[3] << 16) | (buffer[4] << 8) | (buffer[5]) ) & 0x00FFFFFF; // Guardo canal 1
		_READDATA.ch[1]  = 0;
		_READDATA.ch[1]  = ( (buffer[6] << 16) | (buffer[7] << 8) | (buffer[8]) ) & 0x00FFFFFF; // Guardo canal 2
	}
}

uint8_t ADS1292_Interrupt_Flag(){
	return _interrupt_flag;
}

ADS_DATA ADS1292_Read_Data(){
	ADS_DATA _SENDDATA;
	_interrupt_flag = 0;

	for (int i = 0; i<2 ; i++){
		_SENDDATA.ch[i] = _READDATA.ch[i];
		if (_SENDDATA.ch[i] & 0x00800000){  // If its negative (in Two´s complement 24bits)
			_SENDDATA.ch[i] |= 0xFF000000;  // Complete the rest of the 32BITS
		}
	}
	return _SENDDATA;
}

ADS_REG ADS1292_Default_REG(){
	ADS_REG result;
	result.CONV_MODE     = CFG1_CONT_MODE;
	result.VREF          = CFG2_VREF_2V;
	result.CFG2_CLK      = CFG2_CLK_DIS;
	result.SP_RATE       = CFG1_DR_FMOD_DIV256;
	result.PDB_REFBUF    = CFG2_PDB_REFBUF; //CFG2_PDB_REFBUF_OFF;
	result.PDB_LOFF_COMP = CFG2_PDB_LOFF_COMP_OFF;
	result.TEST.FREC     = CFG2_TEST_FREQ_1K;
	result.GPIO          = GPIOC_OUT;
	result.RESP1         = RESP1_OFF;
	result.RESP2         = RESP2_OFF;

	result.RLD.CHOP      = RLD_OFF;
	result.RLD.PDB_RLD   = RLD_OFF;
	result.RLD.LOFF_SE   = RLD_OFF;
	result.RLD.CH1.P     = RLD_OFF;
	result.RLD.CH1.N     = RLD_OFF;
	result.RLD.CH2.P     = RLD_OFF;
	result.RLD.CH2.N     = RLD_OFF;

	result.LOFF.COMP     = LOFF_COMP_TH_95_5;
	result.LOFF.ILEAD    = LOFF_ILEAD_OFF_6NA;
	result.LOFF.FLEAD    = LOFF_FLEAD_OFF_DC;
	result.LOFF.SENSE    = LOFF_SENSE_OFF;
	result.LOFF.STAT     = LOFF_STAT_OFF;

	result.CH1.ENABLED   = CHNSET_POWER_DISABLED;
	result.CH1.GAIN      = CHNSET_GAIN_1;
	result.CH1.MUX       = CHNSET_MUXN_INPUTSHORTED;

	result.CH2.ENABLED   = CHNSET_POWER_DISABLED;
	result.CH2.GAIN      = CHNSET_GAIN_1;
	result.CH2.MUX       = CHNSET_MUXN_INPUTSHORTED;
	return result;
}
