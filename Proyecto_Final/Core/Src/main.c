/* USER CODE BEGIN Header */
/*
 *  fecha: 12-2023
 *  Autor: Martín De Luca - martindeluca@frba.utn.edu.ar
 *
 */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define nanofisio
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
uint8_t recv_char;
uint8_t recv_3char[3];

uint32_t add_sn       = 0x0801F800;	//PAGINA 126// DIR DE MEMORIA FLASH Serial Number
uint32_t add_state    = 0x0801FC00;	// DIR DE MEMORIA estado del equipo
uint32_t add_bth_name = 0x0801FD00;	// DIR DE MEMORIA estado del equipo
uint32_t add_id       = 0x0801FC10;	// DIR DE MEMORIA FLASH Version del firmware

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
uint8_t fnc_debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void fnc_TEST_LED (){
	uint8_t LED = LED_1;
//	uint8_t ACT_STATE = 0;
	int8_t LOOP_TIMES = 2;
	LED_START(LED_1);
	LED_START(LED_2);

	while (LOOP_TIMES){
		LED_Color(LED_COLOR_R, LED);
		vTaskDelay(100);
		LED_Off(LED);
		vTaskDelay(100);
		LED_Color(LED_COLOR_G, LED);
		vTaskDelay(100);
		LED_Off(LED);
		vTaskDelay(100);
		LED_Color(LED_COLOR_B, LED);
		vTaskDelay(100);
		LED_Off(LED);

		if (LED_1 == LED)
			LED = LED_2;
		else
			LED = LED_1;
		LOOP_TIMES--;
	}
}
void config_MAX(){
	taskENTER_CRITICAL();
	if(0 == max3010x_init_state()){
		max3010x_initialization(&hi2c2);
	}
	taskEXIT_CRITICAL();
}
void config_ADS(){
	ADS_REG REG_CONFIG = ADS1292_Default_REG();
	taskENTER_CRITICAL();// DELIMITACIÓN DE SECCIÓN CR�?TICA
	ADS1292_Init(&hspi2, SPI_CS_GPIO_Port, SPI_CS_Pin, EXTI0_IRQn);
	REG_CONFIG.SP_RATE      = CFG1_DR_FMOD_DIV256;
	REG_CONFIG.TEST.FREC    = CFG2_TEST_FREQ_1K;
	REG_CONFIG.TEST.ENABLED = CFG2_TEST_ON;
	REG_CONFIG.PDB_REFBUF   = CFG2_PDB_REFBUF;

	REG_CONFIG.CH1.ENABLED  = CHNSET_POWER_ENABLED;
	REG_CONFIG.CH1.GAIN     = CHNSET_GAIN_1;
	REG_CONFIG.CH1.MUX      = CHNSET_MUXN_TEST_SIGNAL;

	REG_CONFIG.CH2.ENABLED  = CHNSET_POWER_ENABLED;
	REG_CONFIG.CH2.GAIN     = CHNSET_GAIN_6;
	REG_CONFIG.CH2.MUX      = CHNSET_MUXN_NORMAL;
	ADS1292_Config(REG_CONFIG);
	//ADS1292_Suspend();
	taskEXIT_CRITICAL();// FINALIZACIÓN DE LA SECCIÓN CRITICA
}

static void tsk_GET_SIGNALS (void *pvParameters){
	HAL_StatusTypeDef ret_state;
	uint8_t ACT_STATE = 0;

	DATAQUEUE_2 DATA2SEND2;
	DATAQUEUE_3 DATA2SEND3;

	OperationMode mode;
	const SystemConfig  *config_mode;
	//Var ECG
	ADS_DATA SAMPLE_ECG;
	int32_t ECG[2];
	Filter_Data ecgch1f = {0};
	int32_t ecgcw = 0;
	UNUSED(ecgch1f);
	//Var PPG
	int16_t SAMPLE_PPG[2];
	int32_t IR_REDac[2];

	Filter_Data irf  = {0};
	Filter_Data redf = {0};
	//Var TRIANG
	uint32_t test_signal[2];

	//Loop
	while(1){
		//xQueuePeek(qe_states, &ACT_STATE, portMAX_DELAY);
		/* ECG - ADS1292 24BITS
		 * Interrupción por software a 1khz
		 * TODO: FILTROS 50-100-150HZ
		 */
		if ((uxQueueMessagesWaiting(qe_tx_uart3) >= 15)||(uxQueueMessagesWaiting(qe_tx_uart2) >= 15)) {
			if (xHandleUART != NULL) {
			    xTaskNotify(xHandleUART, 0, eIncrement);
			} else {
			    // opcional: toggle LED, log, etc.
			}
		}

		if( ADS1292_Interrupt_Flag() ){
			SAMPLE_ECG = ADS1292_Read_Data();
			ECG[0] = DCRemove(SAMPLE_ECG.ch[1], &ecgcw);
			ECG[1] = SAMPLE_ECG.ch[0];
			//STATUS = SAMPLE_ECG.status;
		//	xQueueSendToFront(qe_tx_uart, &ECG, portMAX_DELAY);
		}
		/**
		 * PPG - MAX30102
		 * IR and RED channels 16bits
		 * TODO: Chequear e implementar la cola fifo incluida en el chip
		 */
		if (max3010x_read_data( &SAMPLE_PPG[0], &SAMPLE_PPG[1])){
			clearFIFO();
			IR_REDac[0] = DCRemove(SAMPLE_PPG[0],& IRcw);
			IR_REDac[1] = DCRemove(SAMPLE_PPG[1],& REDcw);

			IR_REDac[0] = LowPassButterworthFilter(IR_REDac[0], &irf);
			IR_REDac[1] = LowPassButterworthFilter(IR_REDac[1], &redf);
			IR_REDac[0] = -( IR_REDac[0] );
			IR_REDac[1] = -( IR_REDac[1] );
		//	xQueueSendToFront(qe_tx_uart, &IR_REDac, portMAX_DELAY);
		}
		/*
		 * Test triangular 101
		 * TODO: Se puede hacer algo mas cheto
		 */
		// In tsk_GET_SIGNALS task:
		static int32_t test_signal = 0;
		static int32_t step = 0x0FFF; // Adjust step size as needed

		test_signal += step;
		if (test_signal >= 0x7FFFFF || test_signal <= -0x7FFFFF) {
		    step = -step; // Reverse direction at peaks
		}
		/*
		 * TRANSMIT DATA TO BUFFER
		 */
		xQueuePeek(qe_mode, &mode, 0);
		config_mode = &mode_config[mode];
		if (config_mode->channels == 2){
			if (config_mode->ecg_enabled == true){
				switch (config_mode->ecg_mode ){
					case 3:
						DATA2SEND2.ch2 = ECG[1];
						DATA2SEND2.ch1 = ECG[0];
						break;
					case 1:
						DATA2SEND2.ch1 = ECG[1];
						break;
					case 0:
						DATA2SEND2.ch1 = ECG[1];
						break;
				}
			}
			if (config_mode->ppg1_enabled == true)
				DATA2SEND2.ch2 = IR_REDac[0];
			if (config_mode->ppg2_enabled == true)
				DATA2SEND2.ch2 = IR_REDac[1];
			if (config_mode->test_signal_enabled == true)
				DATA2SEND2.ch2 = test_signal;
			xQueueSendToBack(qe_tx_uart2, &DATA2SEND2, portMAX_DELAY);
		}
		if (config_mode->channels == 3){
			if (config_mode->ecg_enabled == true){
				switch (config_mode->ecg_mode ){
					case 1:
						DATA2SEND3.ch1 = ECG[1];
						break;
					case 0:
						DATA2SEND3.ch1 = ECG[0];
						break;
				}
			}
			if (config_mode->ppg1_enabled == true)
				DATA2SEND3.ch2 = IR_REDac[0];
			if (config_mode->ppg2_enabled == true)
				DATA2SEND3.ch3 = IR_REDac[1];
			xQueueSendToBack(qe_tx_uart3, &DATA2SEND3, portMAX_DELAY);
		}

	}
	vTaskDelete( NULL ); //SAFETY
}
typedef struct __attribute__((packed)) {
	int32_t sync;
	int32_t ch1;
	int32_t ch2;
} TX_FRAME_2CH;

void wrap_dataqueue2(TX_FRAME_2CH *dest, const DATAQUEUE_2 *src) {
	dest->sync = 0x7FFFFFFF;
	dest->ch1  = src->ch1;
	dest->ch2  = src->ch2;
}

static void tsk_UART_TX (void *pvParameters){
	HAL_StatusTypeDef ret_state;
	uint8_t ACT_STATE = 0;
	UBaseType_t count;

    // Buffer para almacenar hasta 15 elementos antes de enviar
    DATAQUEUE_3 buffer_3[15];  // Para el modo de 3 canales
    DATAQUEUE_2 buffer_2[15];  // Para el modo de 2 canales

    TX_FRAME_2CH tx_buffer[15];

	OperationMode current_mode;
	const SystemConfig  *config_mode;
	uint32_t ulNotificationValue;
	BaseType_t xResult;

	static uint32_t notifyCount = 0;

	while(1){
		xResult = xTaskNotifyWait(
			    0,              // Do not clear bits on entry
			    0xFFFFFFFF,     // Clear all bits on exit (equivalent to ULONG_MAX)
			    &ulNotificationValue,
			    portMAX_DELAY
			);
		notifyCount++;
		xQueuePeek(qe_mode, &current_mode, 0);
		config_mode = &mode_config[current_mode];
		if (config_mode->channels == 3) {
			// Extraer 15 elementos de la cola
			for (int i = 0; i < 15; i++) {
				xQueueReceive(qe_tx_uart3, &buffer_3[i], portMAX_DELAY);
				//xQueueReset(qe_tx_uart3);
			}
			// Enviar los 15 paquetes de una sola vez (15 * 12 bytes)
			//ret_state = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buffer_3, sizeof(DATAQUEUE_3));
		}
		else {
			configASSERT(uxQueueMessagesWaiting(qe_tx_uart2) >= 15);
			for (int i = 0; i < 15; i++) {
				xQueueReceive(qe_tx_uart2, &buffer_2[i], portMAX_DELAY);
				wrap_dataqueue2(&tx_buffer[i], &buffer_2[i]);
			}
			//xQueueReset(qe_tx_uart2);
			//xQueueReceive(qe_tx_uart2, &buffer_2[0], portMAX_DELAY);
/*
			if (huart1.gState == HAL_UART_STATE_READY) {
			    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)tx_buffer, sizeof(tx_buffer));
			}*/
			ret_state = HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_buffer, sizeof(tx_buffer));
		}
	}
	vTaskDelete( NULL ); //SAFETY
}

static void tsk_MAIN (void *pvParameters){
	HAL_StatusTypeDef ret_state;
	BaseType_t ret_task = pdFALSE;
	static OperationMode prev_mode = STOP,
						 ACT_STATE;

	LED_START(LED_2);
	fnc_TEST_LED ();

	while (1){
		vTaskDelay(50);
		xQueuePeek(qe_mode, &ACT_STATE, 0);
		if( fnc_debounce(BTN_GPIO_Port, BTN_Pin) ){
			ACT_STATE = (ACT_STATE == DBG03) ? STOP : ACT_STATE + 1;
			xQueueOverwrite(qe_mode, &ACT_STATE);
		}
	    if(ACT_STATE != prev_mode){
			taskENTER_CRITICAL();
			xQueueReset(qe_tx_uart2);
			xQueueReset(qe_tx_uart3);
			if (STOP == ACT_STATE){
				LED_Off(LED_2);
				ADS1292_Suspend();
				vTaskSuspend(xHandleSIGNALS);
				vTaskSuspend(xHandleUART);
			}
			else{
				if (STOP == prev_mode){
					ADS1292_Resume();
					vTaskResume(xHandleUART);
					vTaskResume(xHandleSIGNALS);
				}
				if (5 <= ACT_STATE)
					LED_Color(LED_WHITE, LED_2);
				if (1 == ACT_STATE)
					LED_Color(LED_COLOR_B, LED_2);
				if (2 == ACT_STATE)
					LED_Color(LED_COLOR_G, LED_2);
				if (3 == ACT_STATE)
					LED_Color(LED_COLOR_R, LED_2);
				if (4 == ACT_STATE)
					LED_Color(LED_COLOR_GB, LED_2);
			}
			prev_mode = ACT_STATE;
			taskEXIT_CRITICAL();
	        //vTaskDelay(pdMS_TO_TICKS(10)); // Tiempo de estabilización
	    }
	}
	vTaskDelete( NULL ); //SAFETY
}

static void tsk_BAT (void *pvParameters){
	LED_START(LED_1);
	HAL_ADCEx_Calibration_Start(&hadc1);
	int BAT_LVL = 0;

	HAL_GPIO_WritePin(BAT_SENS_GPIO_Port, BAT_SENS_Pin, GPIO_PIN_SET);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 500);
	BAT_LVL = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	HAL_GPIO_WritePin(BAT_SENS_GPIO_Port, BAT_SENS_Pin, GPIO_PIN_RESET);
	if (BAT_LVL >= 2000)
		LED_Color(LED_STANDBY, LED_1);
	else
		LED_Color(LED_LOWPOW, LED_1);

	while (1){
		vTaskDelay((1000 / portTICK_PERIOD_MS)*2);

		HAL_GPIO_WritePin(BAT_SENS_GPIO_Port, BAT_SENS_Pin, GPIO_PIN_SET);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 500);
		BAT_LVL = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		HAL_GPIO_WritePin(BAT_SENS_GPIO_Port, BAT_SENS_Pin, GPIO_PIN_RESET);
		if (BAT_LVL <= LOWBAT)
			LED_Toggle_Color(LED_LOWPOW, LED_1);
		else if (BAT_LVL <= MEDBAT)
			LED_Toggle_Color(LED_MEDPOW, LED_1);
		else
			LED_Toggle_Color(LED_FULLPOW, LED_1);
	}
	vTaskDelete( NULL ); //SAFETY
}

void handle_uart_command(const char *cmd) {
    OperationMode new_mode;

    taskENTER_CRITICAL();
    if(strcmp(cmd, "MODE1") == 0)      new_mode = MODE1;
    else if(strcmp(cmd, "MODE2") == 0) new_mode = MODE2;
    else if(strcmp(cmd, "MODE3") == 0) new_mode = MODE3;
    else if(strcmp(cmd, "MODE4") == 0) new_mode = MODE4;
    else if(strcmp(cmd, "DBG01") == 0) new_mode = DBG01;
    else if(strcmp(cmd, "DBG02") == 0) new_mode = DBG02;
    else if(strcmp(cmd, "DBG03") == 0) new_mode = DBG03;
    else if(strcmp(cmd, "STOP" ) == 0) new_mode = STOP;
    else if(strcmp(cmd, "RESET") == 0) new_mode = RST;
    else {
     //TODO:   send_uart_response("ERR: Invalid command");
        return;
    }
    if(new_mode == RST) new_mode = MODE1;

    xQueueSend(qe_mode,&new_mode,0);

//TODO:   apply_operation_mode(new_mode);
 //   send_uart_response("OK: Mode set");
    taskEXIT_CRITICAL();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	ADS1292_On_Interrupt();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    static char rx_buf[5];
    static uint8_t idx = 0;

	if( USART1 == huart->Instance){
		HAL_StatusTypeDef ret_hal_int;
		BaseType_t pxHigherPriorityTaskWoken = pdFALSE, ret_queue;

        rx_buf[idx++] = recv_char;

        if(5 == idx){
            uart_cmd_t new_cmd;
            memcpy(new_cmd.cmd, rx_buf, 5);
            new_cmd.cmd[5] = '\0';
            ret_queue = xQueueSendFromISR(qe_rx_uart, &new_cmd, pxHigherPriorityTaskWoken);
            idx = 0;
        }
		ret_hal_int = HAL_UART_Receive_IT(&huart1, &recv_char, 1); //UART1 Interrupt call
		portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
    	LED_Toggle_Color(LED_ERROR, LED_2);
    }
}


uint8_t fnc_debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	if ( !HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) ){
		HAL_Delay(80);
		if( !HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) ){
			while(!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin));
			return 1;
		}
	}
	return 0;
}

void stop_all_peripherals(){
	ADS1292_PW_OFF();
}
/*
void apply_operation_mode(OperationMode mode) {
    static OperationMode prev_mode = STOP;

    // Detener periféricos si hubo cambio de modo
    if(mode != prev_mode){
        stop_all_peripherals();
        vTaskDelay(pdMS_TO_TICKS(10)); // Tiempo de estabilización
    }

    const SystemConfig *cfg = &mode_config[mode];

    // Configurar ECG
    if(cfg->ecg_enabled){
        if(cfg->ecg_mode == 0)
        	init_ecg_real(cfg->test_freq);
        else
        	init_ecg_test_square(cfg->test_freq);
    }

    // Configurar PPG1
    if(cfg->ppg1_enabled) {
        if(cfg->ppg1_mode == 0)
        	init_ppg1_real();
        else
        	init_ppg1_test_triangular(cfg->test_freq);
    }

    // Configurar PPG2 (siempre real)
    if(cfg->ppg2_enabled)
    	init_ppg2_real();

    // Configurar señal de test
    if(cfg->test_signal_enabled) {
        init_test_generator(TRIANGULAR, cfg->test_freq);
    }
    prev_mode = mode;
}*/

//
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  //qe_states   = xQueueCreate(1, sizeof(STATE));
  qe_mode     = xQueueCreate(1, sizeof(OperationMode));
  qe_tx_uart3 = xQueueCreate(25, sizeof(DATAQUEUE_3)); //3x32bits
  qe_tx_uart2 = xQueueCreate(25, sizeof(DATAQUEUE_2)); //2x32bits
  configASSERT(qe_tx_uart2 != NULL); // Aborta si falla
  configASSERT(qe_tx_uart3 != NULL); // Aborta si falla

//  qe_tx_uart = xQueueCreate(1, sizeof(int32_t)); //2x16bits
//  qe_rx_uart = xQueueCreate(1, sizeof(uint16_t));

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_PCD_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  INIT_LEDS(htim3, htim2);
  config_MAX();
  config_ADS();

  /* USER CODE BEGIN SysInit */

  static OperationMode INITMODE = MODE1;//DBG01;
  xQueueSend(qe_mode,&INITMODE,0);

  BaseType_t ret_task = pdFALSE;

  ret_task = xTaskCreate(tsk_MAIN,
						"",
						configMINIMAL_STACK_SIZE,
						NULL,
						tskIDLE_PRIORITY + 3,
						NULL);

  ret_task = xTaskCreate(tsk_BAT,
						"",
						configMINIMAL_STACK_SIZE,
						NULL,
						tskIDLE_PRIORITY + 3,
						NULL);

  ret_task = xTaskCreate(tsk_UART_TX,
						"",
						configMINIMAL_STACK_SIZE,
						NULL,
						tskIDLE_PRIORITY + 3,
						&xHandleUART);


  ret_task = xTaskCreate(tsk_GET_SIGNALS,
						"",
						configMINIMAL_STACK_SIZE,
						NULL,
						tskIDLE_PRIORITY + 3,
						&xHandleSIGNALS);

  vTaskSuspend(xHandleSIGNALS);
  vTaskSuspend(xHandleUART);

  if (pdFALSE == ret_task){
//	  TODO: error
  }
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  /*const char msg[] = "DMA WORKS\r\n";

	  if (huart1.gState == HAL_UART_STATE_READY) {
	      HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg, sizeof(msg)-1);
	  }*/
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_PILL_GPIO_Port, LED_PILL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BAT_SENS_Pin|BTH_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_PILL_Pin */
  GPIO_InitStruct.Pin = LED_PILL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PILL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_Pin BTH_STATE_Pin */
  GPIO_InitStruct.Pin = BTN_Pin|BTH_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADS_DRDY_Pin */
  GPIO_InitStruct.Pin = ADS_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADS_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_SENS_Pin */
  GPIO_InitStruct.Pin = BAT_SENS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BAT_SENS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTH_KEY_Pin */
  GPIO_InitStruct.Pin = BTH_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BTH_KEY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  LED_Toggle_Color(LED_ERROR, LED_2);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
