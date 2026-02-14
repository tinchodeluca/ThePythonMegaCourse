#include "bluetooth.h"
#include "hardware.h"

// Function to initialize the UART for communication with HC-05


void HC05_Init(UART_HandleTypeDef *uart) {
	_huart_bth = *uart;
}

void HC05_SendATCommand(const char* command) {
    // Implement UART send function here to send the AT command to HC-05
    // Make sure to handle the end of command correctly (e.g., \r\n)
}
//void HC05_GetParam(const char* command){
//	char command_string[50];
//	sprintf(command_string, "%s?", command, EOL);
//	RET_PARAM = HC05_SendATCommand(command);
//    return RET_PARAM;
//}
void HC05_SetParam(const char* command,const char* param){
	char command_string[50];
	sprintf(command_string, "%s=%s%s", command, param, EOL);
    HC05_SendATCommand(command);
}
void HC05_SetName(const char* name) {
    char command[50];
    UNUSED(command);
    HC05_SetParam(CM_NAME, name);
//    sprintf(command, "AT+NAME%s%s", name, EOL);
//    HC05_SendATCommand(command);
}
void HC05_SetPassword(const char* password) {
    char command[50];
    UNUSED(command);
    HC05_SetParam(CM_PASS, password);
//    sprintf(command, "AT+PIN%s%s", password, EOL);
//    HC05_SendATCommand(command);
}

void HC05_SetConfigMode(uint8_t enable) {
    if (enable) {
        // Set the GPIO pin to HIGH to enable configuration mode
    	HAL_GPIO_WritePin(CONFIG_MODE_GPIO_PORT, CONFIG_MODE_GPIO_PIN, GPIO_PIN_RESET);
    } else {
        // Set the GPIO pin to LOW to disable configuration mode
    	HAL_GPIO_WritePin(CONFIG_MODE_GPIO_PORT, CONFIG_MODE_GPIO_PIN, GPIO_PIN_RESET);
    }
}

void HC05_CONFIG(HC05_REG _HC05CONFIG){
	int bth_state;
	char uart_string[]     = "AT\r\n";
	//  char uart_conf_baud[]  = "AT+UART=57600,1,2\r\n"; //Two bits, even parity
	char uart_conf_baud[]  = "AT+UART=115200,0,0\r\n";
	char uart_conf_name[]  = "AT+NAME=HC05-nFisio\r\n";
	char uart_conf_reset[] = "AT+RESET\r\n";
	HAL_StatusTypeDef ret_state;
//	HAL_GPIO_WritePin(BTH_KEY_GPIO_Port, BTH_KEY_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(BTH_RESET_GPIO_Port, BTH_RESET_Pin, GPIO_PIN_RESET);

	HC05_SetConfigMode(1);

	HAL_UART_AbortReceive_IT(&_huart_bth);
//	&_huart_bth.Init.BaudRate = 38400; // TODO: ARREGLAR ESTO!!!!!!!!

	if (HAL_UART_Init(&_huart_bth) != HAL_OK)
		Error_Handler();

	bth_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);

	HC05_SetConfigMode(1);

	bth_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	UNUSED(bth_state);
	HAL_Delay(100);

	ret_state = HAL_UART_Transmit(&_huart_bth,
								 (uint8_t*)"AT+UART?\r\n",
								 strlen("AT+UART?\r\n"),
								 HAL_MAX_DELAY);
	ret_state = HAL_UART_Receive(&_huart_bth,
								 (uint8_t*) &__recv_str,
								 25,
								 2000);
	ret_state = HAL_UART_Transmit(&_huart_bth,
								 (uint8_t*)"AT+NAME?\r\n",
								 strlen("AT+NAME?\r\n"),
								 HAL_MAX_DELAY);
	ret_state = HAL_UART_Receive(&_huart_bth,(uint8_t*) &__recv_str,
								 25,
								 2000);
	ret_state = HAL_UART_Transmit(&_huart_bth,
									(uint8_t*)"AT+PSWD?\r\n",
									 strlen("AT+PSWD?\r\n"),
									 HAL_MAX_DELAY);
	ret_state = HAL_UART_Receive(&_huart_bth,(uint8_t*) &__recv_str,
									 25,
									 2000);
	ret_state = HAL_UART_Transmit(&_huart_bth,
									 (uint8_t*)"AT+ADDR?\r\n",
									 strlen("AT+ADDR?\r\n"),
									 HAL_MAX_DELAY);
	ret_state = HAL_UART_Receive(&_huart_bth,
									(uint8_t*) &__recv_str,
									 25,
									2000);
	ret_state = HAL_UART_Transmit(&_huart_bth,
									(uint8_t*)"AT+VERSION?\r\n",
									 strlen("AT+VERSION?\r\n"),
									 HAL_MAX_DELAY);
	ret_state = HAL_UART_Receive(&_huart_bth,
									(uint8_t*) &__recv_str,
									 25, 2000);
	ret_state = HAL_UART_Transmit(&_huart_bth, (uint8_t*)"AT+STATE?\r\n", strlen("AT+STATE?\r\n"), HAL_MAX_DELAY);
	ret_state = HAL_UART_Receive(&_huart_bth,(uint8_t*) &__recv_str, 25, 2000);

	ret_state = HAL_UART_Transmit(&_huart_bth, (uint8_t*)uart_string, strlen(uart_string), HAL_MAX_DELAY);
	ret_state = HAL_UART_Receive(&_huart_bth,(uint8_t*) &__recv_str, 4, 2000); //UART1 Interrupt call
	ret_state = HAL_UART_Transmit(&_huart_bth, (uint8_t*)uart_conf_baud, strlen(uart_conf_baud), HAL_MAX_DELAY);
	ret_state = HAL_UART_Receive(&_huart_bth, (uint8_t*)&__recv_str, 4, 2000);
	ret_state = HAL_UART_Transmit(&_huart_bth, (uint8_t*)uart_conf_name, strlen(uart_conf_name), HAL_MAX_DELAY);
	ret_state = HAL_UART_Receive(&_huart_bth,(uint8_t*) &__recv_str, 4, 2000);

	ret_state = HAL_UART_Transmit(&_huart_bth, (uint8_t*)uart_conf_reset, strlen(uart_conf_reset), HAL_MAX_DELAY);
	ret_state = HAL_UART_Receive(&_huart_bth, (uint8_t*)&__recv_str, 4, 2000);

	UNUSED(ret_state);

	bth_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
//	__recv_str[0] = NULL;
//	memset(__recv_str, 0, strlen(__recv_str));

	HAL_Delay(100);
//	_huart_bth.Init.BaudRate = 115200;
//	_huart_bth.Init.Parity   = UART_PARITY_EVEN;
//	_huart_bth.Init.StopBits = UART_STOPBITS_2;

	if (HAL_UART_Init(&_huart_bth) != HAL_OK){
		Error_Handler();
	}
}

HC05_REG HC05_default_reg(){
	HC05_REG result;

	strcpy(result.NAME, "nanoFISIO 1\n");
	result.BAUD   = 38400;
	result.PARITY = 0;
	result.STOP   = 1;
	result.PASS   = 1234;
	return result;
}
