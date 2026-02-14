#ifndef BLUETOOTH_H
#define BLUETOOTH_H


#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "main.h"

#define EOL "\r\n"
//#define EOL "\r" //LINUX
#define CM_ACK   "OK"
#define CM_CHECK "AT"//AT : Ceck the connection.
#define CM_VER	 "AT+VERSION"//AT+VERSION : See version
#define CM_NAME  "AT+NAME"//AT+NAME : See default name
#define CM_ADD   "AT+ADD"//AT+ADDR : see default address
#define CM_PASS  "AT+PSWD"//AT+PSWD: see default password
#define CM_BR    "AT+UART"//AT+UART : See baudrate
#define CM_ROLE  "AT+ROLE"//AT+ROLE: See role of bt module(1=master/0=slave)
#define CM_ORGL  "AT+ORGL"//AT+ORGL : Restore factory settings
#define CM_RESET "AT+RESET"//AT+RESET : Reset and exit AT mode

typedef struct HC05_REG{
	char NAME[12];
	int16_t PASS;
	int16_t BAUD;
	uint8_t PARITY;
	uint8_t STOP;
}HC05_REG;

UART_HandleTypeDef _huart_bth;
uint8_t __recv_str[30];

#define BAUDRATE 115200

void HC05_Init(UART_HandleTypeDef *uart); // Function to initialize the HC-05 module
void HC05_SendATCommand(const char* command);// Function to send AT commands to the HC-05 module
void HC05_SetName(const char* name); // Function to set the BTH name
void HC05_SetPassword(const char* password);// Function to set the BTH password
void HC05_SetConfigMode(uint8_t enable);// Function to enable/disable configuration mode using a GPIO pin
void HC05_CONFIG(HC05_REG);
HC05_REG HC05_default_reg();

#endif // BLUETOOTH_H
