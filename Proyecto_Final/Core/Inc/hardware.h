/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


#define TIM_LED1_CH1 TIM_CHANNEL_1
#define TIM_LED1_CH2 TIM_CHANNEL_2
#define TIM_LED1_CH3 TIM_CHANNEL_4

#define TIM_LED2_CH1 TIM_CHANNEL_2
#define TIM_LED2_CH2 TIM_CHANNEL_3
#define TIM_LED2_CH3 TIM_CHANNEL_4

#define LED_2_R TIM2->CCR2
#define LED_2_G TIM2->CCR3
#define LED_2_B TIM2->CCR4

#define LED_1_R TIM3->CCR1
#define LED_1_G TIM3->CCR4
#define LED_1_B TIM3->CCR2

#define LED_1 1
#define LED_2 2

// Define the UART peripheral you are using to communicate with HC-05
#define UART_PORT USART1

// Define the GPIO pin you are using to enable/disable configuration mode
#define CONFIG_MODE_GPIO_PIN GPIO_PIN_0
#define CONFIG_MODE_GPIO_PORT GPIOA
