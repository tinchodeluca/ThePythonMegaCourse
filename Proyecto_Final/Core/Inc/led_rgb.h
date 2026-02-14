#include "hardware.h"

#include <stdint.h> // Include the necessary header for uint8_t

#include "FreeRTOS.h"
#include "task.h"

TIM_HandleTypeDef HTIM_LED1;
TIM_HandleTypeDef HTIM_LED2;

//typedef struct {
//	TIM_HandleTypeDef HTIM_LED;
//	uint8_t red;
//	uint8_t green;
//	uint8_t blue;
//} LedColor;

#define LED_MAX_POWER 99
#define LED_min_POWER 5

void INIT_LEDS (TIM_HandleTypeDef HTIM_1, TIM_HandleTypeDef HTIM_2);
int GET_SIGN(int number);
// Function prototypes

void LED_Color(const uint8_t *color, uint8_t led);
void LED_Toggle_Color(const uint8_t *color, uint8_t led);
void LED_Off(uint8_t led);
void LED_START(uint8_t led);
void LED_STOP(uint8_t led);
void LED_BLINK(uint8_t TOn, uint8_t TOff, uint8_t Cicles, uint8_t led);
void LED_FADE(const uint8_t *color, int Cicles, uint8_t led);
uint8_t LED_READ(uint8_t led);
void LED_FADE2COLOR(const uint8_t *color, uint8_t led);
void LED_TEST(uint8_t led);

