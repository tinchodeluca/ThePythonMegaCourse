#include "led_rgb.h"
#include "df_states.h"

void INIT_LEDS (TIM_HandleTypeDef HTIM_1, TIM_HandleTypeDef HTIM_2){
	HTIM_LED1 = HTIM_1;
	HTIM_LED2 = HTIM_2;
}

int GET_SIGN(int number) {
    // Extract the sign bit (the most significant bit) of the number
    int sign_bit = (number >> (sizeof(int) * 8 - 1)) & 1;

    // If the sign bit is 1, the number is negative
    // If the sign bit is 0, the number is non-negative (positive or zero)
    // We use -1 (0xFFFFFFFF in binary) to represent the sign of a negative number
    // and 1 (0x00000001 in binary) to represent the sign of a non-negative number
    return sign_bit ? -1 : 0x00000001;
}
void LED_Color (const uint8_t *color, uint8_t led){
	if ( LED_2 == led){
		LED_2_R = color[0];
		LED_2_G = color[1];
		LED_2_B = color[2];
	}
	if ( LED_1 == led){
		LED_1_R = color[0];
		LED_1_G = color[1];
		LED_1_B = color[2];
	}
}
void LED_Toggle_Color (const uint8_t *color, uint8_t led){
	if ( LED_2 == led){
		if ((LED_2_R == color[0])&&(LED_2_G == color[1])&&(LED_2_B == color[2]))
			LED_Off(led);
		else{
			LED_2_R = color[0];
			LED_2_G = color[1];
			LED_2_B = color[2];
		}
	}
	if ( LED_1 == led){
		if ((LED_1_R == color[0])&&(LED_1_G == color[1])&&(LED_1_B == color[2]))
			LED_Off(led);
		else{
			LED_1_R = color[0];
			LED_1_G = color[1];
			LED_1_B = color[2];
		}
	}
}

void LED_Off(uint8_t led){
//	LED_Color(LED_OFF, led);
	if ( LED_2 == led){
		LED_2_R = 0;
		LED_2_G = 0;
		LED_2_B = 0;
	}
	if ( LED_1 == led){
		LED_1_R = 0;
		LED_1_G = 0;
		LED_1_B = 0;
	}
}
void LED_START(uint8_t led){
	LED_Off(led);
	if ( LED_1 == led ){
		HAL_TIM_PWM_Start(&HTIM_LED1, TIM_LED1_CH1);
		HAL_TIM_PWM_Start(&HTIM_LED1, TIM_LED1_CH2);
		HAL_TIM_PWM_Start(&HTIM_LED1, TIM_LED1_CH3);
	}
	if ( LED_2 == led ){
		HAL_TIM_PWM_Start(&HTIM_LED2, TIM_LED2_CH1);
		HAL_TIM_PWM_Start(&HTIM_LED2, TIM_LED2_CH2);
		HAL_TIM_PWM_Start(&HTIM_LED2, TIM_LED2_CH3);
	}
}
void LED_STOP(uint8_t led){
	if ( LED_1 == led){
		HAL_TIM_PWM_Stop(&HTIM_LED1, TIM_LED1_CH1);
		HAL_TIM_PWM_Stop(&HTIM_LED1, TIM_LED1_CH2);
		HAL_TIM_PWM_Stop(&HTIM_LED1, TIM_LED1_CH3);
	}
	if ( LED_2 == led){
		HAL_TIM_PWM_Stop(&HTIM_LED2, TIM_LED2_CH1);
		HAL_TIM_PWM_Stop(&HTIM_LED2, TIM_LED2_CH2);
		HAL_TIM_PWM_Stop(&HTIM_LED2, TIM_LED2_CH3);
	}
}

void LED_BLINK(uint8_t TOn, uint8_t TOff, uint8_t Cicles, uint8_t led){
	while (Cicles){
		LED_STOP(led);
		vTaskDelay(TOff*100);
		LED_START(led);
		vTaskDelay(TOn*100);
		Cicles--;
	}
}
// Fading between ON&OFF /\/\/\/\....
void LED_FADE(const uint8_t *color, int Cicles, uint8_t led){
	uint8_t POWER = 1;
	uint8_t NewColor[3];
	int SIGN      = 1;
	int index;
	const uint8_t MAX = 90;
	const uint8_t min = 10;
	Cicles = Cicles*(MAX - min);

	while(Cicles){
		for(index=0; index<3; index++){
			if (color[index])
				NewColor[index] = POWER;
		}
		LED_Color(NewColor, led);
//		if ( color[0] )
//			LED_1_R = POWER;
//		if ( color[1] )
//			LED_1_B = POWER;
//		if ( color[2] )
//			LED_1_G = POWER;
		if (MAX == POWER)
			SIGN = 0;
		if (min == POWER)
			SIGN = 1;

		if ( 1 == SIGN)
			POWER++;
		else
			POWER--;
		vTaskDelay(10);
		Cicles--;
	}
}

uint8_t LED_READ(uint8_t led){
	uint8_t prev_color[3];
	if ( LED_1 == led ){
		prev_color[0] = LED_1_R;
		prev_color[1] = LED_1_G;
		prev_color[2] = LED_1_B;
	}
	if ( LED_2 == led ){
		prev_color[0] = LED_2_R;
		prev_color[1] = LED_2_G;
		prev_color[2] = LED_2_B;
	}
	return prev_color;
}
void LED_FADE2COLOR (const uint8_t *color, uint8_t led){
	uint8_t prev_color[3], i = 99;
	uint32_t LED_R, LED_G, LED_B;
	int dif_color[3];

	if ( LED_1 == led ){
		LED_R = LED_1_R;
		LED_G = LED_1_G;
		LED_B = LED_1_B;
	}
	if ( LED_2 ==led ){
		LED_R = LED_2_R;
		LED_G = LED_2_G;
		LED_B = LED_2_B;
	}
	prev_color[0] = LED_R;
	prev_color[1] = LED_G;
	prev_color[2] = LED_B;

	for(i = 0; i< 3; i++)
		dif_color[i] = color[i] - prev_color[i];

	i = 99;
	while (i){
		if (color[0] != LED_R)
			prev_color[0] = prev_color[0] + 1*GET_SIGN(dif_color[0]);
		if (color[1] != LED_G)
			prev_color[1] = prev_color[1] + 1*GET_SIGN(dif_color[1]);
		if (color[2] != LED_B)
			prev_color[2] = prev_color[2] + 1*GET_SIGN(dif_color[2]);

//		LED_2_R = prev_color[0];
		LED_Color(prev_color, led);
		vTaskDelay(5);
		i--;
	}
}
//Test para recorrer todos los colores del led RGB
void LED_TEST(uint8_t led){
	uint8_t color[3] = {05,05,05};
	uint32_t i;

	for(i = LED_min_POWER; i <= LED_MAX_POWER; i++){
		color[0] = i;
		if ( LED_MAX_POWER == color[0]){
			color[1]++;
			i = LED_min_POWER;
		}
		if ( LED_MAX_POWER == color[1]){
			color[1] = LED_min_POWER;
			color[2]++;
		}
		if ( LED_MAX_POWER == color[2]){
			color[2] = LED_min_POWER;
			i = LED_MAX_POWER +1;
		}
		LED_Color(color, led);
		vTaskDelay(5);
	}
}
