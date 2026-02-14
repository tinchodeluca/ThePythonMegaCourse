/* State defines
 */
#ifndef DF_STATES_H
#define DF_STATES_H

#include "led_rgb.h"

typedef enum { false, true } bool;

#define FULLBAT 4095 //EQUIVALE A 8V DE BATERIA
#define MEDBAT 2170 //EQUIVALE A 7V DE BATERIA
#define LOWBAT 1240//EQUIVALE A 5V DE BATERIA

#define ERROR       0   //TODO: VARIOS ESTADOS DE ERROR?
#define CONFIG      1
#define STANDBY     2
#define MEASUREMENT 3
#define TEST_ECG    4
#define TEST_PPG    5

#define FIRM_SN 0xF1C10010

//RR,GG,BB cuanto más alto más intensidad, es PWM, más tiempo implica más potencia
//en %, hasta 100(%)
// STATE
extern const uint8_t LED_ERROR[3];
extern const uint8_t LED_STANDBY[3];
extern const uint8_t LED_DISCOVER[3];
extern const uint8_t LED_ACK[3];
extern const uint8_t LED_SLEEP[3];

extern const uint8_t LED_RECOD[3];
extern const uint8_t LED_MEASUR[3];
extern const uint8_t LED_STANDB[3];
extern const uint8_t LED_LOWPOW[3];
extern const uint8_t LED_MEDPOW[3];
extern const uint8_t LED_FULLPOW[3];
extern const uint8_t LED_COLOR_R[3];
extern const uint8_t LED_COLOR_G[3];
extern const uint8_t LED_COLOR_B[3];
extern const uint8_t LED_COLOR_GB[3];
extern const uint8_t LED_OFF[3];
extern const uint8_t LED_WHITE[3];

extern uint8_t STATE;
#define STATE_IDLE 1
#define STATE_TEST 2
#define STATE_ECG 3
#define STATE_PPG 4
#define STATE_ECG_PPG 5
#define LEN_STATE 4

TaskHandle_t xHandleUART,
			xHandleSIGNALS;
			 // xHandleMAX,
			//  xHandleTEST,
	  	  	//  xHandleADS,
			//  xHandleECGPPG;

extern const uint8_t STATE_CONFIG_PROT;
extern const uint8_t STATE_CONFIG_BTH;
extern const uint8_t STATE_CONFIG_ADS;
extern const uint8_t STATE_CONFIG_MAX;

typedef struct RX_PROTOCOL{
	union {
		char recv_str[4];
		struct {
			uint8_t sync;
			uint16_t comm;
			uint8_t checksum;
		}prot;
	};
}RX_PROTOCOL;
/*
typedef struct TX_PROTOCOL{
	union {
		char send_str[8];
		struct {
			uint8_t sync;
			uint8_t state;
			uint8_t ecg[3];
			uint16_t ppg;
			uint8_t checksum;
		}prot;
	};
}TX_PROTOCOL;
*/

typedef struct __attribute__((packed)){
	int32_t ch1;
	int32_t ch2;
}DATAQUEUE_2;

typedef struct __attribute__((packed)){
	int32_t ch1;
	int32_t ch2;
	int32_t ch3;
}DATAQUEUE_3;

typedef union TX_PROTOCOL_SP{
	char send_str[6];
	struct {
		int32_t sync;
		int32_t signal1;
		int32_t signal2;
	}prot;
}TX_PROTOCOL_SP;

// Estructura para comandos
typedef struct {
    char cmd[6];  // 5 chars + null terminator
} uart_cmd_t;



typedef enum {
    STOP,           // Todos los sensores desactivados
    MODE1,          // ECG real @1kHz + PPG1 (IR)
    MODE2,          // ECG real @1kHz + PPG1 (IR) + PPG2 (ROJO)
    MODE3,          // Solo PPGs: PPG1 (IR) + PPG2 (ROJO)
    MODE4,          // ECG + ECG cuadrado
    DBG01,          // ECG cuadrado + Triangular
    DBG02,          // ECG cuadrado + PPG1 (IR) + PPG2 (ROJO)
    DBG03,          // ECG + Señal triangular
	RST             // Vuelve a MODE1 (configuración por defecto)
//TODO: CFG01; .... Modo confguracion de sensores
} OperationMode;

typedef struct {
    bool ecg_enabled;
    bool ppg1_enabled;
    bool ppg2_enabled;
    bool test_signal_enabled;
    uint8_t ecg_mode; // 0=real, 1=cuadrado
    uint8_t channels; //cant de canales
} SystemConfig;

extern const SystemConfig mode_config[];

//TODO: VER SI CONVIENE USAR ESTE PROTOCOLO PARA CONFIGURARLO

#define PROT_BITS		32
#define RX_BYTES_ASYNC	3
/**************************************
 * PROTOCOL COMM MASKS
 **************************************/
#define SYNC_RX 		0xAD
#define RX_COMM_MASK 	0xE000
#define RX_COMM_CFGLB	0x8000
#define RX_COMM_CFBTH	0xA000
#define RX_COMM_CFMAX	0x6000
#define RX_COMM_CFMUX	0x4000
#define RX_COMM_CFADS	0x2000
/**************************************
 * ADS1292 CONFIG MASK
 **************************************/
#define RX_COMM_CFADS_SR		0x1C00
#define RX_COMM_CFADS_CH1		0x03C0
#define RX_COMM_CFADS_CH1_EN	0x0200
#define RX_COMM_CFADS_GAIN1		0x01C0
#define RX_COMM_CFADS_CH2		0x003C
#define RX_COMM_CFADS_CH2_EN	0x0020
#define RX_COMM_CFADS_GAIN2		0x001C
#define RX_COMM_CFADS_RLD		0x0002
#define RX_COMM_CFADS_RSVD		0x0001
/**************************************
 * SIGNAL MUX CONFIG MASK
 **************************************/
#define RX_COMM_CFMUX_MASK		0x1F00
#define RX_COMM_CFMUX_ADS		0x1C00
#define RX_COMM_CFMUX_MAX		0x0300
#define RX_COMM_CFMUX_RSVD		0x00FF
//#define RX_COMM_CFMUX_TEMP		0x0001
/**************************************
 * SIGNAL MUX CONFIG DEFINES
 **************************************/
#define RX_COMM_CFMUX_ADS_CH1_NORM 	0x00
#define RX_COMM_CFMUX_ADS_CH1_TEST	0x04
#define RX_COMM_CFMUX_ADS_CH2_NORM	0x08
#define RX_COMM_CFMUX_ADS_CH2_TEST	0x0C
#define RX_COMM_CFMUX_ADS_TEST_TRIA	0x10
#define RX_COMM_CFMUX_ADS_OFF		0x14
#define RX_COMM_CFMUX_MAX_IR		0x00
#define RX_COMM_CFMUX_MAX_RED		0x01
#define RX_COMM_CFMUX_MAX_TEST_TRIA	0x02
#define RX_COMM_CFMUX_MAX_OFF		0x03
/**************************************
 * MAX CONFIG MASK
 **************************************/
#define RX_COMM_CFMAX_MASK		0x6000
#define RX_COMM_CFMAX_SR		0x1C00
#define RX_COMM_CFMAX_SAVE		0x0280
#define RX_COMM_CFMAX_ILED		0x0070
#define RX_COMM_CFMAX_IREF		0x000C
#define RX_COMM_CFMAX_MODE		0x0002
#define RX_COMM_CFMAX_RSVD		0x0001
/**************************************
 * GLOBAL CONFIG MASK
 **************************************/
#define RX_COMM_CFGBL_MASK		0x8000
#define RX_COMM_CFGBL_STOP		0x1000
#define RX_COMM_CFGBL_SPLOT		0x0800
#define RX_COMM_CFGBL_ADS1		0x0400
#define RX_COMM_CFGBL_ADS2		0x0200
#define RX_COMM_CFGBL_MAX		0x0100
#define RX_COMM_CFGBL_TLED		0x0080
#define RX_COMM_CFGBL_TUART		0x0040
#define RX_COMM_CFGBL_RSVD		0x003F
/**************************************
 * BLUETOOTH CONFIG MASK
 **************************************/
#define RX_COMM_CFBTH_MASK		0xA000
#define RX_COMM_CFBTH_NAME		0x1E00
#define RX_COMM_CFBTH_BAUD		0x01C0
#define RX_COMM_CFBTH_PAR		0x0020
#define RX_COMM_CFBTH_STOP		0x0010
#define RX_COMM_CFBTH_RSVD		0x00F0
/**************************************
 * RESERVED CONFIG MASK
 **************************************/
#define RX_COMM_RESERVED1		0xC000
#define RX_COMM_RESERVED2		0xE000

#endif // DF_STATES_H
