#include "df_states.h"

const uint8_t LED_ERROR[3]    = {70,00,00};
const uint8_t LED_STANDBY[3]  = {10,70,00};
const uint8_t LED_DISCOVER[3] = {00,50,50};
const uint8_t LED_ACK[3]      = {00,50,50};
const uint8_t LED_SLEEP[3]    = {40,70,00};
// STATE LED2
const uint8_t LED_RECOD[3]    = {70,00,00};
const uint8_t LED_MEASUR[3]   = {00,70,00};
const uint8_t LED_STANDB[3]   = {10,70,30};
const uint8_t LED_LOWPOW[3]   = {60,00,00};
const uint8_t LED_MEDPOW[3]   = {00,00,60};
const uint8_t LED_FULLPOW[3]  = {15,60,15};

const uint8_t LED_COLOR_R[3]  = {80,00,00};
const uint8_t LED_COLOR_G[3]  = {00,80,00};
const uint8_t LED_COLOR_B[3]  = {00,00,80};
const uint8_t LED_COLOR_GB[3] = {00,80,00};
const uint8_t LED_OFF[3]      = {00,00,00};
const uint8_t LED_WHITE[3]    = {99,99,99};

uint8_t STATE;
//const uint8_t STATE_IDLE      = 5;
//const uint8_t STATE_TEST      = 7;
////const uint8_t STATE_ECG     = 8
//const uint8_t STATE_PPG       = 9;
//const uint8_t STATE_ECG_PPG   = 10;
//const uint8_t PROT_SPLOT      = 11;

const uint8_t STATE_CONFIG_PROT = 100;
const uint8_t STATE_CONFIG_BTH  = 101;
const uint8_t STATE_CONFIG_ADS  = 110;
const uint8_t STATE_CONFIG_MAX  = 120;

//TODO: Agregar frecuencia de parpadeo??
const SystemConfig mode_config[] = {
    [MODE1] = {
        .ecg_enabled  = true,
        .ppg1_enabled = true,
        .ecg_mode     = 0,
		.channels     = 2
    },
    [MODE2] = {
        .ecg_enabled  = true,
        .ppg1_enabled = true,
        .ppg2_enabled = true,
        .ecg_mode     = 0,
		.channels     = 3
    },
    [MODE3] = {
        .ppg1_enabled = true,
        .ppg2_enabled = true,
		.channels     = 2
    },
    [MODE4] = {
        .ecg_enabled         = true,
		.channels            = 2
    },
    [DBG01] = {
        .ecg_enabled  = true,
        .ppg1_enabled = true,
        .ecg_mode     = 2,
		.channels     = 2
    },
    [DBG02] = {
        .ecg_enabled         = true,
        .ppg1_enabled        = true,
        .test_signal_enabled = true,
        .ecg_mode            = 1,
		.channels            = 3
    },
    [DBG03] = {
        .ecg_enabled         = true,
        .test_signal_enabled = true,
        .ecg_mode            = 0,
		.channels            = 2
    },
    [STOP] = {
        .ecg_enabled         = false,
        .ppg1_enabled        = false,
        .ppg2_enabled        = false,
        .test_signal_enabled = false,
    }
};
