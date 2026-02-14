/*
 * Define Queues and Semaphores
 */

#include "queue.h"
#include "semphr.h"

QueueHandle_t
				qe_ecg,
				qe_ppg,
				qe_rx_uart,
				qe_tx_uart,
				qe_tx_uart2,
				qe_tx_uart3,
//				qe_ads_config,
//				qe_max_config,
//				qe_bth_config,
//				qe_protocol_config,
				qe_protocol,
				qe_states,
				qe_mode;

SemaphoreHandle_t sm_BTN;
