#include "main.h"

int8_t checksum(char *data, int8_t size);

BaseType_t fnc_CONFIG_ADS (RX_PROTOCOL RX_STR);
BaseType_t fnc_CONFIG_MAX(RX_PROTOCOL RX_STR);
BaseType_t fnc_CONFIG_BLUETOOTH(RX_PROTOCOL RX_STR);
BaseType_t fnc_CONFIG_MUX(RX_PROTOCOL RX_STR);
BaseType_t fnc_CONFIG_GLOBAL(RX_PROTOCOL RX_STR);

void fnc_TEST_LED (void);
