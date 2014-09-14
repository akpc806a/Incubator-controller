#include <stdint.h>

uint8_t ds_read_data_single(uint8_t *buff);
void ds_init(void);
uint8_t ds_start_convert_single(void);
void ds_strong_pull_up(uint8_t enable);
signed int ds_conv_to_temperature(uint8_t *buff);
