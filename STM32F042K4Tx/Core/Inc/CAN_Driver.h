#include "main.h"
#include <string.h>

//defines for data size
#define BIT8    1U
#define BIT16   2U
#define BIT32   4U
#define BIT64	8U

//other stuff
uint32_t TxMailbox = 0;
uint32_t fifo = 0;

void data2bytes(void *val, uint8_t *bytes_array, uint8_t size, uint8_t is_float);
void add_data(void *val, uint8_t *bytes_array, uint8_t size, uint8_t is_float, uint8_t start_pos);
void bytes2data(void *val, uint8_t *bytes_array, uint8_t type, uint8_t is_float, uint8_t start_pos);
