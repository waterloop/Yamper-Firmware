#pragma once

#define N 12
#define R 0.057
#define SAMPLING_PERIOD_S 0.1

void init_timer(void);
float compute_rpm(int n);
void add_data(void *val, uint8_t *bytes_array, uint8_t size, uint8_t is_float, uint8_t start_pos);

int app_main();

