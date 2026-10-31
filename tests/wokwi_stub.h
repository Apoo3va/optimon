// wokwi_stub.h - Test-side handle on the chip model: I2C transactions, timer ticks, pins, slider values
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void sim_chip_init(void);
int  sim_i2c_connect(uint32_t addr, int read);
uint8_t sim_i2c_read(void);
int  sim_i2c_write(uint8_t b);
void sim_tick_10ms(void);
void sim_tick_50us(void);
int  sim_pin_get(const char *name);      // value the chip drives on a pin
void sim_pin_set(const char *name, int v);  // value the ESP32 drives into a chip input
float sim_dac(void);
void sim_attr_set(const char *name, float v);
#ifdef __cplusplus
}
#endif
