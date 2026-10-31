// wokwi_stub.c - PC stand-in for the Wokwi runtime: records pins, attributes, I2C and timer callbacks
// so tests can drive the chip model (sfp.chip.c) directly.
#include "wokwi-api.h"
#include "wokwi_stub.h"
#include <string.h>
#include <stdio.h>
static struct { const char *name; uint32_t value; } pins[16];
static int npins;
static struct { const char *name; float value; } attrs[32];
static int nattrs;
static i2c_config_t i2c;
static timer_config_t timers[4];
static int ntimers;
static float dac;
pin_t pin_init(const char *name, uint32_t mode) { pins[npins].name = name; pins[npins].value = (mode == OUTPUT_HIGH); return npins++; }
uint32_t pin_read(pin_t p) { return pins[p].value; }
void pin_write(pin_t p, uint32_t v) { pins[p].value = v; }
float pin_dac_write(pin_t p, float v) { (void)p; dac = v; return v; }
uint32_t attr_init_float(const char *name, float d) {
  for (int i = 0; i < nattrs; i++) if (!strcmp(attrs[i].name, name)) return i;
  attrs[nattrs].name = name; attrs[nattrs].value = d; return nattrs++; }
float attr_read_float(uint32_t a) { return attrs[a].value; }
uint32_t i2c_init(const i2c_config_t *c) { i2c = *c; return 0; }
timer_t timer_init(const timer_config_t *c) { timers[ntimers] = *c; return ntimers++; }
void timer_start_ns(timer_t t, uint64_t ns, bool r) { (void)t; (void)ns; (void)r; }
static int pin_index(const char *name) { for (int i = 0; i < npins; i++) if (!strcmp(pins[i].name, name)) return i; return -1; }
void sim_chip_init(void) { chip_init(); }
int sim_i2c_connect(uint32_t a, int r) { return i2c.connect(i2c.user_data, a, r); }
uint8_t sim_i2c_read(void) { return i2c.read(i2c.user_data); }
int sim_i2c_write(uint8_t b) { return i2c.write(i2c.user_data, b); }
void sim_tick_10ms(void) { timers[0].callback(timers[0].user_data); }
void sim_tick_50us(void) { timers[1].callback(timers[1].user_data); }
int sim_pin_get(const char *n) { int i = pin_index(n); return i < 0 ? -1 : (int)pins[i].value; }
void sim_pin_set(const char *n, int v) { int i = pin_index(n); if (i >= 0) pins[i].value = v; }
float sim_dac(void) { return dac; }
void sim_attr_set(const char *name, float v) { for (int i = 0; i < nattrs; i++) if (!strcmp(attrs[i].name, name)) attrs[i].value = v; }
