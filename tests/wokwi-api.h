// Minimal PC stand-in for the Wokwi chip API (only what sfp.chip.c uses)
#ifndef WOKWI_API_H
#define WOKWI_API_H
#include <stdint.h>
#include <stdbool.h>
enum pin_value { LOW = 0, HIGH = 1 };
enum pin_mode { INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 2, INPUT_PULLDOWN = 3, ANALOG = 4, OUTPUT_LOW = 16, OUTPUT_HIGH = 17 };
#ifdef __cplusplus
extern "C" {
#endif
typedef int32_t pin_t;
typedef uint32_t timer_t_;
#define timer_t timer_t_
typedef struct { void *user_data; uint32_t address; pin_t scl; pin_t sda;
  bool (*connect)(void *user_data, uint32_t address, bool read);
  uint8_t (*read)(void *user_data);
  bool (*write)(void *user_data, uint8_t data);
  void (*disconnect)(void *user_data);
  uint32_t reserved[8]; } i2c_config_t;
typedef struct { void *user_data; void (*callback)(void *user_data); uint32_t reserved[8]; } timer_config_t;
void chip_init(void);
pin_t pin_init(const char *name, uint32_t mode);
uint32_t pin_read(pin_t pin);
void pin_write(pin_t pin, uint32_t value);
float pin_dac_write(pin_t pin, float voltage);
uint32_t attr_init_float(const char *name, float default_value);
float attr_read_float(uint32_t attr);
uint32_t i2c_init(const i2c_config_t *config);
timer_t timer_init(const timer_config_t *config);
void timer_start_ns(timer_t timer, uint64_t ns, bool repeat);
#ifdef __cplusplus
}
#endif
#endif
