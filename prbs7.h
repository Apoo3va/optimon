#pragma once
// prbs7.h - PRBS-7 pattern generator (polynomial x^7 + x^6 + 1).
// Pure C++ (no Arduino code) so it can be unit-tested on a PC.
#include <stdint.h>

// Advances a 7-bit PRBS register by one step and returns the new bit.
// The sequence repeats every 127 bits. The all-zeros state is invalid (it never leaves zero).
static inline uint8_t prbs7Next(uint8_t &s) {
  uint8_t b = ((s >> 6) ^ (s >> 5)) & 1;
  s = (uint8_t)(((s << 1) | b) & 0x7F);
  return b;
}