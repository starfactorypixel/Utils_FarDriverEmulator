#pragma once
// Позволяет собирать ядро эмулятора и на AVR, и на ПК (для тестов).
#include <stdint.h>
#include <string.h>
#include <math.h>

#if defined(ARDUINO)
  #include <Arduino.h>
  #include <avr/pgmspace.h>
#else
  #define PROGMEM
  #define memcpy_P memcpy
  #define pgm_read_byte(p) (*(const uint8_t *)(p))
  #ifndef PI
    #define PI 3.14159265358979f
  #endif
#endif
