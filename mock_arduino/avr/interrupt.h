#ifndef AVR_INTERRUPT_H
#define AVR_INTERRUPT_H
#include "Arduino.h"
#define sei() {}
#define cli() {}
#define WDT_vect
#define EMPTY_INTERRUPT(vector) void vector##_dummy() {}
#endif
