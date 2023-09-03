#ifndef LEDLIB_H
#define LEDLIB_H
#include <Arduino.h>
void LEDLib_init();
void LEDclear();
void LEDdisplay();
void LEDStateSet(int x, int y, bool state);
void LEDStateInv(int x, int y);
void LEDStateSetAll(bool state);
bool LEDStateGet(int x, int y);
#endif