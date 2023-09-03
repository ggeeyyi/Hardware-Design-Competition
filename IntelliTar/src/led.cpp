#include <led.h>
#define RCLK_Pin_x 18
#define SRCLK_Pin_x 19
#define SER_Pin_x 21
#define RCLK_Pin_y 15
#define SRCLK_Pin_y 17
#define SER_Pin_y 5

bool LEDstate[6][8];

void LEDLib_init()
{
    pinMode(RCLK_Pin_x, OUTPUT);
    pinMode(SRCLK_Pin_x, OUTPUT);
    pinMode(SER_Pin_x, OUTPUT);
    pinMode(RCLK_Pin_y, OUTPUT);
    pinMode(SRCLK_Pin_y, OUTPUT);
    pinMode(SER_Pin_y, OUTPUT);
}

void updateShiftRegisterX(unsigned int leds)
{
    digitalWrite(RCLK_Pin_x, LOW);
    shiftOut(SER_Pin_x, SRCLK_Pin_x, LSBFIRST, leds);
    digitalWrite(RCLK_Pin_x, HIGH);
}
void updateShiftRegisterY(unsigned int leds)
{
    digitalWrite(RCLK_Pin_y, LOW);
    shiftOut(SER_Pin_y, SRCLK_Pin_y, LSBFIRST, leds);
    digitalWrite(RCLK_Pin_y, HIGH);
}

void LEDclear()
{
    updateShiftRegisterX(B00000000);
    updateShiftRegisterY(B11111111);
}

byte arr2byte(bool arr[8])
{
    byte result = 0;
    for (int i = 0; i < 8; i++)
    {
        result <<= 1;
        result |= arr[i];
    }
    return result;
}

void LEDStateSet(int x, int y, bool state)
{
    if(x>12 || y>4 || x<1 || y<1)
        return;
    LEDstate[(x-1)/2][(x-1)%2*4+y-1] = state;
}

void LEDStateInv(int x, int y)
{
    if(x>12 || y>4 || x<1 || y<1)
        return;
    LEDstate[(x-1)/2][(x-1)%2*4+y-1] = !LEDstate[(x-1)/2][(x-1)%2*4+y-1];
}

bool LEDStateGet(int x, int y)
{
    if(x>12 || y>4 || x<1 || y<1)
        return false;
    return LEDstate[(x-1)/2][(x-1)%2*4+y-1];
}

void LEDStateSetAll(bool state)
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 8; j++)
            LEDstate[i][j] = state;
}

void LEDdisplay()
{
    byte ScanY = B10000000;
    for (int i = 0; i < 6; i++)
    {
        LEDclear();
        updateShiftRegisterY(~ScanY);
        updateShiftRegisterX(arr2byte(LEDstate[i]));
        ScanY >>= 1;
    }
}
