#include "mbed.h"

DigitalOut led1(LED1);
DigitalOut led2(LED2);

int main()
{
  led1 = 0;
  led2 = 0;

  for (;;) {
    led1 = 1;
    wait(0.2);
    led1 = 0;
    led2 = 1;
    wait(0.2);
    led2 = 0;
  }
}
