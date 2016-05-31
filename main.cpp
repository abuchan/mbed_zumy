#include "mbed.h"
#include "SerialRPCInterface.h"

SerialRPCInterface RPC(USBTX, USBRX, 115200);
DigitalOut leds[] = { (LED1), (LED2), (LED3), (LED4) };

int main(void) {
    int count = 0;
    int i;
    while(true) {
        if (++count == 4)
            count = 0;
        for (i=0; i<4; i++) {
            leds[i] = (count == i) ? 1 : 0;
        }
        wait(0.25);
    }
}