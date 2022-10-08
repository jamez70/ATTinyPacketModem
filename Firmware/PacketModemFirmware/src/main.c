
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

void setup(void);
void loop(void);

int main(void)
{
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0;
    setup();
    while (1)
    {
        loop();
    }
    return 0;
}
// end of file
