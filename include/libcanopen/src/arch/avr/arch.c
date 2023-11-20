#ifndef ARCH_BYTE16
#include <avr/wdt.h>

void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();

    return;
}

void soft_restart()
{
    wdt_enable(WDTO_15MS);
    for(;;)
        ;
}
#endif
