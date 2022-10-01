#include "clock_gettime.h"
//#include "api.h"

/*#define micro_rollover_useconds 4294967295

int clock_realtime_count = 0;

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    if (unused == CLOCK_REALTIME) {
        clock_realtime_count++;
        char s[11]; 
        sprintf(s,"Count: %ld", clock_realtime_count);
        lcd_set_text(4, s);
        
        tp->tv_sec = -1;
        tp->tv_nsec = -1;

    } else {
        uint64_t m = micros();

        tp->tv_sec = m / 1000000;
        tp->tv_nsec = (m % 1000000) * 1000;
    }


    return 0;
}*/

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    (void)unused;

    uint64_t m = micros();

    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;

    return 0;
}

/*int clock_gettime(clockid_t unused, struct timespec *tp)
{
    (void)unused;
    static uint32_t rollover = 0;
    static uint32_t last_measure = 0;

    uint32_t m = micros();
    rollover += (m < last_measure) ? 1 : 0;

    uint64_t real_us = (uint64_t) (m + rollover * micro_rollover_useconds);
    tp->tv_sec = real_us / 1000000;
    tp->tv_nsec = (real_us % 1000000) * 1000;
    last_measure = m;

    return 0;
}*/
