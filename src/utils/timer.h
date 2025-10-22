#ifndef TIMER_H
#define TIMER_H
#include <time.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double dt;         // seconds since last tick
    bool overrun;      // true if we missed our period
    struct timespec period;
    struct timespec next;
    struct timespec previous;
} Timer;


void timer_tick(Timer *timer);
void timer_reset(Timer *timer);
void timer_init(Timer *timer, double frequency_hz);

#ifdef __cplusplus
}
#endif

#endif
