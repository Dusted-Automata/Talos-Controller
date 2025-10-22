#include "timer.h"
#include <stdio.h>

static inline void timespec_add(struct timespec *timer, const struct timespec *b) {
    timer->tv_sec += b->tv_sec;
    timer->tv_nsec += b->tv_nsec;

    if (timer->tv_nsec >= 1000000000L) { 
        timer->tv_sec++;
        timer->tv_nsec -= 1000000000L;
    }
}


static inline void timespec_sub(struct timespec *timer, const struct timespec *b) {
    timer->tv_sec -= b->tv_sec;
    timer->tv_nsec -= b->tv_nsec;

    if (timer->tv_nsec < 0) { 
        timer->tv_sec--;
        timer->tv_nsec += 1000000000L;
    }
}


static double timespec_diff(const struct timespec *end, const struct timespec *start) {
    return (end->tv_sec - start->tv_sec) + (end->tv_nsec - start->tv_nsec) / 1e9;
}

void timer_tick(Timer *t) {

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);


    if ((now.tv_sec < t->next.tv_sec) ||
        (now.tv_sec == t->next.tv_sec && now.tv_nsec < t->next.tv_nsec)) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t->next, NULL);
        now = t->next;
    }

    t->dt = timespec_diff(&now, &t->previous);
    t->previous = now;
    struct timespec over = now;
    timespec_sub(&over, &t->next);
    t->overrun = (over.tv_sec > 0 || over.tv_nsec > 0);

    if (t->overrun) {
        fprintf(stderr, "control-loop overrun (dt=%.6fs)\n", t->dt);
        t->next = now;
        t->previous = now;
    }

    t->previous = now;
    timespec_add(&t->next, &t->period);
}

void timer_reset(Timer *timer) {
    clock_gettime(CLOCK_MONOTONIC, &timer->next);
    timer->previous = timer->next;
    timer->dt = 0.0;
}

void timer_init(Timer *timer, double frequency_hz) {
    double sec = 1.0 / frequency_hz;
    struct timespec t;
    t.tv_sec = (time_t)sec;
    t.tv_nsec = (long)((sec - t.tv_sec) * 1e9);
    timer->period = t;
    clock_gettime(CLOCK_MONOTONIC, &timer->next);
    timer->previous = timer->next;
    timer->dt = 0.0;
}
