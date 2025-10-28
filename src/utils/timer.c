#include "timer.h"
#include <errno.h>
#include <stdio.h>

#ifdef __APPLE__
// macOS doesn't implement clock_nanosleep/TIMER_ABSTIME.
// Provide a compatible replacement that supports TIMER_ABSTIME
// by converting the absolute time to a relative nanosleep.
#ifndef TIMER_ABSTIME
#define TIMER_ABSTIME 1
#endif

static int clock_nanosleep_compat(clockid_t clk_id, int flags,
                                  const struct timespec *req,
                                  struct timespec *rem)
{
    if (flags == TIMER_ABSTIME) {
        struct timespec now, rel;
        if (clock_gettime(clk_id, &now) != 0) return -1;

        // rel = req - now
        rel.tv_sec  = req->tv_sec  - now.tv_sec;
        rel.tv_nsec = req->tv_nsec - now.tv_nsec;
        if (rel.tv_nsec < 0) { rel.tv_sec--; rel.tv_nsec += 1000000000L; }

        // Already in the past? nothing to sleep.
        if (rel.tv_sec < 0) return 0;

        // Sleep, handling EINTR by continuing with remaining time
        while (nanosleep(&rel, &rel) == -1 && errno == EINTR) {}
        return 0;
    } else {
        // Relative sleep path
        while (nanosleep(req, rem) == -1 && errno == EINTR) {}
        return 0;
    }
}

#define clock_nanosleep clock_nanosleep_compat
#endif

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
