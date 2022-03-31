#include "Timer.hpp"

#include <stdio.h>

Timer::Timer() : running(false) {}

void Timer::start(const float &interval_seconds) {
    running = true;
    interval_s = interval_seconds;
    gettimeofday(&startTime, 0);
}

bool Timer::isExpired() {
    if (running && getElapsedTime() >= interval_s) {
        return true;
    }
    return false;
}

float Timer::getElapsedTime() {
    if (running) {
        timeval now, diff;
        gettimeofday(&now, 0);
        timersub(&now, &startTime, &diff);
        return diff.tv_sec + (diff.tv_usec / 1000000.0);
    }
    return -1;
}
