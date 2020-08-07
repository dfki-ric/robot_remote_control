#include "Timer.hpp"

#include <stdio.h>

Timer::Timer() {}
Timer::~Timer() {}

void Timer::start(const float &interval_seconds) {
    interval_s = interval_seconds;
    gettimeofday(&startTime, 0);
}



bool Timer::isExpired() {
    if (getElapsedTime() >= interval_s) {
        return true;
    }
    return false;
}

float Timer::getElapsedTime() {
    timeval now, diff;
    gettimeofday(&now, 0);
    timersub(&now, &startTime, &diff);
    return diff.tv_sec + (diff.tv_usec / 1000000.0);
}
