#pragma once

#include <sys/time.h>


class Timer {
 public:
    Timer();
    virtual ~Timer() = default;

    void start(const float &interval_seconds = 0);

    float getElapsedTime();

    bool isExpired();


 private:
    float interval_s;
    bool running;
    timeval startTime;
};
