#pragma once

#include <sys/time.h>


class Timer {
 public:
    explicit Timer();
    virtual ~Timer();

    void start(const float &interval_seconds = 0);

    float getElapsedTime();

    bool isExpired();


 private:
    float interval_s;
    timeval startTime;
};
