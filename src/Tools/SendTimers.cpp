#include "SendTimers.hpp"

SendTimers::SendTimers(uint32_t size, bool enabled) {
    config.resize(size, Config(enabled));
}

SendTimers::~SendTimers() {}


void SendTimers::enable(const uint32_t &type) {
    config[type].enable();
}

void SendTimers::disable(const uint32_t &type) {
    config[type].enable(false);
}

bool SendTimers::isSendRequired(const uint32_t &type) {
    Config & conf = config[type];
    if (conf.isEnabled()) {
        if (conf.speed > 0) {
            return isTimeOver(&conf);
        } else if (conf.speed == 0) {
            return false;
        }
        return true;
    }
    return false;
}

bool SendTimers::isTimeOver(Config* conf) {
    timeval now, diff;
    gettimeofday(&now, 0);
    timersub(&now, &(conf->lastSendTime), &diff);
    int64_t diff_ms = static_cast<int64_t>(diff.tv_sec) * UsecPerSec + diff.tv_usec;

    if (diff_ms >= conf->speed) {
        conf->lastSendTime = now;
        return true;
    }
    return false;
}
