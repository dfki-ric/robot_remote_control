#pragma once

#include <vector>
#include <sys/time.h>
#include <cstdint>

namespace robot_remote_control {

class SendTimers {
 public:
    /**
     * @brief Construct a new Send Timers object
     * 
     * @warning you need to call resize to inti the timers
     * 
     */
    SendTimers() {}

    /**
     * @brief Construct a new Send Timers object
     * 
     * @param size number of configurations to store, 
     *             should be robot_remote_control::TELEMETRY_MESSAGE_TYPES_NUMBER
     *             or robot_remote_control::TELEMETRY_MESSAGE_TYPES_NUMBER
     *             from MessageTypes.h
     * @param enabled if true "always send" if not configured otherwise is the default
     */
    explicit SendTimers(uint32_t size, bool enabled = true);
    virtual ~SendTimers();

    void resize(const uint32_t &size, bool enabled = true) {
        config.resize(size, Config(enabled));
    }

    /**
     * @brief check if data needs to be send based on the config
     * 
     * @param type message type enum
     * @return true if time elapsed since last call
     * @return false if tiem is not elapsed since last call
     */
    bool isSendRequired(const uint32_t &type);

    /**
     * @brief Set time during which no data should be forwarded
     * 
     * @param type 
     * @param speed_ms time between calls isSendRequired() where it retruns true,
     *                  0 means isSendRequired() is always false
     *                  <0 means isSendRequired() is always true
     */
    void setSendSpeed(const uint32_t &type, int speed_ms = -1 ) {
        config[type].speed = speed_ms*1000;
    }

    /**
     * @brief enables the timing: return of isSendRequired() is defiend by setSendSpeed()
     * 
     * @param type message type enum
     */
    void enable(const uint32_t &type);

    /**
     * @brief disabled the timing: isSendRequired() is always false
     * 
     * @param type message type enum
     */
    void disable(const uint32_t &type);


 private:
    static const int UsecPerSec = 1000000LL;
    class Config {
     public:
        explicit Config(bool enabled = true):speed(-1), enabled(enabled) {
            lastSendTime.tv_sec = 0;
            lastSendTime.tv_usec = 0;
            enabled = false;
        }
        int speed;  // how fast shall it be sent (time in ms between two deliveries)
        timeval lastSendTime;
        void enable(bool newvalue = true) {
            enabled = newvalue;
        }
        bool isEnabled() {
            return enabled;
        }
     private:
        bool enabled;
    };

    bool isTimeOver(Config* conf);

    std::vector<Config> config;
};

}  // namespace robot_remote_control
