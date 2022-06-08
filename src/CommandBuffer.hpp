# pragma once

#include <vector>
#include <string>
#include "RingBuffer.hpp"

namespace robot_remote_control {

struct CommandBufferBase {
    CommandBufferBase() {}
    virtual ~CommandBufferBase() {}
    virtual bool write(const std::string &serializedMessage) = 0;
    virtual bool read(std::string *receivedMessage) = 0;
    void notify() {
        auto callCb = [](const std::function<void()> &cb){cb();};
        std::for_each(callbacks.begin(), callbacks.end(), callCb);
    }
    std::vector< std::function<void()> > callbacks;
    void addCommandReceivedCallback(const std::function<void()> &cb) {
        callbacks.push_back(cb);
    }
};

template<class COMMAND> struct CommandBuffer: public CommandBufferBase{
 public:
    CommandBuffer():isnew(false) {}

    virtual ~CommandBuffer() {}

    bool read(COMMAND *target) {
        bool oldval = isnew.load();
        *target = command.lockedAccess().get();
        isnew.store(false);
        return oldval;
    }

    void write(const COMMAND &src) {
        command.lockedAccess().set(src);
        isnew.store(true);
        notify();
    }

    virtual bool write(const std::string &serializedMessage) {
        // command.lock();
        if (!command.lockedAccess()->ParseFromString(serializedMessage)) {
            isnew.store(false);
            return false;
        }
        isnew.store(true);
        notify();
        return true;
    }

    virtual bool read(std::string *receivedMessage) {
        bool oldval = isnew.load();
        command.lockedAccess()->SerializeToString(receivedMessage);
        isnew.store(false);
        return oldval;
    }

 private:
    LockableClass<COMMAND> command;
    std::atomic<bool> isnew;
};

template<class COMMAND> struct CommandRingBuffer: public CommandBufferBase{
 public:
    explicit CommandRingBuffer(const size_t & buffersize):isnew(false), buffer(RingBuffer<COMMAND>(buffersize)) {}

    virtual ~CommandRingBuffer() {}

    bool read(COMMAND *target) {
        bool oldval = isnew.load();
        auto lockable = buffer.lockedAccess();
        if (!lockable->popData(target, true)) {
            auto protocommand = lastcommand.lockedAccess();
            target->CopyFrom(protocommand.get());
        }
        isnew.store(lockable->size());
        return oldval;
    }

    void write(const COMMAND &src) {
        {
            auto lockable = buffer.lockedAccess();
            lockable->pushData(src, true);
            isnew.store(lockable->size());
        }
        notify();
    }

    virtual bool write(const std::string &serializedMessage) {
        COMMAND protocommand;
        if (!protocommand.ParseFromString(serializedMessage)) {
            return false;
        }
        {
            auto lockable = buffer.lockedAccess();
            lockable->pushData(protocommand, true);
            isnew.store(lockable->size());
        }
        notify();
        return true;
    }

    virtual bool read(std::string *receivedMessage) {
        bool oldval = isnew.load();
        auto protocommand = lastcommand.lockedAccess();
        auto lockable = buffer.lockedAccess();
        lockable->popData(&(protocommand.get()), true);
        protocommand->SerializeToString(receivedMessage);
        isnew.store(lockable->size());
        return oldval;
    }

 private:
    LockableClass<RingBuffer<COMMAND>> buffer;
    LockableClass<COMMAND> lastcommand;
    std::atomic<bool> isnew;
};

}  // namespace robot_remote_control
