# pragma once

#include <vector>
#include <string>
#include "RingBuffer.hpp"

namespace robot_remote_control {

struct CommandBufferBase {
    CommandBufferBase() {}
    virtual ~CommandBufferBase() {}
    virtual bool write(const std::string &serializedMessage) = 0;
    virtual bool read(std::string *receivedMessage, bool onlyNewest = true) = 0;
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
    explicit CommandBuffer(const size_t & buffersize):isnew(false), buffer(RingBuffer<COMMAND>(buffersize)) {}

    virtual ~CommandBuffer() {}

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

    bool read(COMMAND *target, bool onlyNewest = true) {
        bool oldval = isnew.load();
        auto lockable = buffer.lockedAccess();
        if (!lockable->popData(target, onlyNewest)) {
            auto protocommand = lastcommand.lockedAccess();
            target->CopyFrom(protocommand.get());
        }
        isnew.store(lockable->size());
        return oldval;
    }

    virtual bool read(std::string *receivedMessage, bool onlyNewest = true) {
        bool oldval = isnew.load();
        auto protocommand = lastcommand.lockedAccess();
        auto lockable = buffer.lockedAccess();
        lockable->popData(&(protocommand.get()), onlyNewest);
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
