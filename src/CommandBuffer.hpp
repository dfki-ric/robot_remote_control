# pragma once

#include <vector>
#include <string>
#include "RingBuffer.hpp"
#include "MessageTypes.hpp"

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
        write(protocommand);
        return true;
    }

    int hasNew() {
        return buffer.lockedAccess()->size();
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
        auto protocommand = lastcommand.lockedAccess();
        bool oldval = read(&protocommand.get(), onlyNewest);
        protocommand->SerializeToString(receivedMessage);
        return oldval;
    }

 private:
    std::atomic<bool> isnew;
    LockableClass<RingBuffer<COMMAND>> buffer;
    LockableClass<COMMAND> lastcommand;
};

struct MessageIdCommandBuffer: public CommandBufferBase{
 public:
    explicit MessageIdCommandBuffer(const size_t & buffersize):isnew(false), buffer(RingBuffer<MessageId>(buffersize)) {}

    virtual ~MessageIdCommandBuffer() {}

    void write(const MessageId &src) {
        {
            auto lockable = buffer.lockedAccess();
            lockable->pushData(src, true);
            isnew.store(lockable->size());
        }
        notify();
    }

    virtual bool write(const std::string &serializedMessage) {
        write(std::atoi(serializedMessage.c_str()));
        return true;
    }

    int hasNew() {
        return buffer.lockedAccess()->size();
    }

    bool read(MessageId *target, bool onlyNewest = true) {
        bool oldval = isnew.load();
        auto lockable = buffer.lockedAccess();
        if (!lockable->popData(target, onlyNewest)) {
            auto protocommand = lastcommand.lockedAccess();
            *target = protocommand.get();
        }
        isnew.store(lockable->size());
        return oldval;
    }

    virtual bool read(std::string *receivedMessage, bool onlyNewest = true) {
        MessageId id;
        bool res = read(&id, onlyNewest);
        *receivedMessage = std::to_string(id);
        return res;
    }

 private:
    std::atomic<bool> isnew;
    LockableClass<RingBuffer<MessageId>> buffer;
    LockableClass<MessageId> lastcommand;
};

}  // namespace robot_remote_control
