#include <boost/test/unit_test.hpp>
#include "../src/RingBuffer.hpp"


namespace robot_remote_control {

/**
 * @brief helper function to fill buffer with increasing numbers
 */

void fillBuffer(unsigned int size, RingBuffer<int> *buffer, bool overwrite = false) {
    for (unsigned int i = 0; i < size; i++) {
        buffer->pushData(i, overwrite);
    }
}

/**
 * @brief helper function to check buffer content if it has increasing numbers
 * as defne to get proper line numbers wen errors occur
 */
#define CHECK_BUFFER(SIZE, BUFFER, STARTWITH) \
    { \
        int content; \
        for (unsigned int i = STARTWITH; i < STARTWITH+SIZE; i++) { \
            if (BUFFER.popData(&content)) { \
                BOOST_CHECK_EQUAL(content, i); \
            } else { \
                BOOST_CHECK_EQUAL(true, false); \
            } \
        } \
    }


BOOST_AUTO_TEST_CASE(return_correctly_on_pop) {
    RingBuffer<int> buffer(10);
    int data;

    // buffer is empty
    BOOST_CHECK_EQUAL(buffer.popData(&data), false);

    data = 0;
    buffer.pushData(data);
    // now buffer is not empty
    BOOST_CHECK_EQUAL(buffer.popData(&data), true);

    // buffer is empty
    BOOST_CHECK_EQUAL(buffer.popData(&data), false);
}

BOOST_AUTO_TEST_CASE(return_correctly_on_push) {
    RingBuffer<int> buffer(1);

    int i = 0;
    bool res1 = buffer.pushData(i);
    // now the buffer is full
    bool res2 = buffer.pushData(i);

    BOOST_CHECK_EQUAL(res1, true);
    BOOST_CHECK_EQUAL(res2, false);
}

BOOST_AUTO_TEST_CASE(push_pop) {
    RingBuffer<int> buffer(5);

    fillBuffer(2, &buffer);
    BOOST_CHECK_EQUAL(buffer.size(), 2);

    CHECK_BUFFER(2, buffer, 0);

    BOOST_CHECK_EQUAL(buffer.size(), 0);

    // no new items are added, until the buffer was read, so a size5 buffer schoud have 0-4
    fillBuffer(20, &buffer);

    BOOST_CHECK_EQUAL(buffer.size(), 5);

    // checks if first 5 are in order
    CHECK_BUFFER(5, buffer, 0);

    BOOST_CHECK_EQUAL(buffer.size(), 0);
}

BOOST_AUTO_TEST_CASE(push_pop_overwrite) {
    RingBuffer<int> buffer(5);

    fillBuffer(2, &buffer, true);
    BOOST_CHECK_EQUAL(buffer.size(), 2);

    CHECK_BUFFER(2, buffer, 0);

    BOOST_CHECK_EQUAL(buffer.size(), 0);

    // oldest items are overwritten so a size 5 buffer schoud have 15-19
    fillBuffer(20, &buffer, true);

    BOOST_CHECK_EQUAL(buffer.size(), 5);

    // checks if first 5 are in order
    CHECK_BUFFER(5, buffer, 15);

    BOOST_CHECK_EQUAL(buffer.size(), 0);
}


BOOST_AUTO_TEST_CASE(preserve_content_multiuse) {
    RingBuffer<int> buffer(10);

    for (int i = 0; i < 1000; i++) {
        if (!(i % 2)) {
            // run first
            fillBuffer(5, &buffer);
            CHECK_BUFFER(3, buffer, 0);
            BOOST_CHECK_EQUAL(buffer.size(), 2);
        } else {
            // run second
            fillBuffer(5, &buffer);
            // 2 left to read, startgin with 3
            CHECK_BUFFER(2, buffer, 3);
            CHECK_BUFFER(5, buffer, 0);
            BOOST_CHECK_EQUAL(buffer.size(), 0);
        }
    }
}

BOOST_AUTO_TEST_CASE(resize_buffer) {
    RingBuffer<int> buffer(1);

    BOOST_CHECK_EQUAL(buffer.capacity(), 1);

    fillBuffer(5, &buffer);  // further adds fail
    CHECK_BUFFER(1,  buffer, 0);  // has first value, push fails if buffer is full

    fillBuffer(5, &buffer, true);  // fill with overwrite
    CHECK_BUFFER(1, buffer, 4);  // has first value, push fails if buffer is full

    buffer.resize(10);
    BOOST_CHECK_EQUAL(buffer.capacity(), 10);

    fillBuffer(10, &buffer);  // fill
    CHECK_BUFFER(10, buffer, 0);  // check

    fillBuffer(10, &buffer);  // fill again
    buffer.resize(5);
    BOOST_CHECK_EQUAL(buffer.capacity(), 5);
    CHECK_BUFFER(5, buffer, 0);  // has oldest values (resize chops at the end of buffer)

    // works as before
    fillBuffer(5, &buffer);
    CHECK_BUFFER(5, buffer, 0);
}

}  // namespace robot_remote_control
