#include <boost/test/unit_test.hpp>
#include <interaction-library-controlled_robot/RingBuffer.hpp>


/**
 * @brief helper function to fill buffer with increasing numbers
 */

void fillBuffer(unsigned int size, RingBuffer<int>& buffer){
    for (int i = 0;i<size;i++){
        buffer.pushData(i);
    }
};

/**
 * @brief helper function to check buffer content if it has increasing numbers
 * 
 */
void checkBuffer(unsigned int size, RingBuffer<int>& buffer, unsigned int startwith = 0){

    int content;
    for (int i = startwith;i<startwith+size;i++){
        
        if (buffer.popData(content)){
            BOOST_CHECK_EQUAL(content,i);
            printf("%i %i\n",content,i);
        }else{
            //should not happen
            BOOST_CHECK_EQUAL(true,false);
        }
        
    }
}


BOOST_AUTO_TEST_CASE(retrun_correctly_on_pop)
{
    RingBuffer<int> buffer(10);
    int data;

    //buffer is empty
    BOOST_CHECK_EQUAL(buffer.popData(data),false);

    data = 0;
    buffer.pushData(data);
    //now buffer is not empty
    BOOST_CHECK_EQUAL(buffer.popData(data),true);

    //buffer is empty
    BOOST_CHECK_EQUAL(buffer.popData(data),false);

}

BOOST_AUTO_TEST_CASE(retrun_correctly_on_push)
{
    RingBuffer<int> buffer(1);
    
    int i = 0;
    bool res1 = buffer.pushData(i);
    //now the buffer is full
    bool res2 = buffer.pushData(i);

    BOOST_CHECK_EQUAL(res1,true);
    BOOST_CHECK_EQUAL(res2,false);
}

BOOST_AUTO_TEST_CASE(push_pop)
{

    RingBuffer<int> buffer(5);
    
    fillBuffer(2,buffer);
    BOOST_CHECK_EQUAL(buffer.size(),2);

    checkBuffer(2,buffer);

    BOOST_CHECK_EQUAL(buffer.size(),0);

    //no new items are added, until the buffer was read, so a size5 buffer schoud have 0-4 
    fillBuffer(20,buffer);

    BOOST_CHECK_EQUAL(buffer.size(),5);

    //checks if first 5 are in order
    checkBuffer(5,buffer);

    BOOST_CHECK_EQUAL(buffer.size(),0);

}


BOOST_AUTO_TEST_CASE(preserve_content_multiuse)
{
    RingBuffer<int> buffer(10);
    
    int content;
    for (int i = 0;i<1000;i++){
        printf("%i\n",i%2);

        if (!(i%2)){
            //run first
            fillBuffer(5,buffer);
            checkBuffer(3,buffer);
            BOOST_CHECK_EQUAL(buffer.size(),2);
        }else{
            //run second
            fillBuffer(5,buffer);
            //2 left to read, startgin with 3
            checkBuffer(2,buffer,3);
            checkBuffer(5,buffer);
            BOOST_CHECK_EQUAL(buffer.size(),0);
        }   
    }
}

