#include <boost/test/unit_test.hpp>

#include "../src/UpdateThread/UpdateThread.hpp"

using namespace robot_remote_control;

class TestThread: public UpdateThread{

  void update(){
    
  }

};



BOOST_AUTO_TEST_CASE(thread_stops_correctly)
{

      TestThread t;

      //start a 500ms thread
      t.startUpdateThread(500);
      //directly stop it

      //usleep (500000);
      t.stopUpdateThread();

}

