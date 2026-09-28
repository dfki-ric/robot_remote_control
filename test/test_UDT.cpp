#include <boost/test/unit_test.hpp>

#include "../src/Transports/UDT/TransportUDT.hpp"

#include <unistd.h>

#include <iostream>

BOOST_AUTO_TEST_CASE(check_UDT_communication) {
      // redirect UDT output messages to /dev/null
    std::streambuf* cout_sbuf = std::cout.rdbuf();
    std::ofstream   fout("/dev/null");
    std::cout.rdbuf(fout.rdbuf());

      // init communication
    robot_remote_control::TransportUDT client(robot_remote_control::TransportUDT::CLIENT,9000,"127.0.0.1");
    robot_remote_control::TransportUDT server(robot_remote_control::TransportUDT::SERVER,9000);
    usleep(100 * 1000);

      // send and receive string
    std::string msg = std::to_string(std::rand());
    server.send(msg);

    std::string buf;
    client.receive(&buf);

      // restore the original output stream buffer
    std::cout.rdbuf(cout_sbuf); 

    BOOST_CHECK(msg.size() == buf.size());
    BOOST_TEST(msg == buf);
}
