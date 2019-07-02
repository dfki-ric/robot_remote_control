#include <boost/test/unit_test.hpp>

#include <interaction-library-controlled_robot/ControlledRobot.hpp>
#include <interaction-library-controlled_robot/RobotController.hpp>
#include <interaction-library-controlled_robot/Transports/TransportZmq.hpp>
//#include <interaction-library-controlled_robot/RobotController.cpp>
#include <thread>

using namespace interaction;

TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7001",TransportZmq::REQ));
TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7002",TransportZmq::SUB));

TransportSharedPtr command = TransportSharedPtr(new TransportZmq("tcp://*:7001",interaction::TransportZmq::REP));
TransportSharedPtr telemetri = TransportSharedPtr(new TransportZmq("tcp://*:7002",interaction::TransportZmq::PUB));

BOOST_AUTO_TEST_CASE(Checking_twist_command_transfer)
{
  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

	robot.startUpdateThread(10);
  
  interaction::Twist sendtwistcommand;
  interaction::Twist receivetwistcommand;
  sendtwistcommand.mutable_angular()->set_z(0.4);
  sendtwistcommand.mutable_linear()->set_x(0.6);
  
  controller.setTwistCommand(sendtwistcommand);	
  
  receivetwistcommand = robot.getTwistCommand();
	
  robot.stopUpdateThread();
 
  BOOST_CHECK_EQUAL(sendtwistcommand.mutable_angular()->z(),receivetwistcommand.mutable_angular()->z());
  BOOST_CHECK_EQUAL(sendtwistcommand.mutable_angular()->x(),receivetwistcommand.mutable_angular()->x());
  	
  BOOST_TEST( true /* test assertion */ );


}

BOOST_AUTO_TEST_CASE(checking_target_pose)
{
  RobotController controller(commands, telemetry);
	ControlledRobot robot(command, telemetri);
	
  Position position;
  Orientation orientation;
  Pose pose;
  Pose pose2; 
	
  position.set_x(4);
  position.set_y(2);
  position.set_z(7);

  orientation.set_x(8);
  orientation.set_y(1);
  orientation.set_z(3);
  orientation.set_w(4);
    
  *(pose.mutable_position()) = position;
  *(pose.mutable_orientation()) = orientation; 
	
  robot.startUpdateThread(10);
   
  //This is not executing 	
  controller.setTargetPose(pose);
    
  pose2 = robot.getTargetPose();

  robot.stopUpdateThread();
	
  //printf("%.2f", pose.position().x());
  //printf("%.2f", pose2.position().x());
  BOOST_CHECK_EQUAL(pose.position().x(),pose2.position().x());
  BOOST_CHECK_EQUAL(pose.position().y(),pose2.position().y());
  BOOST_CHECK_EQUAL(pose.position().z(),pose2.position().z());
  BOOST_CHECK_EQUAL(pose.orientation().x(),pose2.orientation().x());
  BOOST_CHECK_EQUAL(pose.orientation().y(),pose2.orientation().y());
  BOOST_CHECK_EQUAL(pose.orientation().z(),pose2.orientation().z());
  BOOST_CHECK_EQUAL(pose.orientation().w(),pose2.orientation().w());
	
	
  BOOST_TEST( true /* test assertion */ );


}
