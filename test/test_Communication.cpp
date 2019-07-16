#include <boost/test/unit_test.hpp>

#include <interaction-library-controlled_robot/ControlledRobot.hpp>
#include <interaction-library-controlled_robot/RobotController.hpp>
#include <interaction-library-controlled_robot/Transports/TransportZmq.hpp>


using namespace controlledRobot;
 
TransportSharedPtr commands;
TransportSharedPtr telemetry;

TransportSharedPtr command;
TransportSharedPtr telemetri;


#define COMPARE_PROTOBUF(VAR1,VAR2) BOOST_TEST(VAR1.SerializeAsString() == VAR2.SerializeAsString())

/**
 * @brief used to init the communication only when it is used, it is reused then.
 * 
 */
void initComms(){
  
  if (!commands.get()){commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7003",TransportZmq::REQ));}
  if (!telemetry.get()){telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7004",TransportZmq::SUB));}

  if (!command.get()){command = TransportSharedPtr(new TransportZmq("tcp://*:7003",TransportZmq::REP));}
  if (!telemetri.get()){telemetri = TransportSharedPtr(new TransportZmq("tcp://*:7004",TransportZmq::PUB));}
}


BOOST_AUTO_TEST_CASE(Checking_twist_command_transfer)
{
  initComms();

  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

	robot.startUpdateThread(10);
  
  Twist sendtwistcommand;
  Twist receivetwistcommand;
  sendtwistcommand.mutable_angular()->set_z(0.4);
  sendtwistcommand.mutable_linear()->set_x(0.6);
  
  controller.setTwistCommand(sendtwistcommand);	
  
  receivetwistcommand = robot.getTwistCommand();
	
  robot.stopUpdateThread();
 
  COMPARE_PROTOBUF(sendtwistcommand,receivetwistcommand);

}

BOOST_AUTO_TEST_CASE(checking_target_pose)
{
  initComms();
  
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

  controller.setTargetPose(pose);
    
  pose2 = robot.getTargetPose();

  robot.stopUpdateThread();
	

  COMPARE_PROTOBUF(pose,pose2);
	

}

BOOST_AUTO_TEST_CASE(checking_current_pose)
{

  initComms();
  RobotController controller(commands, telemetry);
  ControlledRobot robot(command, telemetri);

  Position position;
  Orientation orientation;
  Pose pose;
  Pose currentpose;

  position.set_x(4);
  position.set_y(2);
  position.set_z(7);

  orientation.set_x(8);
  orientation.set_y(1);
  orientation.set_z(3);
  orientation.set_w(4);
   
  *(pose.mutable_position()) = position;
  *(pose.mutable_orientation()) = orientation;

  //buffer for size comparsion
  std::string buf;
  pose.SerializeToString(&buf);
 
  //send tememetry data
  int sent = robot.setCurrentPose(pose);
    
  //wait a little for data transfer
  //the Telemetry send is non-blocking in opposite to commands
  usleep(100 * 1000);
  //receive pending data
  controller.update();

  controller.getCurrentPose(currentpose);
 
  
  //data was sent completely
  BOOST_CHECK((unsigned int)sent == buf.size());
  //and is the same
  COMPARE_PROTOBUF(pose,currentpose);


}
