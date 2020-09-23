import robot_remote_control as rrc
import time

print("commands")
commands=rrc.PyTransportZmq("tcp://127.0.0.1:7001",0)
commands.printConnections()
print("tele")
telemetry=rrc.PyTransportZmq("tcp://127.0.0.1:7002",3)
print("print")
commands.printConnections()
telemetry.printConnections()
print("controller")
controller=rrc.PyRobotController(commands,telemetry)
print("tthread")
controller.startUpdateThread(1000)

time.sleep(2)
print("pose")
print(controller.getCurrentPose())
print("pose_end")