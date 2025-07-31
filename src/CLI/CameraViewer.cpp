//Include Libraries
#include<opencv2/opencv.hpp>
 

#include "RobotController.hpp"
#include "Transports/TransportZmq.hpp"
#include "ConsoleCommands.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;


struct Connection
{
    std::string ip;
    std::string commandPort;
    std::string telemetryPort;

    Connection() : ip(""), commandPort(""), telemetryPort(""){}
    Connection(const Connection& a) : ip(a.ip), commandPort(a.commandPort), telemetryPort(a.telemetryPort) {}
};



int main(int argc, char** argv)
{
    Connection connectionInfo;

    // evaluate command line arguments
    if (argc == 1) {
        connectionInfo.ip = "localhost";
        connectionInfo.commandPort = "7001";
        connectionInfo.telemetryPort = "7002";
    } else if (argc == 4) {
        connectionInfo.ip = argv[1];
        connectionInfo.commandPort = argv[2];
        connectionInfo.telemetryPort = argv[3];
    } else {
        printf("needs 0 or 3 params: ip commandport telemetryport\n");
        exit(1);
    }

    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://"+connectionInfo.ip+":"+connectionInfo.commandPort, TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://"+connectionInfo.ip+":"+connectionInfo.telemetryPort, TransportZmq::SUB));
    std::shared_ptr<robot_remote_control::RobotController> controller_ = std::make_shared<robot_remote_control::RobotController>(commands, telemetry);

    controller_->startUpdateThread(10);

    robot_remote_control::Image rrcimg;

    char key = 0;

    cv::namedWindow("Display Image", cv::WINDOW_NORMAL );
    cv::resizeWindow("Display Image", 800, 600);

    while( key != 27 ) {
        if (controller_->getImage(&rrcimg, true)){

            // TODO: switch other types, based on rrcimg.encoding() and rrcimg.step()
            //std::cout << rrcimg.encoding() << std::endl;
            if (rrcimg.encoding() != "MODE_RGB" && rrcimg.encoding() != "MODE_BGR" ) {
                std::cout << "Image received but encoding " << rrcimg.encoding() << " not supported for display "<< std::endl;
                continue;
            }
            int type = CV_8UC3;
            
            //create target structure
            cv::Mat img (rrcimg.height(), rrcimg.width(), type);
            // copy image data            
            memcpy( img.data, rrcimg.data().data(), rrcimg.data().size());

            //convert image data RGB2BGR (TODO: switch based on incoming encoding)

            int conversion = -1;
            
            if (rrcimg.encoding() == "MODE_RGB") {
                conversion = cv::COLOR_RGB2BGR;
            } //TODO else if (rrcimg.encoding() == "MODE_XY") {}

            // if conversion needed, cobvert
            if (conversion >= 0) {
                cv::cvtColor(img, img, conversion);
            }
            
            //show
            cv::imshow("Display Image", img);
        } else {
            usleep(30000);
        }


        key = cv::waitKey(30); 
    }
    
    cv::destroyAllWindows();
    

    return 0;
}


