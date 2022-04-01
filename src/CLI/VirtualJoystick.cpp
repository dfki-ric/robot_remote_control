#include <cmath>
#include <functional>
#include <iostream>
#include <memory> 
#include <ncurses.h> 
#include <unistd.h>

#include "RobotController.hpp"
#include "Transports/TransportZmq.hpp"
#include "ConsoleCommands.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;

// typedef for unique_ptr with custom deleter 
// https://stackoverflow.com/questions/19053351/how-do-i-use-a-custom-deleter-with-a-stdunique-ptr-member
template<typename T>
using windowPtr = std::unique_ptr<T,std::function<void(T*)>>;

//TODO integrate virtual joystick in robot_controller
//TODO name executable rrc_virtual_joystick
//TODO split into several files

//TODO remap controls
//     space or zero = 0
//     ad = ty, ws=tx, qe=tz
//     left/right = ay, up/down=ax, Bild=az
//     +/- for increment
//     use escape and Ctrl-C to stop

// TODO use lower two windows for input visualization wasd and arrows including iteration step
//      use top two windows for info:
//          - Right hand side status: connection, heartbeat, sent Twist
//          - Left hand side tab switchable getCurrentTwist, getCurrentPose, getCurrentIMU


//    mvwprintw(helpWindow , 1, 2, "INFO:"); 
//    mvwprintw(helpWindow , 3, 4, "Use arrow keys to change twist command"); 
//    mvwprintw(helpWindow , 4, 4, "Use -/+ to change increment_"); 
//    mvwprintw(helpWindow , 5, 4, "Press 0 to send stop command"); 

struct Connection
{
    std::string ip;
    std::string commandPort;
    std::string telemetryPort;

    Connection() : ip(""), commandPort(""), telemetryPort(""){}
    Connection(const Connection& a) : ip(a.ip), commandPort(a.commandPort), telemetryPort(a.telemetryPort) {}
};

class WindowManager
{
    public:
        WindowManager(std::shared_ptr<robot_remote_control::RobotController> controller, const Connection & connectionInfo):
          connectionInfo_(connectionInfo),
          controller_(controller)
        {
            ipString_ = "IP: "+connectionInfo_.ip;
            portString_ = "Port: "+connectionInfo_.commandPort+":"+connectionInfo_.telemetryPort;

            initScreen();
            initWindows();
            initController();
            refreshAllWindows();
        }

        ~WindowManager()
        {
            if (controller_->isConnected())
            {
                controller_->stopUpdateThread();
            }
            deleteAllWindows();
            use_default_colors();
            standend(); //turn off all attributes
	        endwin();
        }

        void run()
        {
            bool warned = false;
            int ch;
	        while((ch = getch()) != 'q')
	        {	
                if ( LINES != currHeight_ || COLS != currWidth_ )
                {
                    regenerateAllWindows();
                    warned = false;
                }
                if(not controller_->isConnected() && not warned)
                {
                    deleteAllWindows();
                    showWarning();
                    warned = true;
                } else if (controller_->isConnected() && warned) {
                    clearWarning();
                    initWindows();
                    refreshAllWindows();
                    warned = false;
                } else if (not controller_->isConnected() && warned) {
                    continue;
                } else {
                    printHeartBeat();
                    printIncrement();
                    evaluateKey(ch);
                }
	        }
        }

        void refreshAllWindows()
        {
            for(auto&& entry : windowMap_)
            {
        	    box(entry.second.get(), 0 , 0);
                wrefresh(entry.second.get());
            }
            printTwistCmd();
            showStatusWindow();
            updateArrowWindows();
        }

    private:
        std::map<std::string, windowPtr<WINDOW >> windowMap_;
        std::map<std::string, windowPtr<WINDOW >> arrowMap_;
        windowPtr<WINDOW> warnWindow_;
        Connection connectionInfo_;
        std::string ipString_, portString_;
        std::shared_ptr<robot_remote_control::RobotController> controller_;
        robot_remote_control::Twist twistCmd_;
        robot_remote_control::RobotName robotName_;
        int currWidth_;
        int currHeight_;
        double increment_ = 0.1;

        windowPtr<WINDOW> create_newwin(int height, int width, int starty, int startx)
        {
            windowPtr<WINDOW> local_win( newwin(height, width, starty, startx), [](WINDOW* w) { werase(w) && wrefresh(w) && delwin(w); });
        	box(local_win.get(), 0 , 0);
        	wrefresh(local_win.get());
            return local_win;
        }

        void initScreen()
        {
        	initscr();
            currWidth_ = COLS;
            currHeight_ = LINES;
            raw();                   // disable line buffering
        	keypad(stdscr, TRUE);	 // required for arrow keys
            curs_set(0);             // hide cursor
        	noecho();                // Don't echo() while we do getch
            timeout(200);            // clear buttons if nothing is pressed
            start_color();           
            init_pair(1,COLOR_WHITE, COLOR_GREEN);
            refresh();
        }

        void initWindows()
        {
            windowMap_["helpWindow"]    = std::move(create_newwin(LINES/2.0, COLS/2.0, 0, 0));
            windowMap_["statusWindow"]  = std::move(create_newwin(LINES/2.0, COLS/2.0, 0, COLS/2.0));
            windowMap_["arrowWindow"]   = std::move(create_newwin(LINES/2.0, COLS/2.0, LINES/2.0, 0));
            windowMap_["cmdWindow"]     = std::move(create_newwin(LINES/2.0, COLS/2.0, LINES/2.0, COLS/2.0));

            arrowMap_["arrowUp"]        = std::move(create_newwin(ceil(LINES/10.0), floor(COLS/10.0), floor(3*(LINES/4.0)-(LINES/10.0)), 2*COLS/10.0));
            arrowMap_["arrowDown"]      = std::move(create_newwin(ceil(LINES/10.0), floor(COLS/10.0), floor(3*(LINES/4.0)+(LINES/10.0)), 2*COLS/10.0));
            arrowMap_["arrowLeft"]      = std::move(create_newwin(ceil(LINES/10.0), floor(COLS/10.0), floor(3*(LINES/4.0)             ), COLS/10.0));
            arrowMap_["arrowRight"]     = std::move(create_newwin(ceil(LINES/10.0), floor(COLS/10.0), floor(3*(LINES/4.0)             ), 3*COLS/10.0));
        }

        void initController()
        {
            controller_->startUpdateThread(0);
            controller_->setHeartBeatDuration(1);
            controller_->setupLostConnectionCallback([&](const float& time){});
            controller_->requestRobotName(&robotName_);
        }

        void evaluateKey(int key)
        {
            switch(key)
	        {	case KEY_LEFT:
                    twistCmd_.mutable_linear()->set_y(twistCmd_.linear().y()-increment_);
                    controller_->setTwistCommand(twistCmd_);
                    printTwistCmd();
                    highlightButton("arrowLeft");
                    break;
	        	case KEY_RIGHT:
                    twistCmd_.mutable_linear()->set_y(twistCmd_.linear().y()+increment_);
                    controller_->setTwistCommand(twistCmd_);
                    printTwistCmd();
                    highlightButton("arrowRight");
                    break;
	        	case KEY_UP:
                    twistCmd_.mutable_linear()->set_x(twistCmd_.linear().x()+increment_);
                    controller_->setTwistCommand(twistCmd_);
                    printTwistCmd();
                    highlightButton("arrowUp");
                    break;
	        	case KEY_DOWN:
                    twistCmd_.mutable_linear()->set_x(twistCmd_.linear().x()-increment_);
                    controller_->setTwistCommand(twistCmd_);
                    printTwistCmd();
                    highlightButton("arrowDown");
                    break;
	        	case '0':
                    twistCmd_.mutable_linear()->set_x(0);
                    twistCmd_.mutable_linear()->set_y(0);
                    twistCmd_.mutable_linear()->set_z(0);
                    controller_->setTwistCommand(twistCmd_);
                    printTwistCmd();
                    break;
                case '+':
                    increment_+=0.1;
                    break;
                case '-':
                    increment_-=0.1;
                    break;
                default:
                    highlightButton("arrowRight", false);
                    highlightButton("arrowLeft", false);
                    highlightButton("arrowDown", false);
                    highlightButton("arrowUp", false);
                    break;
	        }
        }

        void highlightButton(const std::string & windowKey, bool enable=true)
        {
            if (enable)
            {
                wbkgd(arrowMap_[windowKey].get(), COLOR_PAIR(1));
            } else {
                wbkgd(arrowMap_[windowKey].get(), COLOR_PAIR(0));
            }
            wrefresh(arrowMap_[windowKey].get());
        }

        void showWarning()
        {
            warnWindow_ = std::move(create_newwin(LINES/2.0, COLS/2.0, LINES/4.0, COLS/4.0));
            mvwprintw(warnWindow_.get() , 1, 2, "Connection to robot could not be established."); 
            mvwprintw(warnWindow_.get() , 3, 4, ipString_.c_str()); 
            mvwprintw(warnWindow_.get() , 4, 4, portString_.c_str()); 
        	wborder(warnWindow_.get(), '!', '!', '-','-','+','+','+','+');
            wrefresh(warnWindow_.get());
        }

        void clearWarning()
        {
            warnWindow_.reset();
        }

        void regenerateAllWindows()
        {
            currHeight_ = LINES;
            currWidth_ = COLS;
            deleteAllWindows();
            initWindows();
            refreshAllWindows();
        }

        void showStatusWindow()
        {
            std::string connected = "Connected to ";
            mvwprintw(windowMap_["statusWindow"].get() , 1, 2, connected.c_str()); 
            wattron(windowMap_["statusWindow"].get(), A_BOLD);	
            mvwprintw(windowMap_["statusWindow"].get() , 1, 16, robotName_.value().c_str()); 
            wattroff(windowMap_["statusWindow"].get(), A_BOLD);
            mvwprintw(windowMap_["statusWindow"].get() , 2, 4, ipString_.c_str()); 
            mvwprintw(windowMap_["statusWindow"].get() , 3, 4, portString_.c_str()); 
            wrefresh(windowMap_["statusWindow"].get());
        }

        void updateArrowWindows()
        {
            mvwprintw(arrowMap_["arrowUp"].get()    , 0, COLS/20.0-1, "UP");
            mvwprintw(arrowMap_["arrowDown"].get()  , 0, COLS/20.0-2, "DOWN");
            mvwprintw(arrowMap_["arrowLeft"].get()  , 0, COLS/20.0-2, "LEFT");
            mvwprintw(arrowMap_["arrowRight"].get() , 0, COLS/20.0-2, "RIGHT");

            for(auto&& entry : arrowMap_)
            {
                box(entry.second.get(), 0, 0);
                wrefresh(entry.second.get());
            }
        }

        void deleteAllWindows()
        {
            windowMap_.clear();
            arrowMap_.clear();
            refresh();
        }

        void printTwistCmd()
        {
            mvwprintw(windowMap_["cmdWindow"].get() , 1, 2, "Sending Twist command:");
            mvwprintw(windowMap_["cmdWindow"].get(), 4, 4, twistCmd_.ShortDebugString().c_str());
            wrefresh(windowMap_["cmdWindow"].get());
        }

        void printHeartBeat()
        {
            std::string heartBeat = "HeartBeat RTT: " + std::to_string(controller_->getHeartBeatRoundTripTime());
            mvwprintw(windowMap_["statusWindow"].get() , 5, 2, heartBeat.c_str()); 
            wrefresh(windowMap_["statusWindow"].get());
        }

        void printIncrement()
        {
            mvwprintw(windowMap_["arrowWindow"].get() , 1, 1, "Current Increment: "); 
            mvwprintw(windowMap_["arrowWindow"].get() , 1, 21, std::to_string(increment_).c_str()); 
            wrefresh(windowMap_["arrowWindow"].get());
        }
};


int main(int argc, char** argv)
{
    // store connection values
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

    WindowManager wm(controller_, connectionInfo);
    wm.refreshAllWindows();
    wm.run();

	return 0;
}
