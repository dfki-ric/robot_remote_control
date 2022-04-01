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

//TODO use shared or unique pointers instead of raw pointers!?
//TODO Make a box for +/- and 0
//TODO better method to highlight button press (black background for window?
//TODO clearing Warning Window does not work properly
//TODO Fix button highlighing with color?

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
            //TODO remove those and work with mv(y,x,..)
            ipString_ = "IP: "+connectionInfo_.ip;
            portString_ = "Port: "+connectionInfo_.commandPort+":"+connectionInfo_.telemetryPort;

            initScreen();
            initWindows();
            updateArrowWindows();
            showStatusWindow();
            initController();

//            std::string loadString = ".";
//            while(not controller_->isConnected())
//            {
//                mvprintw(LINES/2.0, COLS/2.0,"Connecting to the robot%s", loadString);
//                loadString+=".";
//                sleep(0.1);
//            }
        }

        ~WindowManager()
        {
            //TODO should I stop update thread?!
            if (controller_->isConnected())
            {
                controller_->stopUpdateThread();
            }
            deleteAllWindows();
            use_default_colors();
//            standend();
	        endwin();
        }

        void initScreen()
        {
        	initscr();
            raw();                   // disable line buffering
        	keypad(stdscr, TRUE);	 // required for arrow keys
            curs_set(0);             // hide cursor
        	noecho();                // Don't echo() while we do getch
            timeout(200);            // clear buttons if nothing is pressed
//            start_color();
//            init_pair(1,COLOR_RED, COLOR_BLUE);
            refresh();
        }

        void initController()
        {
            controller_->startUpdateThread(0);
            controller_->setHeartBeatDuration(1);
            controller_->setupLostConnectionCallback([&](const float& time){});
            controller_->requestRobotName(&robotName_);
        }

        void run()
        {
            bool warned = false;
            int ch;
	        while((ch = getch()) != 'q')
	        {	
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

        void evaluateKey(int key)
        {
            switch(key)
	        {	case KEY_LEFT:
                    twistCmd_.mutable_linear()->set_y(twistCmd_.linear().y()-increment_);
                    controller_->setTwistCommand(twistCmd_);
                    refreshCmdWindow();
                    highlightButton("arrowLeft");
                    break;
	        	case KEY_RIGHT:
                    twistCmd_.mutable_linear()->set_y(twistCmd_.linear().y()+increment_);
                    controller_->setTwistCommand(twistCmd_);
                    refreshCmdWindow();
                    highlightButton("arrowRight");
                    break;
	        	case KEY_UP:
                    twistCmd_.mutable_linear()->set_x(twistCmd_.linear().x()+increment_);
                    controller_->setTwistCommand(twistCmd_);
                    refreshCmdWindow();
                    highlightButton("arrowUp");
                    break;
	        	case KEY_DOWN:
                    twistCmd_.mutable_linear()->set_x(twistCmd_.linear().x()-increment_);
                    controller_->setTwistCommand(twistCmd_);
                    refreshCmdWindow();
                    highlightButton("arrowDown");
                    break;
	        	case '0':
                    twistCmd_.mutable_linear()->set_x(0);
                    twistCmd_.mutable_linear()->set_y(0);
                    twistCmd_.mutable_linear()->set_z(0);
                    controller_->setTwistCommand(twistCmd_);
                    refreshCmdWindow();
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

        void showWarning()
        {
//            warnWindow_.reset(create_newwin(LINES/2.0, COLS/2.0, LINES/4.0, COLS/4.0));
            warnWindow_.reset();
            warnWindow_ = std::move(create_newwin(LINES/2.0, COLS/2.0, LINES/4.0, COLS/4.0));
            mvwprintw(warnWindow_.get() , 1, 2, "Connection to robot could not be established."); 
            mvwprintw(warnWindow_.get() , 3, 4, ipString_.c_str()); 
            mvwprintw(warnWindow_.get() , 4, 4, portString_.c_str()); 
        	wborder(warnWindow_.get(), '!', '!', '-','-','+','+','+','+');
            wrefresh(warnWindow_.get());
        }

        void clearWarning()
        {
            werase(warnWindow_.get());
            wrefresh(warnWindow_.get());
            delwin(warnWindow_.get());
//            warnWindow_.reset();
//            refreshAllWindows(); //TODO where else is this used
        }

        //TODO still needed?
        void clearAllWindows(std::map<std::string, windowPtr<WINDOW>> map)
        {
            for(auto&& entry : map)
            {
                werase(entry.second.get());
                wrefresh(entry.second.get());
            }
        }

        void refreshAllWindows()
        {
            for(auto&& entry : windowMap_)
            {
        	    box(entry.second.get(), 0 , 0);
                wrefresh(entry.second.get());
            }
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
        double increment_ = 0.1;

        windowPtr<WINDOW> create_newwin(int height, int width, int starty, int startx)
        {
            windowPtr<WINDOW> local_win( newwin(height, width, starty, startx), [](WINDOW* w) { werase(w) && delwin(w); });
        	box(local_win.get(), 0 , 0);
        	wrefresh(local_win.get());
            return local_win;
        }

        void showStatusWindow()
        {
            // print status window
            std::string connected = "Connected to ";
            mvwprintw(windowMap_["statusWindow"].get() , 1, 2, connected.c_str()); 
            wattron(windowMap_["statusWindow"].get(), A_BOLD);	
            mvwprintw(windowMap_["statusWindow"].get() , 1, 16, robotName_.value().c_str()); 
            wattroff(windowMap_["statusWindow"].get(), A_BOLD);
            mvwprintw(windowMap_["statusWindow"].get() , 2, 4, ipString_.c_str()); 
            mvwprintw(windowMap_["statusWindow"].get() , 3, 4, portString_.c_str()); 
            wrefresh(windowMap_["statusWindow"].get());
        }

        void highlightButton(const std::string & windowKey, bool enable=true)
        {
            if (enable)
            {
                attron(COLOR_PAIR(1));
//                wbkgd(windowMap_[windowKey], COLOR_PAIR(1));
                mvwprintw(windowMap_[windowKey].get() , 1, 1, "++++");
            } else {
                attroff(COLOR_PAIR(1));
                mvwprintw(windowMap_[windowKey].get() , 1, 1, "    ");
            }
            wrefresh(windowMap_[windowKey].get());
        }

        void refreshCmdWindow()
        {
            werase(windowMap_["cmdWindow"].get());
	        box(windowMap_["cmdWindow"].get(), 0 , 0);
            mvwprintw(windowMap_["cmdWindow"].get() , 1, 2, "Sending Twist command:");
            mvwprintw(windowMap_["cmdWindow"].get(), 4, 4, twistCmd_.ShortDebugString().c_str());
            wrefresh(windowMap_["cmdWindow"].get());
        }

        void deleteAllWindows()
        {

            windowMap_.clear();
            arrowMap_.clear();
            refresh();
        }

        void initWindows()
        {
//            windowMap_["helpWindow"].reset();
//            windowMap_["statusWindow"].reset();
//            windowMap_["arrowWindow"].reset();
//            windowMap_["cmdWindow"].reset();
//
            windowMap_["helpWindow"]    = std::move(create_newwin(LINES/2.0, COLS/2.0, 0, 0));
            windowMap_["statusWindow"]  = std::move(create_newwin(LINES/2.0, COLS/2.0, 0, COLS/2.0));
            windowMap_["arrowWindow"]   = std::move(create_newwin(LINES/2.0, COLS/2.0, LINES/2.0, 0));
            windowMap_["cmdWindow"]     = std::move(create_newwin(LINES/2.0, COLS/2.0, LINES/2.0, COLS/2.0));

//            windowMap_["helpWindow"].reset(create_newwin(LINES/2.0, COLS/2.0, 0, 0));
//            windowMap_["statusWindow"].reset(create_newwin(LINES/2.0, COLS/2.0, 0, COLS/2.0));
//            windowMap_["arrowWindow"].reset(create_newwin(LINES/2.0, COLS/2.0, LINES/2.0, 0));
//            windowMap_["cmdWindow"].reset(create_newwin(LINES/2.0, COLS/2.0, LINES/2.0, COLS/2.0));
              
//            windowMap_["helpWindow"]    = std::move(create_newwin(LINES/2.0, COLS/2.0, 0, 0));
//            windowMap_["statusWindow"]  = std::move(create_newwin(LINES/2.0, COLS/2.0, 0, COLS/2.0));
//            windowMap_["arrowWindow"]   = std::move(create_newwin(LINES/2.0, COLS/2.0, LINES/2.0, 0));
//            windowMap_["cmdWindow"]     = std::move(create_newwin(LINES/2.0, COLS/2.0, LINES/2.0, COLS/2.0));

//            arrowMap_["arrowUp"].reset(std::move(create_newwin(ceil(LINES/10.0), floor(COLS/10.0), floor(3*(LINES/4.0)-(LINES/10.0)), 2*COLS/10.0)));
//            arrowMap_["arrowDown"].reset(std::move(create_newwin(ceil(LINES/10.0), floor(COLS/10.0), floor(3*(LINES/4.0)+(LINES/10.0)), 2*COLS/10.0)));
//            arrowMap_["arrowLeft"].reset(std::move(create_newwin(ceil(LINES/10.0), floor(COLS/10.0), floor(3*(LINES/4.0)             ), COLS/10.0)));
//            arrowMap_["arrowRight"].reset(std::move(create_newwin(ceil(LINES/10.0), floor(COLS/10.0), floor(3*(LINES/4.0)             ), 3*COLS/10.0)));
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

    //TODO why doesnt it exit?!

//TODO distribute on other windows
//    // print info in help window
//    mvwprintw(helpWindow , 1, 2, "INFO:"); 
//    mvwprintw(helpWindow , 3, 4, "Use arrow keys to change twist command"); 
//    mvwprintw(helpWindow , 4, 4, "Use -/+ to change increment_"); 
//    mvwprintw(helpWindow , 5, 4, "Press 0 to send stop command"); 

//    // print info on cmdWindow
//    mvwprintw(cmdWindow , 1, 2, "Sending Twist command:");

//    exit(0);
	return 0;
}
