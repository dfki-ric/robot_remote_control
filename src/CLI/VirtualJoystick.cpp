#include <iostream>
#include <unistd.h>
#include <ncurses.h> 
#include <memory> 
#include <cmath>

#include "RobotController.hpp"
#include "Transports/TransportZmq.hpp"
#include "ConsoleCommands.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;


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
            initController();
            showStatusWindow();
        }

        ~WindowManager()
        {
            //TODO should I stop update thread?!
            controller_->stopUpdateThread();
            deleteAllWindows();
            use_default_colors();
            standend();
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
                if(not controller_->isConnected())
                {
                    showWarning();
                    warned = true;
                } else if (warned) {
                    clearWarning();
                    refreshAllWindows();
                    warned = false;
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
            mvwprintw(warnWindow_ , 1, 2, "Connection to robot could not be established."); 
            mvwprintw(warnWindow_ , 3, 4, ipString_.c_str()); 
            mvwprintw(warnWindow_ , 4, 4, portString_.c_str()); 
        	wborder(warnWindow_, '!', '!', '-','-','+','+','+','+');
            wrefresh(warnWindow_);
        }

        void clearWarning()
        {
            werase(warnWindow_);
            wrefresh(warnWindow_);
            refreshAllWindows();
        }

        void refreshAllWindows()
        {
            for(auto entry : windowMap)
            {
                wrefresh(entry.second);
            }
        }

    private:
        std::map<std::string, WINDOW *> windowMap;
        WINDOW * warnWindow_;
        Connection connectionInfo_;
        std::string ipString_, portString_;
        std::shared_ptr<robot_remote_control::RobotController> controller_;
        robot_remote_control::Twist twistCmd_;
        robot_remote_control::RobotName robotName_;
        double increment_ = 0.1;
        bool connectionLost_;


        WINDOW * create_newwin(int height, int width, int starty, int startx)
        {
        	WINDOW *local_win;
        	local_win = newwin(height, width, starty, startx);
        	box(local_win, 0 , 0);
        	wrefresh(local_win);
        	return local_win;
        }

        void showStatusWindow()
        {
            // print status window
            std::string connected = "Connected to ";
            mvwprintw(windowMap["statusWindow"] , 1, 2, connected.c_str()); 
            attron(A_BOLD);	
            mvwprintw(windowMap["statusWindow"] , 1, 16, robotName_.value().c_str()); 
            attroff(A_BOLD);
            mvwprintw(windowMap["statusWindow"] , 2, 4, ipString_.c_str()); 
            mvwprintw(windowMap["statusWindow"] , 3, 4, portString_.c_str()); 
            wrefresh(windowMap["statusWindow"]);
        }

        void highlightButton(const std::string & windowKey, bool enable=true)
        {
            if (enable)
            {
//                attron(COLOR_PAIR(1));
//                wbkgd(windowMap[windowKey], COLOR_PAIR(1));
                mvwprintw(windowMap[windowKey] , 1, 1, "++++");
            } else {
//                attroff(COLOR_PAIR(1));
                mvwprintw(windowMap[windowKey] , 1, 1, "    ");
            }
            wrefresh(windowMap[windowKey]);
        }

        void refreshCmdWindow()
        {
            werase(windowMap["cmdWindow"]);
	        box(windowMap["cmdWindow"], 0 , 0);
            mvwprintw(windowMap["cmdWindow"] , 1, 2, "Sending Twist command:");
            mvwprintw(windowMap["cmdWindow"], 4, 4, twistCmd_.ShortDebugString().c_str());
            wrefresh(windowMap["cmdWindow"]);
        }

        void deleteAllWindows()
        {
            for(auto entry : windowMap)
            {
                delwin(entry.second);
            }
            delwin(warnWindow_);
        }

        void initWindows()
        {
            warnWindow_    = create_newwin(LINES/2.0, COLS/2.0, LINES/4.0, COLS/4.0);
            windowMap["helpWindow"]    = create_newwin(LINES/2.0, COLS/2.0, 0, 0);
            windowMap["statusWindow"]  = create_newwin(LINES/2.0, COLS/2.0, 0, COLS/2.0);
            windowMap["arrowWindow"]   = create_newwin(LINES/2.0, COLS/2.0, LINES/2.0, 0);
            windowMap["cmdWindow"]     = create_newwin(LINES/2.0, COLS/2.0, LINES/2.0, COLS/2.0);
            windowMap["arrowUp"]       = create_newwin(ceil(LINES/10.0), floor(COLS/10.0), floor(3*(LINES/4.0)-(LINES/10.0)), 2*COLS/10.0);
            windowMap["arrowDown"]     = create_newwin(ceil(LINES/10.0), floor(COLS/10.0), floor(3*(LINES/4.0)+(LINES/10.0)), 2*COLS/10.0);
            windowMap["arrowLeft"]     = create_newwin(ceil(LINES/10.0), floor(COLS/10.0), floor(3*(LINES/4.0)             ), COLS/10.0);
            windowMap["arrowRight"]    = create_newwin(ceil(LINES/10.0), floor(COLS/10.0), floor(3*(LINES/4.0)             ), 3*COLS/10.0);

            // print keys on arrow boxes
            mvwprintw(windowMap["arrowUp"]    , 0, COLS/20.0-1, "UP");
            mvwprintw(windowMap["arrowDown"]  , 0, COLS/20.0-2, "DOWN");
            mvwprintw(windowMap["arrowLeft"]  , 0, COLS/20.0-2, "LEFT");
            mvwprintw(windowMap["arrowRight"] , 0, COLS/20.0-2, "RIGHT");
        }

        void printHeartBeat()
        {
            std::string heartBeat = "HeartBeat RTT: " + std::to_string(controller_->getHeartBeatRoundTripTime());
            mvwprintw(windowMap["statusWindow"] , 5, 2, heartBeat.c_str()); 
            wrefresh(windowMap["statusWindow"]);
            //TODO set lostConnection_ to false somewhere!
        }

        void printIncrement()
        {
            mvwprintw(windowMap["arrowWindow"] , 1, 1, "Current Increment: "); 
            mvwprintw(windowMap["arrowWindow"] , 1, 21, std::to_string(increment_).c_str()); 
            wrefresh(windowMap["arrowWindow"]);
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

    WindowManager wm = WindowManager(controller_, connectionInfo);
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

    exit(0);
//	return 0;
}
