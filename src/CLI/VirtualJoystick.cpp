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

// TODO add more info:
//  switchable getCurrentTwist, getCurrentPose, getCurrentIMU

struct Connection
{
    std::string ip;
    std::string commandPort;
    std::string telemetryPort;

    Connection() : ip(""), commandPort(""), telemetryPort(""){}
    Connection(const Connection& a) : ip(a.ip), commandPort(a.commandPort), telemetryPort(a.telemetryPort) {}
};

class VirtualJoystick
{
    public:
        VirtualJoystick(std::shared_ptr<robot_remote_control::RobotController> controller, const Connection & connectionInfo):
          connectionInfo_(connectionInfo),
          controller_(controller)
        {
            initScreen();
            initWindows();
            initController();
            refreshAllWindows();
        }

        ~VirtualJoystick()
        {
            controller_->stopUpdateThread();
            deleteAllWindows();
            resetScreen();
            endwin();
        }

        void run()
        {
            bool warned = false;
            int ch;
            while(ch = getch())
            {    
                // Exit if ESC is pressed
                if (ch == 27) {
                    //TODO should zero command be sent?!
                    //sendStopCommand();
                    break;
                }

                // resize if needed
                if ( LINES != currHeight_ || COLS != currWidth_ )
                {
                    regenerateAllWindows();
                    warned = false;
                }

                // warn if robot is not connected
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

    private:
        std::map<std::string, windowPtr<WINDOW >> windowMap_;
        std::map<std::string, windowPtr<WINDOW >> arrowMap_;
        windowPtr<WINDOW> warnWindow_;
        Connection connectionInfo_;
        std::shared_ptr<robot_remote_control::RobotController> controller_;
        robot_remote_control::Twist twistCmd_;
        robot_remote_control::RobotName robotName_;
        int currWidth_;
        int currHeight_;
        double increment_ = 0.01;

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
            keypad(stdscr, TRUE);     // required for arrow keys
            curs_set(0);             // hide cursor
            noecho();                // Don't echo() while we do getch
            timeout(200);            // clear buttons if nothing is pressed
            start_color();           
            init_pair(1,COLOR_WHITE, COLOR_GREEN);
            refresh();
        }

        void resetScreen()
        {
            use_default_colors();
            standend(); //turn off all attributes
            curs_set(1);
            echo();
            refresh();
        }

        void initController()
        {
            controller_->startUpdateThread(0);
            controller_->setHeartBeatDuration(1);
            controller_->setupLostConnectionCallback([&](const float& time){});
            controller_->requestRobotName(&robotName_);
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

            drawHelpInfo();
        }

        void refreshAllWindows()
        {
            for(auto&& entry : windowMap_)
            {
                box(entry.second.get(), 0 , 0);
                wrefresh(entry.second.get());
            }
            printTwistCmd();
            drawStatusWindow();
            drawArrowWindows();
        }

        void regenerateAllWindows()
        {
            currHeight_ = LINES;
            currWidth_ = COLS;
            deleteAllWindows();
            initWindows();
            refreshAllWindows();
        }

        void deleteAllWindows()
        {
            windowMap_.clear();
            arrowMap_.clear();
            refresh();
        }

        void showWarning()
        {
            warnWindow_ = std::move(create_newwin(LINES/2.0, COLS/2.0, LINES/4.0, COLS/4.0));
            wattron(warnWindow_.get(), A_BOLD);    
            mvwprintw(warnWindow_.get(), 1, 2, "Connection to robot could not be established."); 
            wattroff(warnWindow_.get(), A_BOLD);    
            mvwprintw(warnWindow_.get(), 4, 4, "At"); 
            wattron(warnWindow_.get(), A_UNDERLINE);    
            mvwprintw(warnWindow_.get(), 4, 7, "%s", connectionInfo_.ip.c_str());
            wattroff(warnWindow_.get(), A_UNDERLINE);    
            mvwprintw(warnWindow_.get(), 4, 8 + connectionInfo_.ip.length(), "on ports");
            wattron(warnWindow_.get(), A_UNDERLINE);    
            mvwprintw(warnWindow_.get(), 4, 17 + connectionInfo_.ip.length(), "%s", (connectionInfo_.commandPort+":"+connectionInfo_.telemetryPort).c_str());
            wattroff(warnWindow_.get(), A_UNDERLINE);    
            wborder(warnWindow_.get(), '!', '!', '-','-','+','+','+','+');
            wrefresh(warnWindow_.get());
        }

        void clearWarning()
        {
            warnWindow_.reset();
        }

        void evaluateKey(int key)
        {
            switch(key)
            {    case KEY_LEFT:
                    twistCmd_.mutable_angular()->set_z(roundf((twistCmd_.angular().z()+increment_)*100)/100);
                    controller_->setTwistCommand(twistCmd_);
                    printTwistCmd();
                    highlightButton("arrowLeft");
                    break;
                case KEY_RIGHT:
                    twistCmd_.mutable_angular()->set_z(roundf((twistCmd_.angular().z()-increment_)*100)/100);
                    controller_->setTwistCommand(twistCmd_);
                    printTwistCmd();
                    highlightButton("arrowRight");
                    break;
                case KEY_UP:
                    twistCmd_.mutable_linear()->set_x(roundf((twistCmd_.linear().x()+increment_)*100)/100);
                    controller_->setTwistCommand(twistCmd_);
                    printTwistCmd();
                    highlightButton("arrowUp");
                    break;
                case KEY_DOWN:
                    twistCmd_.mutable_linear()->set_x(roundf((twistCmd_.linear().x()-increment_)*100)/100);
                    controller_->setTwistCommand(twistCmd_);
                    printTwistCmd();
                    highlightButton("arrowDown");
                    break;
                case ' ':
                    sendStopCommand();
                    printTwistCmd();
                    break;
                case '+':
                    increment_+=increment_;
                    break;
                case '-':
                    increment_-=increment_/2.0;
                    break;
                default:
                    highlightButton("arrowRight", false);
                    highlightButton("arrowLeft", false);
                    highlightButton("arrowDown", false);
                    highlightButton("arrowUp", false);
                    break;
            }
        }

        void sendStopCommand()
        {
            twistCmd_.mutable_linear()->set_x(0.0);
            twistCmd_.mutable_linear()->set_y(0.0);
            twistCmd_.mutable_linear()->set_z(0.0);
            twistCmd_.mutable_angular()->set_x(0.0);
            twistCmd_.mutable_angular()->set_y(0.0);
            twistCmd_.mutable_angular()->set_z(0.0);
            controller_->setTwistCommand(twistCmd_);
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

        void drawArrowWindows()
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

        void drawHelpInfo() {
            wattron(windowMap_["helpWindow"].get(), A_BOLD);    
            mvwprintw(windowMap_["helpWindow"].get(), 1, 2, "INFO:"); 
            wattroff(windowMap_["helpWindow"].get(), A_BOLD);    
            mvwprintw(windowMap_["helpWindow"].get(), 3, 4, "ARROW KEYS => TWIST COMMAND"); 
            mvwprintw(windowMap_["helpWindow"].get(), 4, 4, "SPACE      => STOP COMMAND"); 
            mvwprintw(windowMap_["helpWindow"].get(), 5, 4, "-/+        => CHANGE INCREMENT"); 
            mvwprintw(windowMap_["helpWindow"].get(), 6, 4, "ESC        => QUIT"); 
        }

        void drawStatusWindow()
        {
            wattron(windowMap_["statusWindow"].get(), A_BOLD);    
            std::string connected = "Connected to ";
            mvwprintw(windowMap_["statusWindow"].get() , 1, 2, "%s", connected.c_str()); 
            mvwprintw(windowMap_["statusWindow"].get() , 1, 15, "%s", robotName_.value().c_str()); 
            wattroff(windowMap_["statusWindow"].get(), A_BOLD);    
            mvwprintw(windowMap_["statusWindow"].get() , 3, 4, "At"); 
            wattron(windowMap_["statusWindow"].get(), A_UNDERLINE);    
            mvwprintw(windowMap_["statusWindow"].get() , 3, 7, "%s", connectionInfo_.ip.c_str());
            wattroff(windowMap_["statusWindow"].get(), A_UNDERLINE);    
            mvwprintw(windowMap_["statusWindow"].get() , 3, 8 + connectionInfo_.ip.length(), "on ports");
            wattron(windowMap_["statusWindow"].get(), A_UNDERLINE);    
            mvwprintw(windowMap_["statusWindow"].get() , 3, 17 + connectionInfo_.ip.length(), "%s", (connectionInfo_.commandPort+":"+connectionInfo_.telemetryPort).c_str());
            wattroff(windowMap_["statusWindow"].get(), A_UNDERLINE);    
            wrefresh(windowMap_["statusWindow"].get());
        }

        void printHeartBeat()
        {
            std::string heartBeat = "HeartBeat RTT: " + std::to_string(controller_->getHeartBeatRoundTripTime());
            mvwprintw(windowMap_["statusWindow"].get() , 5, 4, "%s", heartBeat.c_str()); 
            wrefresh(windowMap_["statusWindow"].get());
        }

        void printIncrement()
        {
            wattron(windowMap_["arrowWindow"].get(), A_BOLD);    
            mvwprintw(windowMap_["arrowWindow"].get() , 1, 1, "Current Increment: "); 
            mvwprintw(windowMap_["arrowWindow"].get() , 1, 21, "%s", std::to_string(increment_).c_str()); 
            wrefresh(windowMap_["arrowWindow"].get());
        }

        void printTwistCmd()
        {
            wattron(windowMap_["cmdWindow"].get(), A_BOLD);    
            mvwprintw(windowMap_["cmdWindow"].get(), 1, 2, "Sending Twist command:");
            wattroff(windowMap_["cmdWindow"].get(), A_BOLD);    
            wmove(windowMap_["cmdWindow"].get(), 4, 4);
            wclrtoeol(windowMap_["cmdWindow"].get());
            mvwprintw(windowMap_["cmdWindow"].get(), 4, 4, "%s", twistCmd_.ShortDebugString().c_str());
            wrefresh(windowMap_["cmdWindow"].get());
        }
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

    VirtualJoystick joy(controller_, connectionInfo);
    joy.run();

//TODO DEBUG not properly closing if no connection was ever made!
//    VirtualJoystick * joy = new VirtualJoystick(controller_, connectionInfo);
//    joy->run();
//    delete joy;
//    std::cout << "Finished deletion joystick" << std::endl;
//    controller_.reset();
//    std::cout << "Finished deletion of controller shared ptr" << std::endl;
//    commands.reset();
//    telemetry.reset();
//    std::cout << "Finished reseting TransportSharedPtr" << std::endl;

    return 0;
}
