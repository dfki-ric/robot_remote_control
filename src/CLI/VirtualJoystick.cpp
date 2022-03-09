#include <iostream>
#include <unistd.h>
#include <ncurses.h> 
#include <cmath>

#include "RobotController.hpp"
#include "Transports/TransportZmq.hpp"
#include "ConsoleCommands.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;


//TODO use shared or unique pointers instead of raw pointers!?
//TODO catch and display if not connected to robot: https://www.asciiart.eu/electronics/electronics || https://www.asciiart.eu/electronics/robots
//TODO Make a box for +/- and 0
//TODO Clean UP!
//TODO make function to reset/redraw cmdWindow
//TODO better method to highlight button press (black background for window?

WINDOW * create_newwin(int height, int width, int starty, int startx)
{
	WINDOW *local_win;
	local_win = newwin(height, width, starty, startx);
	box(local_win, 0 , 0);
	wrefresh(local_win);
	return local_win;
}


int main(int argc, char** argv)
{
    // store connection values
    std::string ip;
    std::string commandport;
    std::string telemetryport;

    // evaluate command line arguments
    if (argc == 1) {
        ip = "localhost";
        commandport = "7001";
        telemetryport = "7002";
    } else if (argc == 4) {
        ip = argv[1];
        commandport = argv[2];
        telemetryport = argv[3];
    } else {
        printf("needs 0 or 3 params: ip commandport telemetryport\n");
        exit(1);
    }

    // connect to robot
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://"+ip+":"+commandport, TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://"+ip+":"+telemetryport, TransportZmq::SUB));
    robot_remote_control::RobotController controller(commands, telemetry);
    controller.startUpdateThread(0);
    controller.setHeartBeatDuration(1);

    robot_remote_control::Twist twist;
    robot_remote_control::RobotName robotName;
    controller.requestRobotName(&robotName);

    // init ncurses
	initscr();
    raw();                   //disable line buffering
	keypad(stdscr, TRUE);	 // required for arrow keys
    curs_set(0);
	noecho();                // Don't echo() while we do getch
    timeout(200);            // clear buttons if nothing is pressed
    refresh();

    double increment = 0.1;
	int ch;
	int height = (LINES/2.0);
	int width = (COLS/2.0);

    // create windows
    WINDOW * helpWindow    = create_newwin(height, width, 0, 0);
    WINDOW * statusWindow  = create_newwin(height, width, 0, COLS/2.0);
    WINDOW * arrowWindow   = create_newwin(height, width, LINES/2.0, 0);
    WINDOW * cmdWindow     = create_newwin(height, width, LINES/2.0, COLS/2.0);
    WINDOW * arrowUp       = create_newwin(ceil(height/5.0), floor(width/5.0), floor(3*(height/2.0)-(height/5.0)), 2*width/5.0);
    WINDOW * arrowDown     = create_newwin(ceil(height/5.0), floor(width/5.0), floor(3*(height/2.0)+(height/5.0)), 2*width/5.0);
    WINDOW * arrowLeft     = create_newwin(ceil(height/5.0), floor(width/5.0), floor(3*(height/2.0)             ), width/5.0);
    WINDOW * arrowRight    = create_newwin(ceil(height/5.0), floor(width/5.0), floor(3*(height/2.0)             ), 3*width/5.0);

    // print info in help window
    mvwprintw(helpWindow , 1, 2, "INFO:"); 
    mvwprintw(helpWindow , 3, 4, "Use arrow keys to change twist command"); 
    mvwprintw(helpWindow , 4, 4, "Use -/+ to change increment"); 
    mvwprintw(helpWindow , 5, 4, "Press 0 to send stop command"); 

    // print status window
    std::string status1 = "Connected to " +  robotName.value();
    std::string status2 = "IP: " + ip;
    std::string status3 = "Ports: " + commandport + ":" + telemetryport;
    mvwprintw(statusWindow , 1, 2, status1.c_str()); 
    mvwprintw(statusWindow , 2, 4, status2.c_str()); 
    mvwprintw(statusWindow , 3, 4, status3.c_str()); 

    // print keys on arrow boxes
    mvwprintw(arrowUp    , 0, width/10.0-1, "UP");
    mvwprintw(arrowDown  , 0, width/10.0-2, "DOWN");
    mvwprintw(arrowLeft  , 0, width/10.0-2, "LEFT");
    mvwprintw(arrowRight , 0, width/10.0-2, "RIGHT");

    // print info on cmdWindow
    mvwprintw(cmdWindow , 1, 2, "Sending Twist command:");

    // refresh windows
    wrefresh(helpWindow);
    wrefresh(statusWindow);
    wrefresh(arrowWindow);
    wrefresh(cmdWindow);
    wrefresh(arrowUp);
    wrefresh(arrowDown);
    wrefresh(arrowLeft);
    wrefresh(arrowRight);

    // start update loop
	while((ch = getch()) != 'q')
	{	

        std::string heartBeat = "HeartBeat RTT: " + std::to_string(controller.getHeartBeatRoundTripTime());
        mvwprintw(statusWindow , 5, 2, heartBeat.c_str()); 
        wrefresh(statusWindow);

        mvwprintw(arrowWindow , 1, 1, "Current Increment: "); 
        mvwprintw(arrowWindow , 1, 21, std::to_string(increment).c_str()); 
        wrefresh(arrowWindow);
        
        switch(ch)
		{	case KEY_LEFT:
                twist.mutable_linear()->set_y(twist.linear().y()-increment);
                controller.setTwistCommand(twist);
                werase(cmdWindow);
	            box(cmdWindow, 0 , 0);
                mvwprintw(cmdWindow , 1, 2, "Sending Twist command:");
                mvwprintw(cmdWindow, 4, 4, twist.ShortDebugString().c_str());
                mvwprintw(arrowLeft , 1, 1, "++++");
                wrefresh(cmdWindow);
                wrefresh(arrowLeft);
                break;
			case KEY_RIGHT:
                twist.mutable_linear()->set_y(twist.linear().y()+increment);
                controller.setTwistCommand(twist);
                werase(cmdWindow);
	            box(cmdWindow, 0 , 0);
                mvwprintw(cmdWindow , 1, 2, "Sending Twist command:");
                mvwprintw(cmdWindow, 4, 4, twist.ShortDebugString().c_str());
                mvwprintw(arrowRight , 1, 1, "++++");
                wrefresh(cmdWindow);
                wrefresh(arrowRight);
                break;
			case KEY_UP:
                twist.mutable_linear()->set_x(twist.linear().x()+increment);
                controller.setTwistCommand(twist);
                werase(cmdWindow);
	            box(cmdWindow, 0 , 0);
                mvwprintw(cmdWindow , 1, 2, "Sending Twist command:");
                mvwprintw(cmdWindow, 4, 4, twist.ShortDebugString().c_str());
                mvwprintw(arrowUp , 1, 1, "++++");
                wrefresh(cmdWindow);
                wrefresh(arrowUp);
                break;
			case KEY_DOWN:
                twist.mutable_linear()->set_x(twist.linear().x()-increment);
                controller.setTwistCommand(twist);
                werase(cmdWindow);
	            box(cmdWindow, 0 , 0);
                mvwprintw(cmdWindow , 1, 2, "Sending Twist command:");
                mvwprintw(cmdWindow, 4, 4, twist.ShortDebugString().c_str());
                mvwprintw(arrowDown , 1, 1, "++++");
                wrefresh(cmdWindow);
                wrefresh(arrowDown);
                break;
			case '0':
                twist.mutable_linear()->set_x(0);
                twist.mutable_linear()->set_y(0);
                twist.mutable_linear()->set_z(0);
                controller.setTwistCommand(twist);
                werase(cmdWindow);
	            box(cmdWindow, 0 , 0);
                mvwprintw(cmdWindow , 1, 2, "Sending Twist command:");
                mvwprintw(cmdWindow, 4, 4, twist.ShortDebugString().c_str());
                mvwprintw(arrowDown , 1, 1, "++++");
                wrefresh(cmdWindow);
                wrefresh(arrowDown);
                break;
            case '+':
                increment+=0.1;
                break;
            case '-':
                increment-=0.1;
                break;
            default:
                mvwprintw(arrowRight , 1, 1, "    ");
                mvwprintw(arrowLeft  , 1, 1, "    ");
                mvwprintw(arrowUp    , 1, 1, "    ");
                mvwprintw(arrowDown  , 1, 1, "    ");
                wrefresh(arrowRight);
                wrefresh(arrowLeft);
                wrefresh(arrowUp);
                wrefresh(arrowDown);
                break;
		}
	}

    delwin(helpWindow);
    delwin(statusWindow);
    delwin(arrowWindow);
    delwin(cmdWindow);
    delwin(arrowUp);
    delwin(arrowDown);
    delwin(arrowLeft);
    delwin(arrowRight);

	endwin();

    controller.stopUpdateThread();
	return 0;
}
