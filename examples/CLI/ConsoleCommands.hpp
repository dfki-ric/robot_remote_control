#pragma once

#include <functional>
#include <string>
#include <map>
#include <readline/readline.h>
#include <readline/history.h>

class ConsoleCommands {
 public:
    ConsoleCommands();
    virtual ~ConsoleCommands();

    void readline(const std::string& prompt);

    static void registerCommand(const std::string &name, std::function<void()> func) {
        commands[name] = func;
    }

    void runCommand(const std::string &name) {
        std::string cmd = name;
        if (name[name.size()-1] == ' ') {
            cmd.erase(name.size()-1, 1);
        }
        auto iter = commands.find(cmd);
        if (iter != commands.end()) {
            iter->second();
        }
    }

    static char *match_finder(const char *text, int state);

    static char** attempted_completion_function(const char * text, int start, int);


 private:
    static std::map< std::string, std::function<void()> > commands;

};
