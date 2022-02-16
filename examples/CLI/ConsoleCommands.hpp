#pragma once

#include <functional>
#include <string>
#include <map>
#include <vector>
#include <readline/readline.h>
#include <readline/history.h>

class ConsoleCommands {
 public:
    ConsoleCommands();
    virtual ~ConsoleCommands();

    void readline(const std::string& prompt);

    static void registerCommand(const std::string &name, std::function<void(const std::vector<std::string> &params)> func) {
        commands[name] = func;
    }

    void runCommand(const std::string &name, const std::vector<std::string> &params) {
        std::string cmd = name;
        if (name[name.size()-1] == ' ') {
            cmd.erase(name.size()-1, 1);
        }
        auto iter = commands.find(cmd);
        if (iter != commands.end()) {
            iter->second(params);
        }
    }

    static char *match_finder(const char *text, int state);

    static char** attempted_completion_function(const char * text, int start, int);


 private:
    static std::map< std::string, std::function<void(const std::vector<std::string> &params)> > commands;

};
