#pragma once

#include <functional>
#include <string>
#include <map>
#include <vector>
#include <thread>
#include <atomic>
#include <readline/readline.h>
#include <readline/history.h>
#include <iostream>

class ConsoleCommands {
 public:
    ConsoleCommands();
    virtual ~ConsoleCommands();

    struct ParamDef {
        ParamDef(const std::string& hint, const std::string& defaultvalue):hint(hint), defaultvalues({defaultvalue}) {}
        std::string hint;
        std::vector<std::string> defaultvalues;
    };

    struct CommandDef {
        std::function<void(const std::vector<std::string> &params)> func;
        std::vector<ParamDef> params;
        std::string doc;
        bool use_thread;
    };

    bool readline(const std::string& prompt);

    static char *command_finder(const char *text, int state);

    static char *param_finder(const char *text, int state);

    static char** attempted_completion_function(const char * text, int start, int end);


    static void registerCommand(const std::string &name, const std::string &doc, std::function<void(const std::vector<std::string> &params)> func, bool use_thread = false);

    static int registerParamsForCommand(const std::string &name, const std::vector<ParamDef> &params);


    static void addParamDefaultValue(const std::string &name, const int &paramindex, const std::string &defaultvalue) {
        commands[name].params[paramindex].defaultvalues.push_back(defaultvalue);
    }

    void runCommand(std::vector<std::string> &line);

    static std::vector<std::string> parseLine(const std::string &line, bool filter_empty = false);

    void printHelp();

 private:
    static std::map< std::string, CommandDef > commands;
    static std::vector<std::string> completions;
    std::thread cmdthread;
    std::atomic<bool> thread_running;
};
