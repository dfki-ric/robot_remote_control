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
        ParamDef(const std::string& hint, const std::string& defaultvalue):hint(hint), defaultvalue(defaultvalue) {}
        std::string hint;
        std::string defaultvalue;
    };

    struct CommandDef {
        std::function<void(const std::vector<std::string> &params)> func;
        std::vector<ParamDef> params;
        bool use_thread;
    };

    bool readline(const std::string& prompt);

    static void registerCommand(const std::string &name, std::function<void(const std::vector<std::string> &params)> func, bool use_thread = false) {
        CommandDef def;
        def.func = func;
        def.use_thread = use_thread;
        commands[name] = def;
    }

    static int registerParamsForCommand(const std::string &name, const std::vector<ParamDef> &params) {
        if (commands.find(name) != commands.end()) {
            commands[name].params = params;
            return true;
        }
        printf("unknown command %s\n\tyou need to register the command before adding paramater definitions\n", name.c_str());
        return false;
    }

    void runCommand(const std::string &name, std::vector<std::string> &params) {
        std::string cmd = name;
        if (name[name.size()-1] == ' ') {
            cmd.erase(name.size()-1, 1);
        }
        auto iter = commands.find(cmd);
        if (iter != commands.end()) {
            while (params.size() < iter->second.params.size()) {
                std::string question = iter->second.params[params.size()].hint + "[" + iter->second.params[params.size()].defaultvalue +"]:";
                std::string param;
                std::cout << "missing paramater: " << question;
                std::getline(std::cin, param);
                if (param == "") {
                    param = iter->second.params[params.size()].defaultvalue;
                }
                params.push_back(param);
            }

            if (iter->second.use_thread) {
                thread_running = true;
                cmdthread = std::thread([&](){
                    while (thread_running) {
                        iter->second.func(params);
                    }
                });
                readline("press enter to stop\n");
                thread_running = false;
                cmdthread.join();
            } else {
                iter->second.func(params);
            }
        }
    }


    static char *match_finder(const char *text, int state);

    static char** attempted_completion_function(const char * text, int start, int);


 private:
    static std::map< std::string, CommandDef > commands;
    std::thread cmdthread;
    std::atomic<bool> thread_running;
};
