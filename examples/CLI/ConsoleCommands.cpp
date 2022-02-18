#include "ConsoleCommands.hpp"
#include <algorithm>
#include <utility>
#include <sstream>

std::map< std::string, ConsoleCommands::CommandDef > ConsoleCommands::commands;
std::vector<std::string> ConsoleCommands::completions;

ConsoleCommands::ConsoleCommands() {
    // Configure readline to auto-complete paths when the tab key is hit.
    rl_bind_key('\t', rl_complete);
    rl_attempted_completion_function = &ConsoleCommands::attempted_completion_function;
    // we always interpret a complete line
    rl_completer_word_break_characters = "";
}

ConsoleCommands::~ConsoleCommands() {}

bool ConsoleCommands::readline(const std::string& prompt) {
    char* input = ::readline(prompt.c_str());
    if (!input) return false;
    add_history(input);
    std::istringstream commandstream;
    commandstream.str(std::string(input));
    std::string command;
    std::getline(commandstream, command, ' ');
    std::string param;
    std::vector<std::string> params;

    while (std::getline(commandstream, param, ' ')) {
        params.push_back(param);
    }
    runCommand(command, params);
    free(input);
    return true;
}

char* ConsoleCommands::command_finder(const char *text, int state) {
    static std::vector< std::string >::iterator it;
    
    // state is 0 when a "word" is finished
    if ( state == 0 ) it = completions.begin();

    // check for "incomplete commands"
    while ( it != completions.end() ) {
        auto & command = *it;
        ++it;

        if ( command.rfind(text, 0) == 0 ) {
            return strdup(command.c_str());
        }
    }
    return NULL;
}

char * ConsoleCommands::param_finder(const char *text, int state) {
    static int oldcount = 0;
    static int paramsent = 0;
    // if (state == 0) {
    //     proposedparamtype = 0;
    // }
    //printf("'%i'",state);
    std::string line (text);
    std::istringstream commandstream;
    commandstream.str(line);
    std::string command;
    std::getline(commandstream, command, ' ');

    int params = std::count(line.begin(), line.end(), ' ');

    if ( params > oldcount ) {
        oldcount = params;
    }

    if (state == 0 && params > 0 && params <= commands[command].params.size()) {
        return strdup((line + "<" + commands[command].params[params-1].hint + "> ").c_str());
    }
    if (state == 1 && params > 0 && params <= commands[command].params.size()) {
        return strdup((line + commands[command].params[params-1].defaultvalue + " ").c_str());
    }
    return NULL;
}

char** ConsoleCommands::attempted_completion_function(const char * text, int start, int end) {
    // disable default completion
    rl_attempted_completion_over = 1;
    
    std::string line (text);
    std::istringstream commandstream;
    commandstream.str(line);
    std::string command;
    std::getline(commandstream, command, ' ');

    if (commands.count(command)) {
        return rl_completion_matches(text, &ConsoleCommands::param_finder);
    }
    // generate completions
    return rl_completion_matches(text, &ConsoleCommands::command_finder);
}


void ConsoleCommands::registerCommand(const std::string &name, std::function<void(const std::vector<std::string> &params)> func, bool use_thread) {
    CommandDef def;
    def.func = func;
    def.use_thread = use_thread;
    commands[name] = def;
    completions.push_back(name);
}

int ConsoleCommands::registerParamsForCommand(const std::string &name, const std::vector<ParamDef> &params) {
    if (commands.find(name) != commands.end()) {
        commands[name].params = params;
        // std::string cmd = name;
        // std::for_each(params.begin(),params.end(),[&](const ParamDef &param){cmd += " "+param.defaultvalue;});
        // completions.push_back(cmd);
        return true;
    }
    printf("unknown command %s\n\tyou need to register the command before adding paramater definitions\n", name.c_str());
    return false;
}

void ConsoleCommands::runCommand(const std::string &name, std::vector<std::string> &params) {
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