#include "ConsoleCommands.hpp"
#include <algorithm>
#include <utility>
#include <sstream>

std::map< std::string, ConsoleCommands::CommandDef > ConsoleCommands::commands;
int ConsoleCommands::command_word_counter;

ConsoleCommands::ConsoleCommands() {
    command_word_counter = 0;
    // Configure readline to auto-complete paths when the tab key is hit.
    rl_bind_key('\t', rl_complete);
    rl_attempted_completion_function = &ConsoleCommands::attempted_completion_function;
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
    command_word_counter = 0;
    free(input);
    return true;
}

char* ConsoleCommands::match_finder(const char *text, int state) {
    static std::map< std::string, CommandDef >::iterator it;
    // state is 0 when a "word" is finished
    if ( state == 0 ){
        it = begin(commands);
        command_word_counter++;
    }

    // check for "incomplete commands"
    if (command_word_counter == 1) {
        while ( it != end(commands) ) {
            auto & command = it->first;
            ++it;

            if ( command.rfind(text, 0) == 0 ) {
                return strdup(command.c_str());
            }
        }
    } else if (command_word_counter == 2) {
        std::string command = text;
        if (command[command.size()-1] == ' ') {
            command.erase(command.size()-1, 1);
        }
        printf("command: %s\n",command.c_str());
    }

    return NULL;
}

char** ConsoleCommands::attempted_completion_function(const char * text, int start, int) {
    rl_attempted_completion_over = 1;
    return rl_completion_matches(text, &ConsoleCommands::match_finder);
}


void ConsoleCommands::registerCommand(const std::string &name, std::function<void(const std::vector<std::string> &params)> func, bool use_thread) {
    CommandDef def;
    def.func = func;
    def.use_thread = use_thread;
    commands[name] = def;
}

int ConsoleCommands::registerParamsForCommand(const std::string &name, const std::vector<ParamDef> &params) {
    if (commands.find(name) != commands.end()) {
        commands[name].params = params;
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