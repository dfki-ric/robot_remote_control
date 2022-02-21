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

std::vector<std::string> ConsoleCommands::parseLine(const std::string &line) {
    std::vector<std::string> linevec;
    std::istringstream linestream;
    linestream.str(line);
    std::string part;
    while (std::getline(linestream, part, ' ')) {
        // when part is not created because of double space
        if (part.size()) {
            linevec.push_back(part);
        }
    }
    return linevec;
}

bool ConsoleCommands::readline(const std::string& prompt) {
    char* input = ::readline(prompt.c_str());
    if (!input) return false;
    add_history(input);
    std::vector<std::string> line = parseLine(input);

    runCommand(line);
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
    
    std::string currentline(text);
    std::vector<std::string> line = parseLine(currentline);
    std::string &command = line.front();

    int params = line.size();

    std::string completed;
    std::for_each(line.begin(), line.end()-1 ,[&](const std::string& part) {
        completed += part + " ";
    });

    std::string curpart;
    if ( line.size() > 1 ) {
        curpart = line.back();
        //std::cout << curpart << std::endl;
    }
     

    if (params > 0 && params <= commands[command].params.size()) {
        
        if (state == 0) {
            return strdup((completed + curpart + " ").c_str());
        }
        if (state == 1) {
            return strdup((completed + curpart + " " + commands[command].params[params-1].defaultvalue + " ").c_str());
        }
        if (state == 2 && currentline[currentline.size()-1] == ' ') {
            return strdup((currentline + "<" + commands[command].params[params-1].hint + "> ").c_str());
        }
    }
    return NULL;
}

char** ConsoleCommands::attempted_completion_function(const char * text, int start, int end) {
    // disable default completion
    rl_attempted_completion_over = 1;

    std::string line(text);
    std::vector<std::string> linevec = parseLine(line);

    std::string command;
    if (linevec.size()) {
        command = linevec.front();
    }

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

void ConsoleCommands::runCommand(std::vector<std::string> &line) {
    std::string cmd = line.front();
    std::vector<std::string> params(line.begin()+1, line.end());

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