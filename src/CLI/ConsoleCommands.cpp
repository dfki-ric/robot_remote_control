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
    rl_completer_word_break_characters = (char*)"";
}

ConsoleCommands::~ConsoleCommands() {}

std::vector<std::string> ConsoleCommands::parseLine(const std::string &line, bool filter_empty) {
    std::vector<std::string> linevec;
    std::istringstream linestream;
    linestream.str(line);
    std::string part;
    while (std::getline(linestream, part, ' ')) {
        if (filter_empty) {
            // when part is not created because of double space
            if (part.size()) {
                if (linevec.size() && linevec.back()[linevec.back().size()-1] == '\\') {
                    linevec.back().erase(linevec.back().size()-1);
                    // escaped space was removed by getline
                    linevec.back() += " " + part;
                } else {
                    linevec.push_back(part);
                }
            }
        } else {
            if (linevec.size() && linevec.back().size() >= 2 && linevec.back()[linevec.back().size()-1] == '\\') {
                linevec.back().erase(linevec.back().size()-1);
                // escaped space was removed by getline
                linevec.back() += " " + part;
            } else {
                linevec.push_back(part);
            }
        }
    }
    return linevec;
}

bool ConsoleCommands::readline(const std::string& prompt, bool exitOnFailure) {
    std::unique_ptr<char[]> input = std::unique_ptr<char[]>(::readline(prompt.c_str()));
    if (not input.get()) return false;
    add_history(input.get());
    std::vector<std::string> line = parseLine(input.get());
    if (exitOnFailure) {
        return runCommand(line);
    } else {
        runCommand(line);
        return true;
    }
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

    std::vector<std::string> line = parseLine(currentline, false);
    std::string &command = line.front();
    // count params available in line, first space ist after command, so -1
    int finished_params = std::count(currentline.begin(), currentline.end(), ' ') - 1;
    
    // find escapes an substract from finished params
    int escaped_spaces = 0;
    size_t position = currentline.find("\\ ", 0);
    while (position != std::string::npos) {
        // continue search after modyfied place
        position = currentline.find("\\ ", position+1);
        ++escaped_spaces;
    }
    finished_params -= escaped_spaces;

    int lastSpacepos = currentline.rfind(' ');
    std::string completed = currentline;
    if (lastSpacepos != std::string::npos) {
        completed.erase(lastSpacepos);
    }
    std::string curpart = currentline;
    curpart.erase(0, completed.size());


    // has params left to type
    if (finished_params >= 0 && finished_params < commands[command].params.size()) {
        // find suitable params for current line
        std::vector<std::string> &defaultvalues = commands[command].params[finished_params].defaultvalues;
        std::vector<std::string> matchingValues;
        std::for_each(defaultvalues.begin(), defaultvalues.end(), [&](const std::string& val) {
            // check id prev param was completed (space at end of line)
            if (curpart == " ") {
                matchingValues.push_back(val);
            } else {
                // it is is currently typed: provide only fitting params
                // curpart start with " ", remove it
                std::string parameterPart(curpart.begin()+1, curpart.end());
                if (val.rfind(parameterPart, 0) == 0) {
                    matchingValues.push_back(val);
                }
            }
        });
        if (state < matchingValues.size()) {
            return strdup((completed + " " + matchingValues[state]).c_str());
        }
        if (state == matchingValues.size() && currentline[currentline.size()-1] == ' ') {
            return strdup((currentline + "<" + commands[command].params[finished_params].hint + "> ").c_str());
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


void ConsoleCommands::registerCommand(const std::string &name, const std::string &doc, std::function<bool(const std::vector<std::string> &params)> func, bool use_thread) {
    CommandDef def;
    def.doc = doc;
    def.func = func;
    def.use_thread = use_thread;
    commands[name] = def;
    completions.push_back(name);
}

int ConsoleCommands::registerParamsForCommand(const std::string &name, const std::vector<ParamDef> &params) {
    if (commands.find(name) != commands.end()) {
        std::vector<ParamDef> escaped_params = params;
        std::for_each(escaped_params.begin(), escaped_params.end(), [&](ParamDef& param) {
            std::for_each(param.defaultvalues.begin(), param.defaultvalues.end(), [&](std::string& value) {
                // check for spaces to escape
                size_t position = value.find(" ", 0);
                while (position != std::string::npos) {
                    // escape space
                    value.insert(position, "\\");
                    // continue search after modyfied place
                    position = value.find(" ", position+2);
                }
            });
        });

        commands[name].params = escaped_params;
        return true;
    }
    printf("unknown command %s\n\tyou need to register the command before adding paramater definitions\n", name.c_str());
    return false;
}

bool ConsoleCommands::runCommand(std::vector<std::string> &line) {
    bool success = false;
    if (line.size()) {
        std::string cmd = line.front();
        std::vector<std::string> params(line.begin()+1, line.end());

        auto iter = commands.find(cmd);
        if (iter != commands.end()) {
            while (params.size() < iter->second.params.size()) {
                std::string question = iter->second.params[params.size()].hint + "[" + iter->second.params[params.size()].defaultvalues.front() +"]:";
                std::string param;
                std::cout << "missing paramater: " << question;
                std::getline(std::cin, param);
                if (param == "") {
                    param = iter->second.params[params.size()].defaultvalues.front();
                }
                params.push_back(param);
            }

            if (iter->second.use_thread) {
                thread_running = true;
                cmdthread = std::thread([&](){
                    while (thread_running) {
                        success = iter->second.func(params);
                    }
                });
                readline("press enter to stop\n");
                thread_running = false;
                cmdthread.join();
            } else {
                success = iter->second.func(params);
            }
        }
    }
    return success;
}

void ConsoleCommands::printHelp() {
    std::for_each(commands.begin(), commands.end(), [](const std::pair<std::string, CommandDef> &cmd) {
        std::string params;
        std::for_each(cmd.second.params.begin(), cmd.second.params.end(), [&](const ParamDef &paramdef){
            params += "<"+paramdef.hint + "> ";
        });
        printf("%s %s \n\t - %s\n\n", cmd.first.c_str(), params.c_str(), cmd.second.doc.c_str());
    });
}
