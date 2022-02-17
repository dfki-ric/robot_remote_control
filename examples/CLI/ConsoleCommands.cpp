#include "ConsoleCommands.hpp"
#include <algorithm>
#include <utility>
#include <sstream>

std::map< std::string, ConsoleCommands::CommandDef > ConsoleCommands::commands;

ConsoleCommands::ConsoleCommands() {
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
    free(input);
    return true;
}

char* ConsoleCommands::match_finder(const char *text, int state) {
    static std::map< std::string, CommandDef >::iterator it;
    // state is 0 when a "word" is finished
    if ( state == 0 ) it = begin(commands);

    // check for "incomplete commands"
    while ( it != end(commands) ) {
        auto & command = it->first;
        ++it;

        if ( command.rfind(text, 0) == 0 ) {
            return strdup(command.c_str());
        }
    }
    return NULL;
}

char** ConsoleCommands::attempted_completion_function(const char * text, int start, int) {
    rl_attempted_completion_over = 1;
    return rl_completion_matches(text, &ConsoleCommands::match_finder);
}
