#include "ConsoleCommands.hpp"
#include <algorithm>
#include <utility>

std::map< std::string, std::function<void()> > ConsoleCommands::commands;

ConsoleCommands::ConsoleCommands() {
    // Configure readline to auto-complete paths when the tab key is hit.
    rl_bind_key('\t', rl_complete);
    rl_attempted_completion_function = &ConsoleCommands::attempted_completion_function;
}

ConsoleCommands::~ConsoleCommands() {}

void ConsoleCommands::readline(const std::string& prompt) {
    char* input = ::readline(prompt.c_str());
    if (!input) return;
    add_history(input);
    std::string command(input);
    runCommand(command);
    free(input);
}

char* ConsoleCommands::match_finder(const char *text, int state) {
    static std::map< std::string, std::function<void()> >::iterator it;
    if ( state == 0 ) it = begin(commands);

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
