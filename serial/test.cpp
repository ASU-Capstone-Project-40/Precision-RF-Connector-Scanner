#include <iostream>
#include "sel_commands.h"

int main()
{
    SelCommands SelCommand;

    auto cmd = SelCommand.Home(true, true);
    std::cout << "Home Command: " << cmd << std::endl;

    cmd = SelCommand.Test("helloworld");
    std::cout << "Test Command: " << cmd << std::endl;
    
    return 0;
}