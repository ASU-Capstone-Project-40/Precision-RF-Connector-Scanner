#include <iostream>
#include "sel_commands.h"

int main() {
    std::cout << "Starting up!" << std::endl;

    SimpleSerial serial_helper("COM3");
    
    SelCommands SelCommand;
    auto cmd = SelCommand.Test("helloworld");

    serial_helper.writeString(cmd);

    std::cout << "Command Sent: " << cmd << std::endl;

    std::string resp = serial_helper.readLine();

    std::cout << "Response Recieved: " << resp << std::endl;

    cmd = SelCommand.Home(true, true);
    serial_helper.writeString(cmd);
    std::cout << "Command Sent: " << cmd << std::endl;
    resp = serial_helper.readLine();

    std::cout << "Response Recieved: " << resp << std::endl;

    serial_helper.Close();

    return 0;
}
