#include <iostream>
#include <boost/asio.hpp>
#include <sel_commands.h>

int main() {
    std::cout << "Starting up!" << std::endl;
    boost::asio::io_service io;
    boost::asio::serial_port sp(io, "COM3");

    sp.set_option(serial_port_base::baud_rate(9600));
    sp.set_option(serial_port_base::character_size(8));
    sp.set_option(serial_port_base::parity(serial_port_base::parity::none));
    sp.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

    std::string command = SelCommands::Test("helloworld");
    std::cout << "Command Sent: " << command << std::endl;

    char reply[128];
    boost::asio::read(sp, boost::asio::buffer(reply, 128));

    std::cout << "Response Received: " << reply << std::endl;

    sp.close();

    return 0;
}
