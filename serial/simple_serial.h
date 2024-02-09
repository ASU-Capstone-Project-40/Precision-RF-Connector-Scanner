#ifndef SIMPLE_SERIAL_H
#define SIMPLE_SERIAL_H

#include <boost/asio.hpp>
#include "logging.h"

// Create with help from https://web.archive.org/web/20130825102715/http://www.webalice.it/fede.tft/serial_port/serial_port.html
class SimpleSerial
{
public:
    /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, example 9600 or 115200
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
    SimpleSerial(std::string port, uint32_t baud_rate)
    : io(), serial(io,port), port(port)
    {
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    }

    /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    void writeString(std::string s)
    {
        logv("Sending: " + s);
        boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
    }

    /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
    std::string readLine()
    {
        logv("Receiving: ");
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;
        char c;
        std::string result;
        for(;;)
        {
            asio::read(serial,asio::buffer(&c,1));
            if (VERBOSE_LOGGING) {
                std::cout << c << std::flush;
            }
            switch(c)
            {
                case '\r':
                    break;
                case '\n':
                    logv(""); // Sends an end line since the above std::flush does not
                    return result;
                default:
                    result+=c;
            }
        }
    }

    void Close()
    {
        logv("Closing serial port on " + port);
        serial.close();
    }

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    std::string port;
};

extern SimpleSerial* SEL;

#endif // SIMPLE_SERIAL_H