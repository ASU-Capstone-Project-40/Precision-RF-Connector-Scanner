#ifndef SIMPLE_SERIAL_H
#define SIMPLE_SERIAL_H

#include <boost/asio.hpp>
#include "logging.h"
#include <iomanip>

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
        Logger::info("Opening new serial connection on " + port + " at rate " + std::to_string(baud_rate));
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
        Logger::verbose("Sending: " + s);
        boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
    }

    // TODO: Handle logging better
    void writeBytes(unsigned char* data, std::size_t length){
        std::cout << std::hex << std::setw(2) << std::setfill('0');
        std::cout << "SENDING WRITEBYTES COMMAND" << std::endl;
        // Iterate through each byte in the data array
        for(std::size_t i = 0; i < length; ++i) {
            // Print the current byte in hexadecimal format
            std::cout << static_cast<int>(data[i]) << " ";
        }

        std::cout << std::endl;

        auto buffer = boost::asio::buffer(data,length);
        boost::asio::write(serial,buffer);
    }

    // TODO: Handle logging better
    void writeVector(std::vector<unsigned char> data){
        std::cout << std::hex << std::setw(2) << std::setfill('0');
        std::cout << "Writing vector ";
        for(std::size_t i = 0; i < data.size(); ++i) {
            // Print the current byte in hexadecimal format
            std::cout << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::endl;
        auto buffer = boost::asio::buffer(data, data.size());
        boost::asio::write(serial,buffer);
    }

    /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
    std::string readLine()
    {
        Logger::verbose("Receiving: ");
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;
        char c;
        std::string result;
        for(;;)
        {
            asio::read(serial,asio::buffer(&c,1));
            Logger::verbose_stream(c);

            switch(c)
            {
                case '\r':
                    break;
                case '\n':
                    Logger::verbose(""); // Send an end line since the above verbose_stream does not
                    return result;
                default:
                    result+=c;
            }
        }
    }

    /**
     * Blocks forever.
     * \return never
     * \throws boost::system::system_error on failure
     */
    void readBytes()
    {
        Logger::verbose("Receiving: ");
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;
        unsigned char c;
        std::string result;
        for(;;)
        {
            asio::read(serial,asio::buffer(&c,1));
            std::cout << std::hex << (0xFF & c) << std::flush;
        }
    }

    /**
     * TODO: This doesn't work correctly, might crash
     * Reads data_length bytes from the serial device. Blocks until data_length bytes have been received.
     * \return a vector of bytes of length data_length
     * \throws boost::system::system_error on failure
     */
    std::vector<unsigned char> readBytes(size_t data_length)
    {
        Logger::verbose("Receiving: ");
        using namespace boost;
        std::vector<unsigned char> data;
        asio::read(serial,asio::buffer(&data, data_length));
        for (size_t i = 0; i < data_length; ++i) {
            std::cout << std::hex << (0xFF & data[i]) << std::flush;
        }
        std::cout << std::endl;
        return data;
    }

    void Close()
    {
        Logger::info("SimpleSerial::Close: Closing serial port on " + port);
        serial.close();
        Logger::verbose("SimpleSerial::Close: Success");
    }

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    std::string port;
};

extern SimpleSerial* SEL;
extern SimpleSerial* Gripper;

#endif // SIMPLE_SERIAL_H