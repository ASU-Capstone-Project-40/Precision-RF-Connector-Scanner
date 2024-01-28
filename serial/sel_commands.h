#ifndef SEL_COMMANDS_H
#define SEL_COMMANDS_H

#include <boost/asio.hpp>
#include "logging.h"

std::string COM_PORT;

// Create with help from https://web.archive.org/web/20130825102715/http://www.webalice.it/fede.tft/serial_port/serial_port.html
class SimpleSerial
{
public:
    SimpleSerial(const SimpleSerial&) = delete;
    SimpleSerial& operator=(const SimpleSerial&) = delete;

    static SimpleSerial& GetInstance() {
        if (!instance_) {
            instance_ = std::unique_ptr<SimpleSerial>(new SimpleSerial(COM_PORT));
        }
        return *instance_;
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
        logv("Closing serial port on " + COM_PORT);
        serial.close();
    }

private:
     /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
    SimpleSerial(std::string port)
    : io(), serial(io,port)
    {
        logv("Opening serial port on " + COM_PORT);
        serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    }

    static std::unique_ptr<SimpleSerial> instance_;
    boost::asio::io_service io;
    boost::asio::serial_port serial;
};

class SelCommands
{
public:
    SelCommands() : serial_helper_(SimpleSerial::GetInstance()) {

    }

    void CloseSerial() {
        serial_helper_.Close();
    }

/******************************************************
 ***                Inquiry Commands                ***
 ******************************************************/

    /**
     * Executes communication test. The same characters as the command is transmitted back.
     * \param data Any Letters (10 characters maximum)
     * Example command: ?99TST0123456789@@
     * Example response: #99TST0123456789@@
     */

    std::string Test(const std::string& data) {
        std::string code = "TST";
        std::string cmd = inq_ + code + data + term_;
        serial_helper_.writeString(cmd);
        std::string resp = serial_helper_.readLine();
        return resp;
    }

    /**
     * Inquires about the axis status.
     * Example command: ?99STA@@
     * Example response: #99STA200000150.000 00000150.000 @@
    */
    std::string AxisInquiry() {
        std::string code = "STA";
        std::string cmd = inq_ + code + term_;
        serial_helper_.writeString(cmd);
        std::string resp = serial_helper_.readLine();
        return resp;
    }


/******************************************************
 ***               Execution Commands               ***
 ******************************************************/


    /**
     * Initiates homing sequence. Servo ON function also included.
     * \param x home x axis
     * \param y home y axis
     * \param vel Parameter goes into effect when this is zero
     */
    std::string Home(bool x, bool y, std::string vel = "00") {
        std::string code = "HOM";
        std::string axis_pattern = "0" + std::to_string(uint8_t(x) + 2*uint8_t(y));
        std::string cmd = exec_ + code + axis_pattern + vel + term_;
        serial_helper_.writeString(cmd);
        std::string resp = serial_helper_.readLine();
        return resp;
    }

private:
    std::string exec_ = "!99";
    std::string inq_ = "?99";
    std::string term_ = "@@\r\n";
    SimpleSerial& serial_helper_;
};

class SELMotor {
public:
    SELMotor() = default;

    bool enabled_{false};
    bool homed_{false};
    bool in_motion_{false};
    std::string error_code_{""};
    double position_{0.0};
};

#endif // SEL_COMMANDS_H