#include <boost/asio.hpp>

// From https://web.archive.org/web/20130825102715/http://www.webalice.it/fede.tft/serial_port/serial_port.html
class SimpleSerial
{
public:
    /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, e  xample 9600 or 115200
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
    SimpleSerial(std::string port, unsigned int baud_rate)
    : io(), serial(io,port)
    {
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }

    /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    void writeString(std::string s)
    {
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
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;
        char c;
        std::string result;
        for(;;)
        {
            asio::read(serial,asio::buffer(&c,1));
            switch(c)
            {
                case '\r':
                    break;
                case '\n':
                    return result;
                default:
                    result+=c;
            }
        }
    }

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
};

class SelCommands
{
public:
    /**
     * Executes communication test. The same data as the command is transmitted back.
     * \param data Any Letters (10 characters)
     */

    std::string Test(const std::string& data) {
        std::string code = "TST";
        std::string cmd = inq_ + code + data + term_;
        boost::asio::write(sp, boost::asio::buffer(cmd + "\n"));
        return cmd;
    }

    /**
     * Initiates homing sequence. Servo ON function also included.
     * \param x home x axis
     * \param y home y axis
     * \param vel Parameter goes into effect when this is zero
     */
    std::string Home(bool x, bool y, uint16_t vel){
        std::string code = 'HOM';
        std::string axis_pattern = '0' + std::to_string(x + y);
        std::string vel_str = std::to_string(vel);

        std::string cmd = exec_ + code + axis_pattern + vel_str + term_;
        return cmd;
    }

private:
    std::string exec_ = "!99";
    std::string inq_ = "?99";
    std::string term_ = "@@";
};