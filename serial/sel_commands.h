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
    SimpleSerial(std::string port)
    : io(), serial(io,port)
    {
        serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
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
            std::cout << c << std::flush;
            switch(c)
            {
                case '\r':
                    break;
                case '\n':
                    std::cout << std::endl;
                    return result;
                default:
                    result+=c;
            }
        }
    }

    void Close()
    {
        serial.close();
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
     * \param data Any Letters (10 characters maximum)
     */

    std::string Test(const std::string& data) {
        std::string code = "TST";
        std::string cmd = inq_ + code + data + term_;
        return cmd;
    }

    /**
     * Initiates homing sequence. Servo ON function also included.
     * \param x home x axis
     * \param y home y axis
     * \param vel Parameter goes into effect when this is zero
     */
    std::string Home(bool x, bool y, std::string vel = "00"){
        std::string code = "HOM";
        std::string axis_pattern = '0' + std::to_string(uint8_t(x) + 2*uint8_t(y));

        std::string cmd = exec_ + code + axis_pattern + vel + term_;
        return cmd;
    }

private:
    std::string exec_ = "!99";
    std::string inq_ = "?99";
    std::string term_ = "@@\r\n";
};