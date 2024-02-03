#ifndef SEL_COMMANDS_H
#define SEL_COMMANDS_H

#include "simple_seral.h"

namespace SelCommands
{
            /******************************************************
             ***                Inquiry Commands                ***
             ******************************************************/

    /**
     * Executes communication test. The same characters as the command is transmitted back.
     * \param data Any Letters (10 characters maximum)
     * \param serial_object The SimpleSerial object which will send and receive data.
     * Example command: ?99TST0123456789@@
     * Example response: #99TST0123456789@@
     */
    std::string Test(const std::string& data, SimpleSerial serial_object) {
        std::string code = "TST";
        std::string cmd = inq_ + code + data + term_;
        serial_object.writeString(cmd);
        std::string resp = serial_object.readLine();
        return resp;
    }

    /**
     * Inquires about the axis status.
     * \param serial_object The SimpleSerial object which will send and receive data.
     * Example command: ?99STA@@
     * Example response: #99STA200000150.000 00000150.000 @@
    */
    std::string AxisInquiry(SimpleSerial serial_object) {
        std::string code = "STA";
        std::string cmd = inq_ + code + term_;
        serial_object.writeString(cmd);
        std::string resp = serial_object.readLine();
        return resp;
    }


            /******************************************************
             ***               Execution Commands               ***
             ******************************************************/


    /**
     * Initiates homing sequence. Servo ON function also included.
     * \param x home x axis
     * \param y home y axis
     * \param vel Integer number represented as a two character string
     * \param serial_object The SimpleSerial object which will send and receive data
     */
    std::string Home(bool x, bool y, std::string vel = "00", SimpleSerial serial_object) {
        std::string code = "HOM";
        std::string axis_pattern = "0" + std::to_string(uint8_t(x) + 2*uint8_t(y));
        std::string cmd = exec_ + code + axis_pattern + vel + term_;
        serial_object.writeString(cmd);
        std::string resp = serial_object.readLine();
        return resp;
    }

    std::string exec_ = "!99";
    std::string inq_ = "?99";
    std::string term_ = "@@\r\n";
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