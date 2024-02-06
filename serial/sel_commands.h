#ifndef SEL_COMMANDS_H
#define SEL_COMMANDS_H

#include "simple_serial.h"
#include <sstream>
#include <iomanip>

namespace SelCommands
{
    static std::string exec = "!99";
    static std::string inq = "?99";
    static std::string term = "@@\r\n";

        /**
     * Formats a numeric value into a string with a given length and precision.
     * \param value Value to convert
     * \param length Length of string to output
     * \param precision Number of decimal places to output.
     * Example input: 123.4
     * Example output: "00123.40"
     */
    template <typename T>
    std::string formatValue(T value, uint8_t length, uint8_t precision) {
        logv("SelCommands::formatValue: Converting " + std::to_string(value) + " to string with length " + std::to_string(length) + " and precision " + std::to_string(precision));

        if (value < 0) {
            throw std::runtime_error("SelCommands::formatValue: value " + std::to_string(value) + " must not be negative");
        }

        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << std::setw(length) << std::setfill('0') << value;
        std::string result = stream.str();
        
        if (result.length() > length) {
            throw std::runtime_error("SelCommands::formatValue: value " + std::to_string(value) + " is too large to convert into this format");
        }

        logv("SelCommands::formatValue: Successfully converted value: " + result);
        return result;
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
        std::string cmd = inq + code + data + term;
        SEL->writeString(cmd);
        std::string resp = SEL->readLine();
        return resp;
    }

    /**
     * Inquires about the axis status.
     * Example command: ?99STA@@
     * Example response: #99STA200000150.000 00000150.000 @@
    */
    std::string AxisInquiry() {
        std::string code = "STA";
        std::string cmd = inq + code + SelCommands::term;
        SEL->writeString(cmd);
        std::string resp = SEL->readLine();
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
     */
    std::string Home(bool x, bool y) {
        std::string code = "HOM";
        std::string axis_pattern = "0" + std::to_string(uint8_t(x) + 2*uint8_t(y));
        std::string cmd = exec + code + axis_pattern + "00" + SelCommands::term;
        SEL->writeString(cmd);
        std::string resp = SEL->readLine();
        return resp;
    }

    /**
     * Moves actuator to designated joint state.
     * \param joint_state Vector of axis positions. All axes must be represented. Use -1 to keep axis from moving.
     * Example command: !99 MOV 03 0000 0200 00050.00 00075.00 @@
     * Example response: #99MOV@@
     */
    std::string MoveToPosition(std::vector<double> joint_state) {
        std::string code = "MOV";
        uint8_t axis_pattern = 0;
        std::vector<std::string> axis_positions;
        for (size_t i = 0; i < joint_state.size(); i++) {
            if (joint_state[i] >= 0) {
                axis_pattern += static_cast<uint8_t>(std::pow(2, i));
                axis_positions.push_back(formatValue<double>(joint_state[i], 8, 2));
            }
        }

        if (axis_positions.size() < 1) {
            std::cout << "MoveToPosition error: No axes commanded." << std::endl;
            return "";
        }
        
        std::string axis_pattern_string = "0" + std::to_string(axis_pattern); // TODO: formatValue(3, 2, 0) doesn't work (0♥)?
        
        std::string cmd = exec + code + axis_pattern_string + "0000" + "0100"; // TODO: Make velocity "0100" a param

        for (auto& axis_position : axis_positions) {
            cmd += axis_position;
        }

        cmd += SelCommands::term;
        SEL->writeString(cmd);
        std::string resp = SEL->readLine();
        return resp;
    }
};

#endif // SEL_COMMANDS_H