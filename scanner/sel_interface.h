#ifndef SEL_INTERFACE_H
#define SEL_INTERFACE_H

#include "simple_serial.h"
#include <sstream>
#include <iomanip>
#include <vector>

namespace SEL_Interface
{
    enum Axis {
        X = 1,
        Y = 2,
        XY = 3
    };

    enum Direction {
        NEGATIVE = 0,
        POSITIVE = 1
    };

    static std::string exec = "!99";
    static std::string inq = "?99";
    static std::string term = "@@\r\n";

        /**
     * Formats a numeric value into a string with a given length and precision.
     * \param value Value to convert - must be a number type
     * \param length Length of string to output
     * \param precision Number of decimal places to output. Defaults to zero if unspecified.
     * .
     * Example input: 123.4
     * Example output: "00123.40"
     */
    template <typename T>
    std::string format(T value, uint8_t length, uint8_t precision = 0) {
        Logger::debug("SEL_Interface::format: Converting " + std::to_string(value) + " to string with length " + std::to_string(length) + " and precision " + std::to_string(precision));

        if (value < 0) {
            throw std::runtime_error("SEL_Interface::formatValue: value " + std::to_string(value) + " must not be negative");
        }

        std::ostringstream stream;
        stream << std::fixed << std::setprecision(precision) << std::setw(length) << std::setfill('0') << value;
        std::string result = stream.str();

        if (result.length() > length) {
            throw std::runtime_error("SEL_Interface::format: value " + std::to_string(value) + " is too large to convert into this format");
        }

        Logger::debug("SEL_Interface::format: Successfully converted value: " + result);
        return result;
    }

            /******************************************************
             ***                Inquiry Commands                ***
             ******************************************************/

    /**
     * Executes communication test. The same characters as the command is transmitted back.
     * \param text Any Letters (10 characters maximum)
     * Example command: ?99TST0123456789@@
     * Example response: #99TST0123456789@@
     */
    std::string Test(const std::string& text) {
        if (text.length() > 10) {
            Logger::error("SEL_Interface::Test: Max text length is 10 characters");
            return "";
        }
        std::string code = "TST";
        std::string cmd = inq + code + text + term;
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
        std::string cmd = inq + code + term;
        SEL->writeString(cmd);
        std::string resp = SEL->readLine();
        return resp;
    }


            /******************************************************
             ***               Execution Commands               ***
             ******************************************************/


    /**
     * Initiates homing sequence. Servo ON function also included.
     * \param axis Axis or axes to home.
     * Example command: !99HOM0300@@
     * Example response: #99HOM@@
     */
    std::string Home(int axis) {
        std::string code = "HOM";
        std::string axis_pattern_string = format<int>(axis, 2, 0);
        std::string cmd = exec + code + axis_pattern_string + "00" + term;
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
        uint16_t axis_pattern = 0;
        std::vector<std::string> axis_positions;
        for (size_t i = 0; i < joint_state.size(); ++i) {
            if (joint_state[i] >= 0) {
                axis_pattern += static_cast<uint8_t>(std::pow(2, i));
                axis_positions.push_back(format<double>(joint_state[i], 8, 2));
            }
        }

        if (axis_positions.size() < 1) {
            Logger::error("SEL_Interface::MoveToPosition: No axes commanded.");
            return "";
        }

        std::string axis_pattern_string = format<int>(axis_pattern, 2);

        std::string cmd = exec + code + axis_pattern_string + "0000" + "0100"; // TODO: Make velocity a param instead of hard-coded

        for (auto& axis_position : axis_positions) {
            cmd += axis_position;
        }

        cmd += term;
        SEL->writeString(cmd);
        std::string resp = SEL->readLine();
        return resp;
    }

    /**
     * Slows the axis to a stop specified by the axis pattern.
     * Note: Do not use the Halt protocol command during homing.
     * \param axis Axis or axes to halt
     * Example command: !99HLT03@@
     * Example response:  #99HLT@@
     */
    std::string Halt(int axis) {
        std::string code = "HLT";
        std::string axis_pattern_string = format<int>(axis, 2, 0);
        std::string cmd = exec + code + axis_pattern_string + term;
        SEL->writeString(cmd);
        std::string resp = SEL->readLine();
        if (resp !=  "#99HLT@@") {
            Logger::warn("SEL_Interface::Halt: Expected response #99HLT@@, recieved " + resp);
        }
        return resp;
    }


    /**
     * Stops X and Y axes.
    */
    void HaltAll() {
        Logger::info("SEL_Interface::HaltAll: Halting all axes");
        Halt(Axis::XY);
        Logger::debug("SEL_Interface::HaltAll: Success");
    }

    /**
     * Executes Jog move. When there is no deceleration stop command, it stops at the soft limit.
     * \param axis Axis or axes to jog. Use SEL_Interface::Axis
     * \param direction The direction to jog. Use SEL_Interface::Direction
     * \param velocity Jog velocity in mm/sec. Defaults to 50
     * \param acceleration Jog acceleration in 1/100g. Defaults to 0.3
     * Example command: !99JOG030.3000501@@
     * Example response: #99JOG@@
    */
    std::string Jog(int axis, int direction, uint16_t velocity = 50, double acceleration = 0.3) {
        std::string code = "JOG";
        std::string axis_pattern = format<int>(axis, 2, 0);
        std::string acceleration_string = format<double>(acceleration, 4, 2);
        std::string velocity_string = format<int16_t>(std::abs(velocity), 4, 0);
        std::string direction_string = std::to_string(direction);
        std::string cmd = exec + code + axis_pattern + acceleration_string + velocity_string + direction_string + term;
        SEL->writeString(cmd);
        std::string resp = SEL->readLine();
        if (resp !=  "#99JOG@@") {
            Logger::warn("SEL_Interface::Halt: Received unexpected response `" + resp + "` to command `" + cmd + "`");
        }
        return resp;
    }

    void SetOutputs(std::vector<int> ports, std::vector<bool> values, std::vector<bool>& SEL_outputs) {
        if (ports.size() != values.size() ) {
            throw std::runtime_error("SEL_Interface::SetOutputs: ports and values must have the same number of elements");
        }

        std::vector<int> port_groups;
        for (size_t i=0; i < ports.size(); ++i)
        {
            int port_idx = ports[i] - 300;

            if (port_idx < 0 || port_idx > SEL_outputs.size()) {
                throw std::runtime_error("SEL_Interface::SetOutputs: Invalid port [" + std::to_string(ports[i]) +
                "]. Output ports range from 300 to " + std::to_string( 300 + SEL_outputs.size() - 1));
            }

            SEL_outputs[port_idx] = values[i];

            int group = (ports[i]-300) / 8;
            auto it = std::find(port_groups.begin(), port_groups.end(), group);
            if (it != port_groups.end()) {
                continue;
            }
            port_groups.push_back(group);
        }

        for (auto& group : port_groups) {
            size_t group_start = group * 8;
            size_t group_end = group_start + 7;

            std::vector<bool> group_values(SEL_outputs.begin() + group_start, SEL_outputs.begin() + group_end + 1);

            std::string hex_values_string = "0123456789ABCDEF";
            std::string group_values_string;
            int hex_value = 0;

            for (int i=4; i < 8; ++i) {
                hex_value += group_values[i] * std::pow(2, i-4);
            }

            group_values_string.push_back(hex_values_string[hex_value]);

            hex_value = 0;

            for (int i=0; i < 4; ++i) {
                hex_value += group_values[i] * std::pow(2, i);
            }

            group_values_string.push_back(hex_values_string[hex_value]);
            std::string group_string = format<uint16_t>(group, 2);
            std::string code = "OTS";

            std::string cmd = exec + code + group_string + group_values_string + term;

            SEL->writeString(cmd);
            std::string resp = SEL->readLine();
            if (resp != "#99OTS@@") {
                Logger::warn("SEL_Interface::SetOutputs: Received unexpected response `" + resp +
                                         "` to command `" + cmd + "`");
            }
        }
    }
};

#endif // SEL_INTERFACE_H