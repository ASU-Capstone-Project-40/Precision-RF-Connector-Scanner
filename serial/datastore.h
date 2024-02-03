// Datastore.h
#ifndef DATASTORE_H
#define DATASTORE_H

#include <string>
#include "sel_commands.h"
#include "logging.h"

namespace Datastore {

    SELMotor x_axis;
    SELMotor y_axis;
    uint8_t z_axis = 0;

    bool Update(SimpleSerial serial_object) {
        auto status_msg = SelCommands::AxisInquiry(serial_object);
        uint8_t num_axes = status_msg.at(6) - '0';

        if (num_axes < 1) {
            std::cout << "Error: No axes detected." << std::endl;
            return false;
        }

        uint8_t idx = 7; // Where the relevant axis data starts
        uint8_t position_data_length = 9;
        uint8_t axis_data_length = 14;

        x_axis.enabled_ = status_msg.at(idx) == '1' ? true : false;
        x_axis.homed_ = status_msg.at(idx+1) == '1' ? true : false;
        x_axis.in_motion_ = status_msg.at(idx+2) == '1' ? true : false;
        x_axis.error_code_ = std::string() + status_msg.at(idx+3) + status_msg.at(idx+4);
        
        std::string pos_string = status_msg.substr(idx + 5, position_data_length);
        x_axis.position_ = std::stod(pos_string);

        logv( std::string("X Axis State:\r\n") +
               "Enabled:    " + std::to_string(x_axis.enabled_) + "\r\n" +
               "Homed:      " + std::to_string(x_axis.homed_) + "\r\n" +
               "In motion:  " + std::to_string(x_axis.in_motion_) + "\r\n" +
               "Error code: " + x_axis.error_code_ + "\r\n" +
               "Position:   " + std::to_string(x_axis.position_)
        );

        if (num_axes < 2) {
            std::cout << "Error: y-axis not detected." << std::endl;
            return false;
        }

        idx += axis_data_length;

        y_axis.enabled_ = status_msg.at(idx) == '1' ? true : false;
        y_axis.homed_ = status_msg.at(idx+1) == '1' ? true : false;
        y_axis.in_motion_ = status_msg.at(idx+2) == '1' ? true : false;
        y_axis.error_code_ = std::string() + status_msg.at(idx+3) + status_msg.at(idx+4);
        
        pos_string = status_msg.substr(idx + 5, position_data_length);
        y_axis.position_ = std::stod(pos_string);

        logv(  std::string("Y Axis State:\r\n") +
               "Enabled:    " + std::to_string(y_axis.enabled_) + "\r\n" +
               "Homed:      " + std::to_string(y_axis.homed_) + "\r\n" +
               "In motion:  " + std::to_string(y_axis.in_motion_) + "\r\n" +
               "Error code: " + y_axis.error_code_ + "\r\n" +
               "Position:   " + std::to_string(y_axis.position_)
        );

        return true;
    }
};

#endif // DATASTORE_H
