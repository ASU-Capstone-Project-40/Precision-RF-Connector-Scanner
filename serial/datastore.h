// Datastore.h
#ifndef DATASTORE_H
#define DATASTORE_H

#include <string>
#include "sel_commands.h"
#include "logging.h"

class Datastore {
public:
    Datastore() = default;

    SELMotor x_axis_;
    SELMotor y_axis_;
    uint8_t z_axis_ = 0;

    bool Update() {
        auto status_msg = SelCommand_.AxisInquiry();
        uint8_t num_axes = status_msg.at(6) - '0';

        if (num_axes < 1) {
            std::cout << "Error: No axes detected." << std::endl;
            return false;
        }

        uint8_t idx = 7; // Where the relevant axis data starts
        uint8_t position_data_length = 9;
        uint8_t axis_data_length = 14;

        x_axis_.enabled_ = status_msg.at(idx) == '1' ? true : false;
        x_axis_.homed_ = status_msg.at(idx+1) == '1' ? true : false;
        x_axis_.in_motion_ = status_msg.at(idx+2) == '1' ? true : false;
        x_axis_.error_code_ = std::string() + status_msg.at(idx+3) + status_msg.at(idx+4);
        
        std::string pos_string = status_msg.substr(idx + 5, position_data_length);
        x_axis_.position_ = std::stod(pos_string);

        logv( std::string("X Axis State:\r\n") +
               "Enabled:    " + std::to_string(x_axis_.enabled_) + "\r\n" +
               "Homed:      " + std::to_string(x_axis_.homed_) + "\r\n" +
               "In motion:  " + std::to_string(x_axis_.in_motion_) + "\r\n" +
               "Error code: " + x_axis_.error_code_ + "\r\n" +
               "Position:   " + std::to_string(x_axis_.position_)
        );

        if (num_axes < 2) {
            std::cout << "Error: y-axis not detected." << std::endl;
            return false;
        }

        idx += axis_data_length;

        y_axis_.enabled_ = status_msg.at(idx) == '1' ? true : false;
        y_axis_.homed_ = status_msg.at(idx+1) == '1' ? true : false;
        y_axis_.in_motion_ = status_msg.at(idx+2) == '1' ? true : false;
        y_axis_.error_code_ = std::string() + status_msg.at(idx+3) + status_msg.at(idx+4);
        
        pos_string = status_msg.substr(idx + 5, position_data_length);
        y_axis_.position_ = std::stod(pos_string);

        logv(  std::string("Y Axis State:\r\n") +
               "Enabled:    " + std::to_string(y_axis_.enabled_) + "\r\n" +
               "Homed:      " + std::to_string(y_axis_.homed_) + "\r\n" +
               "In motion:  " + std::to_string(y_axis_.in_motion_) + "\r\n" +
               "Error code: " + y_axis_.error_code_ + "\r\n" +
               "Position:   " + std::to_string(y_axis_.position_)
        );

        return true;
    }

private:
    SelCommands SelCommand_;
};

#endif // DATASTORE_H
