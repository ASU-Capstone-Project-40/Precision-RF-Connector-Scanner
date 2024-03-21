#ifndef DATASTORE_H
#define DATASTORE_H

#include <string>
#include "sel_interface.h"
#include "logging.h"
#include <vector>

class SELMotor {
public:
    SELMotor() = default;

    bool enabled_{false};
    bool homed_{false};
    bool in_motion_{false};
    std::string error_code_{""};
    double position_{0.0};
};

class Datastore {
public:
    SELMotor x_axis;
    SELMotor y_axis;
    uint16_t SEL_velocity = 100; // mm/s
    uint8_t z_axis = 0;

    std::vector<bool> SEL_outputs;

    Datastore(const Datastore&) = delete;
    Datastore& operator=(const Datastore&) = delete;

    static Datastore& getInstance() {
        static Datastore instance;
        return instance;
    }

    bool UpdateSEL() {
        auto status_msg = SEL_Interface::AxisInquiry();
        uint8_t num_axes = status_msg.at(6) - '0';

        if (num_axes < 1) {
            Logger::error("Datastore::UpdateSEL: No axes detected.");
            return false;
        }

        uint8_t idx = 7; // Where the relevant axis data starts
        uint8_t position_data_length = 9;
        uint8_t axis_data_length = 14;

        y_axis.enabled_ = status_msg.at(idx) == '1' ? true : false;
        y_axis.homed_ = status_msg.at(idx+1) == '1' ? true : false;
        y_axis.in_motion_ = status_msg.at(idx+2) == '1' ? true : false;
        y_axis.error_code_ = std::string() + status_msg.at(idx+3) + status_msg.at(idx+4);

        std::string pos_string = status_msg.substr(idx + 5, position_data_length);
        y_axis.position_ = std::stod(pos_string);

        Logger::debug(std::string("Y Axis State:\r\n") +
                                  "Enabled:    " + std::to_string(y_axis.enabled_) + "\r\n" +
                                  "Homed:      " + std::to_string(y_axis.homed_) + "\r\n" +
                                  "In motion:  " + std::to_string(y_axis.in_motion_) + "\r\n" +
                                  "Error code: " + y_axis.error_code_ + "\r\n" +
                                  "Position:   " + std::to_string(y_axis.position_)
        );

        if (y_axis.error_code_ != "00") {
            SEL_Interface::HaltAll();
            throw std::runtime_error("Y axis encountered error " + y_axis.error_code_);
        }

        if (num_axes < 2) {
            Logger::error("Error: x-axis not detected.");
            return false;
        }

        idx += axis_data_length;

        x_axis.enabled_ = status_msg.at(idx) == '1' ? true : false;
        x_axis.homed_ = status_msg.at(idx+1) == '1' ? true : false;
        x_axis.in_motion_ = status_msg.at(idx+2) == '1' ? true : false;
        x_axis.error_code_ = std::string() + status_msg.at(idx+3) + status_msg.at(idx+4);

        pos_string = status_msg.substr(idx + 5, position_data_length);
        x_axis.position_ = std::stod(pos_string);

        Logger::debug(std::string("X Axis State:\r\n") +
                                  "Enabled:    " + std::to_string(x_axis.enabled_) + "\r\n" +
                                  "Homed:      " + std::to_string(x_axis.homed_) + "\r\n" +
                                  "In motion:  " + std::to_string(x_axis.in_motion_) + "\r\n" +
                                  "Error code: " + x_axis.error_code_ + "\r\n" +
                                  "Position:   " + std::to_string(x_axis.position_)
        );

        if (x_axis.error_code_ != "00") {
            SEL_Interface::HaltAll();
            throw std::runtime_error("X axis encountered error " + x_axis.error_code_);
        }

        return true;
    }

    void waitForMotionComplete() {
        UpdateSEL();
        while(x_axis.in_motion_ || y_axis.in_motion_) {
            UpdateSEL();
        }
    }

private:
    Datastore(): SEL_outputs(288, false) {
    }
};


#endif // DATASTORE_H
