#ifndef DATASTORE_H
#define DATASTORE_H

#include <string>
#include "sel_interface.h"
#include "logging.h"
#include <vector>
#include <algorithm>

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

        Logger::verbose(std::string("Y Axis State:\r\n") +
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

        Logger::verbose(std::string("X Axis State:\r\n") +
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

    bool zMotionComplete() {
        auto inputs = SEL_Interface::ReadInputs();
        Logger::verbose("Reading value " + std::string(1, inputs[11]) + " for SEL inputs 19-16");
        if (inputs[11] >= '8')
            return true;
        return false;
    }

    void waitForZMotionComplete() {
        while(true) {
            if (zMotionComplete())
                return;
        }
    }

    void waitForMotionComplete() {
        UpdateSEL();
        while(x_axis.in_motion_ || y_axis.in_motion_) {
            UpdateSEL();
        }
    }

    void MoveRC(uint8_t point) {
        if (point > 15) {
            throw std::runtime_error("DataStore::MoveRC - Invalid port [" + std::to_string(point) + "] provided. Valid ports are 0-15");
        }

        std::vector<int> position_ports {306, 305, 304, 303};
        std::vector<bool> position_values;
        position_values.push_back(static_cast<bool>(point & 0b00001000)); // Returns true only if bit 4 is high 
        position_values.push_back(static_cast<bool>(point & 0b00000100)); // Returns true only if bit 3 is high 
        position_values.push_back(static_cast<bool>(point & 0b00000010)); // Returns true only if bit 2 is high 
        position_values.push_back(static_cast<bool>(point & 0b00000001)); // Returns true only if bit 1 is high
        std::string debug_position_values;
        for (auto val : position_values) {
            debug_position_values += val ? "1" : "0";
        }
        Logger::verbose("Attempting to move RC to point " + std::to_string(point) + " [" + debug_position_values + "]");

        SEL_Interface::SetOutputs(position_ports, position_values, SEL_outputs); // Set position
        SEL_Interface::SetOutputs({302}, {1}, SEL_outputs); // Command start
        SEL_Interface::SetOutputs({302}, {0}, SEL_outputs);
    }

private:
    Datastore(): SEL_outputs(288, false) {
    }
};


#endif // DATASTORE_H
