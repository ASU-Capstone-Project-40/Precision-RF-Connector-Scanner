#ifndef COMMANDER_H
#define COMMANDER_H

#include <string>
#include "sel_interface.h"
#include "logging.h"
#include <vector>
#include <algorithm>
#include <thread>
#include "xy.h"

enum RCPositions {
    HOME = 0,
    POUNCE = 13,
    GRASP = 14,
    MATE = 15,
};

class SELMotor {
public:
    SELMotor() = default;

    bool enabled{false};
    bool homed{false};
    bool in_motion{false};
    std::string error_code{""};
    double position{0.0};
};

class Commander {
public:
    SELMotor x_axis;
    SELMotor y_axis;
    
    bool in_motion{false};
    XY position{XY(0,0)};
    std::vector<bool> SEL_outputs;

    Commander(const Commander&) = delete;
    Commander& operator=(const Commander&) = delete;

    static Commander* getInstance() {
        static Commander instance;
        return &instance;
    }

    bool UpdateSEL() {
        auto status_msg = SEL_Interface::AxisInquiry();
        uint8_t num_axes = status_msg.at(6) - '0';

        if (num_axes < 1) {
            Logger::error("Commander::UpdateSEL: No axes detected.");
            return false;
        }

        uint8_t idx = 7; // Where the relevant axis data starts
        uint8_t position_data_length = 9;
        uint8_t axis_data_length = 14;

        y_axis.enabled = status_msg.at(idx) == '1' ? true : false;
        y_axis.homed = status_msg.at(idx+1) == '1' ? true : false;
        y_axis.in_motion = status_msg.at(idx+2) == '1' ? true : false;
        y_axis.error_code = std::string() + status_msg.at(idx+3) + status_msg.at(idx+4);

        std::string pos_string = status_msg.substr(idx + 5, position_data_length);
        y_axis.position = std::stod(pos_string);

        Logger::verbose(std::string("Y Axis State:\r\n") +
                                  "Enabled:    " + std::to_string(y_axis.enabled) + "\r\n" +
                                  "Homed:      " + std::to_string(y_axis.homed) + "\r\n" +
                                  "In motion:  " + std::to_string(y_axis.in_motion) + "\r\n" +
                                  "Error code: " + y_axis.error_code + "\r\n" +
                                  "Position:   " + std::to_string(y_axis.position)
        );

        if (y_axis.error_code != "00") {
            SEL_Interface::HaltAll();
            throw std::runtime_error("Y axis encountered error " + y_axis.error_code);
        }

        if (num_axes < 2) {
            Logger::error("Error: x-axis not detected.");
            return false;
        }

        idx += axis_data_length;

        x_axis.enabled = status_msg.at(idx) == '1' ? true : false;
        x_axis.homed = status_msg.at(idx+1) == '1' ? true : false;
        x_axis.in_motion = status_msg.at(idx+2) == '1' ? true : false;
        x_axis.error_code = std::string() + status_msg.at(idx+3) + status_msg.at(idx+4);

        pos_string = status_msg.substr(idx + 5, position_data_length);
        x_axis.position = std::stod(pos_string);

        Logger::verbose(std::string("X Axis State:\r\n") +
                                  "Enabled:    " + std::to_string(x_axis.enabled) + "\r\n" +
                                  "Homed:      " + std::to_string(x_axis.homed) + "\r\n" +
                                  "In motion:  " + std::to_string(x_axis.in_motion) + "\r\n" +
                                  "Error code: " + x_axis.error_code + "\r\n" +
                                  "Position:   " + std::to_string(x_axis.position)
        );

        if (x_axis.error_code != "00") {
            SEL_Interface::HaltAll();
            throw std::runtime_error("X axis encountered error " + x_axis.error_code);
        }

        in_motion = x_axis.in_motion || y_axis.in_motion;
        position = XY(x_axis.position, y_axis.position);

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
        Logger::verbose("Waiting for Z Motion complete.");
        while(true) {
            if (zMotionComplete())
                return;
        }
    }

    void waitForXYMotionComplete() {
        UpdateSEL();
        while(x_axis.in_motion || y_axis.in_motion) {
            UpdateSEL();
        }
    }

    void waitForAllMotionComplete() {
        waitForXYMotionComplete();
        waitForZMotionComplete();
    }

    void MoveRC(uint8_t point) {
        if (point > 15) {
            throw std::runtime_error("Commander::MoveRC - Invalid port [" + std::to_string(point) + "] provided. Valid ports are 0-15");
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
        Logger::verbose("Successfully sent moveRC command.");
    }

    /**
     * Grasps a the mobile connector and moves Z axis back up.
     * \param offset The XY distance from current to target position.
     * \param speed Speed to traverse XY. Z speed is set on RC controller
     * \param pause true to require user input before mating, false for full auto
    */
    void GraspMobile(XY offset, int speed, bool pause = false) {
        UpdateSEL();
        SEL_Interface::MoveToPosition(position + offset, speed);
        waitForXYMotionComplete();

        if (pause) {
            Logger::info("Confirm position before grasping.");
            system("pause");
        }
        
        // Z down to grasp connector
        MoveRC(RCPositions::GRASP); // TODO CHANGE THIS BACK TO GRASP
        waitForZMotionComplete();

        if (pause) {
            Logger::info("Confirm position before grasping.");
            system("pause");
        }

        Gripper_Interface::Close();
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        MoveRC(RCPositions::HOME);
    }

    /**
     * Mates a grasped mobile connector with the fixed connector and moves Z axis back up.
     * \param offset The XY distance from current to target position.
     * \param speed Speed to traverse XY. Z speed is set on RC controller
     * \param pause true to require user input before mating, false for full auto
    */
    void MateMobileToFixed(XY offset, int speed, bool pause = false) {
        UpdateSEL();
        SEL_Interface::MoveToPosition(position + offset, speed);
        waitForXYMotionComplete();
        
        // Z down to hover over connector
        MoveRC(RCPositions::POUNCE); //TODO UNCOMMENT THIS
        waitForZMotionComplete();

        if (pause) {
            Logger::info("Confirm position before mating.");
            system("pause");
        }

        // Z down to mate with connector
        MoveRC(RCPositions::MATE);
        waitForZMotionComplete();

        // Open gripper
        Gripper_Interface::Open();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Z up
        MoveRC(RCPositions::HOME);
    }

private:
    Commander(): SEL_outputs(288, false) {
    }
};

extern Commander* commander;

#endif // COMMANDER_H
