#ifndef GRIPPER_INTERFACE_H
#define GRIPPER_INTERFACE_H

#include <vector>
#include <string>
#include "../include/logging.h"
#include "../include/simple_serial.h"

namespace Gripper_Interface
{
    bool CheckResponse(unsigned char* expected_response) {
        std::vector<unsigned char> response;
        response = Gripper->readBytes(sizeof(expected_response));
        Logger::debug("Reading " + std::to_string(sizeof(expected_response)) + " bytes from the gripper. "
            "This may block forever if the expected response is not received."
        );

        if ( sizeof(expected_response) != response.size() ) {
            Logger::error("Gripper response length does not match expected length."); // TODO: print the response
            return false;
        }

        for (size_t i=0; i < response.size(); ++i) {
            if(expected_response[i] != response[i]) {
                Logger::error("Gripper returned an unexpected response."); // TODO: print the response
                return false;
            }
        }

        return true;
    }

    bool CheckResponseVector(std::vector<unsigned char> expected_response) {
        std::vector<unsigned char> response;

        Logger::debug("Reading " + std::to_string(expected_response.size()) + " bytes from the gripper. "
            "This may block forever if the expected response is not received."
        );

        response = Gripper->readBytes(expected_response.size());

        if ( expected_response.size() != response.size() ) {
            Logger::error("Gripper response length does not match expected length."); // TODO: print the response
            return false;
        }

        for (size_t i=0; i < response.size(); ++i) {
            if(expected_response[i] != response[i]) {
                Logger::error("Gripper returned an unexpected response."); // TODO: print the response
                return false;
            }
        }
        return true;
    }

    /**
     * Initializes the gripper.
     * TODO: Check response and return failure if unexpected response
     */
    bool Initialize() {
        std::vector<unsigned char> init {0x01, 0x06, 0x01, 0x00, 0x00, 0x01, 0x49, 0xF6}; // TODO: Can we use std::vector instead?
        Gripper->writeVector(init);
        // bool success = CheckResponse(init); TODO: CheckResponse uses ReadBytes which doesn't work
        return true;
    }

    bool Open() {
        std::vector<unsigned char> open {0x01, 0x06, 0x01, 0x03, 0x03, 0xE8, 0x78, 0x88};
        Gripper->writeVector(open);
        // bool success = CheckResponse(open);
        return true;
    }

    bool Close() {
        std::vector<unsigned char> close {0x01, 0x06, 0x01, 0x03, 0x00, 0x00, 0x78, 0x36};
        Gripper->writeVector(close);
        // bool success = CheckResponse(close);
        return true;
    }

    /**
     * Moves gripper jaws to a certain percentage of open.
     * TODO: Currently hard-coded to 50%
    */
    bool MoveTo(/*int percent*/) {
        // Multiply percent by 10
        // Split high and low bytes
        // Add to move command
        // Calculate CRC
        std::vector<unsigned char> move {0x01, 0x06, 0x01, 0x03, 0x01, 0xF4, 0x78, 0x21};
        Gripper->writeVector(move);
        // bool success = CheckResponse(close);
        return true;
    }

    // TODO: Untested
    std::vector<unsigned char> CaclulateCRC(const unsigned char* cmd, int len) {
        uint16_t crc = 0xFFFF;
        for (int pos = 0; pos < len; pos++) {
            crc ^= static_cast<uint16_t>(cmd[pos]); // XOR byte into least sig. byte of crc

            for (int i = 8; i != 0; i--) {    // Loop over each bit
                if ((crc & 0x0001) != 0) {        // If the LSB is set
                    crc >>= 1;                    // Shift right and XOR 0xA001
                    crc ^= 0xA001;
                }
                else                              // Else LSB is not set
                    crc >>= 1;                    // Just shift right
            }
        }
        // Note, this crc has low and high bytes swapped, so use it accordingly (or swap bytes)
        unsigned char LSB = crc >> 8 & 0xFF;
        unsigned char MSB = crc & 0xFF;
        std::vector<unsigned char> crc_bytes {LSB, MSB};
        return crc_bytes;
    }
}
#endif // GRIPPER_INTERFACE_H