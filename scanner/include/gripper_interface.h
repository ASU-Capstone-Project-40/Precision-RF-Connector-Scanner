#ifndef GRIPPER_INTERFACE_H
#define GRIPPER_INTERFACE_H

#include <vector>
#include <string>
#include "../include/logging.h"

namespace Gripper_Interface
{
    /**
     * Initializes the gripper.
     */
    bool Initialize() {
        unsigned char init_cmd[] = {0x01, 0x06, 0x01, 0x00, 0x00, 0x01, 0x49, 0xF6};
        Gripper->writeBytes(init_cmd, sizeof(init_cmd));
        bool success = Check_Response(init_cmd);
        return success;
    }

    bool Check_Response(unsigned char* expected_response) {
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
}
#endif // GRIPPER_INTERFACE_H