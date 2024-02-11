#include "sel_interface.h"   // Defines SEL controller commands

SimpleSerial *SEL = nullptr;

int main()
{
    Logger::setLogLevel("debug");

    // Test SEL_Interface::toPaddedString
        double a = 123.4;
        float b = 12.345;
        int c = 148;
        uint16_t d = 3;

        SEL_Interface::toPaddedString<double>(a, 8, 2);
        SEL_Interface::toPaddedString<float>(b, 8, 2);
        SEL_Interface::toPaddedString<int>(c, 4, 0);
        SEL_Interface::toPaddedString<uint16_t>(d, 2, 0);

        try {
        SEL_Interface::toPaddedString<double>(a, 2, 1);
        }

        catch (std::runtime_error) {
            Logger::info("Task failed successfully!");
        }

    return 0;
}