#include "sel_commands.h"   // Defines SEL controller commands

SimpleSerial *SEL = nullptr;

int main()
{
    VERBOSE_LOGGING = true;

    // Test SelCommands::toPaddedString
        double a = 123.4;
        float b = 12.345;
        int c = 148;
        uint16_t d = 3;

        SelCommands::toPaddedString<double>(a, 8, 2);
        SelCommands::toPaddedString<float>(b, 8, 2);
        SelCommands::toPaddedString<int>(c, 4, 0);
        SelCommands::toPaddedString<uint16_t>(d, 2, 0);

        try {
        SelCommands::toPaddedString<double>(a, 2, 1);
        }

        catch (std::runtime_error) {
            logv("Task failed successfully!");
        }

    return 0;
}