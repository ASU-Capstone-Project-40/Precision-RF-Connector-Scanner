#include "sel_commands.h"   // Defines SEL controller commands

int main()
{
    VERBOSE_LOGGING = true;

    // Test SelCommands::formatValue
        double a = 123.4;
        float b = 12.345;
        int c = 148;

        SelCommands::formatValue<double>(a, 8, 2);
        SelCommands::formatValue<float>(b, 8, 2);
        SelCommands::formatValue<int>(c, 4, 0);

        try {
        SelCommands::formatValue<double>(a, 2, 1);
        }

        catch (std::runtime_error) {
            logv("Task failed successfully!");
        }

    return 0;
}