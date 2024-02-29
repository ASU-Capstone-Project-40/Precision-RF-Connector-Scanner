#include "sel_interface.h"   // Defines SEL controller commands
#include "datastore.h"

SimpleSerial *SEL = nullptr;
int Logger::log_level_ = Logger::Level::INFO;

int main()
{
    Logger::setLogLevel("debug");

    auto& DS = Datastore::getInstance();

    // for (size_t i=0; i<DS.SEL_outputs.size(); i++) {
    //     std::cout << i << ": " << DS.SEL_outputs[i] << std::endl;
    // }

    SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
    SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs);
    SEL_Interface::SetOutputs({305, 307}, {1, 1}, DS.SEL_outputs);
    SEL_Interface::SetOutputs({306}, {1}, DS.SEL_outputs);
    SEL_Interface::SetOutputs({305, 306, 307, 411, 512}, {0, 0, 0, 1, 1}, DS.SEL_outputs);


    // Test SEL_Interface::format
        // double a = 123.4;
        // float b = 12.345;
        // int c = 148;
        // uint16_t d = 3;
        // uint8_t e = 0;

        // std::string f = "0";
        // std::cout << "Length F: " << f.length() << std::endl;

        // std::string g = std::to_string(0);
        // std::cout << "Length e: " << g.length() << std::endl;

        // SEL_Interface::format<double>(a, 8, 2);
        // SEL_Interface::format<float>(b, 8, 2);
        // SEL_Interface::format<int>(c, 4, 0);
        // SEL_Interface::format<uint16_t>(d, 2, 0);
        // SEL_Interface::format<uint16_t>(0, 2);

        // try {
        // SEL_Interface::format<double>(a, 2, 1);
        // }

        // catch (std::runtime_error) {
        //     Logger::info("Task failed successfully!");
        // }

    return 0;
}