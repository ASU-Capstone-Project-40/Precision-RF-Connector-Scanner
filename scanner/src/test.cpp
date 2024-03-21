#include "../include/sel_interface.h"   // Defines SEL controller commands
#include "../include/datastore.h"

SimpleSerial *SEL = nullptr;
int Logger::log_level_ = Logger::Level::INFO;

int main()
{
    double workspace_x = 330.0;
    double workspace_y = 700.0;
    double scan_width = 20.0;
    int num_passes = std::ceil(workspace_x / scan_width) + 1;
    std::cout << "workspace dimensions: " << workspace_x << ", " << workspace_y << std::endl;
    std::cout << "Scan width: " << scan_width << std::endl;
    std::cout << "num passes: " << num_passes << std::endl;
    // Begin scan
    for (int i = 0; i < num_passes*2; ++i) {
        double x_coordinate = std::min(scan_width * (i/2), workspace_x);
        double y_coordinate = workspace_y * (((i+1)/2) % 2);
        std::cout << "point " << i << ": (" << x_coordinate << ", " << y_coordinate << ")" << std::endl;
    }

    // Logger::setLogLevel("debug");

    // auto& DS = Datastore::getInstance();

    // Logger::error("This is an error");
    // Logger::warn("This is a warning");
    // Logger::info("This is info");
    // Logger::debug("This is debug");

    // for (size_t i=0; i<DS.SEL_outputs.size(); ++i) {
    //     std::cout << i << ": " << DS.SEL_outputs[i] << std::endl;
    // }

    // SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
    // SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs);
    // SEL_Interface::SetOutputs({305, 307}, {1, 1}, DS.SEL_outputs);
    // SEL_Interface::SetOutputs({306}, {1}, DS.SEL_outputs);
    // SEL_Interface::SetOutputs({305, 306, 307, 411, 512}, {0, 0, 0, 1, 1}, DS.SEL_outputs);


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