#include "../include/signal_handler.h" // Handles graceful shutdowns when ctrl+c is pressed
#include "../include/logging.h"        // Supports optional verbose logging
#include "../include/simple_serial.h"  // Handles serial communication
#include "../include/sel_interface.h"  // Defines SEL controller commands
#include "../include/datastore.h"      // Parses and stores system data for easy access
#include <thread>

SimpleSerial *SEL = nullptr;
int Logger::log_level_ = Logger::Level::INFO;

int main(int argc, char* argv[]) {

    std::string sel_port = "COM3";
    int sel_rate = 9600;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--log-level" || arg == "-log") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default log level.");
                continue;
            }
            Logger::setLogLevel((argv[i+1]));
            ++i;
        }
        else if (arg == "--sel-port") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default SEL port " + sel_port);
                continue;
            }
            sel_port = argv[i+1];
            ++i;
        }
        else if (arg == "--sel-rate") {
            if (argc < i+1) {
                Logger::warn(arg + " flag provided but no value specified. Using default SEL baud rate " + std::to_string(sel_rate));
                continue;
            }
            sel_rate = std::stoi(argv[i+1]);
            ++i;
        }
        else {
            Logger::warn(arg + " flag not recognized. Ignoring.");
        }
    }

    Logger::info("Opening new serial connection on " + sel_port + " at rate " + std::to_string(sel_rate));
    SEL = new SimpleSerial(sel_port, sel_rate);

    // Register signal handler to close serial port when ctrl+c is pressed
    // signal(SIGINT, signalHandler);

    try {

        SEL_Interface::MoveToPosition({100, 100});
        auto& DS = Datastore::getInstance();
        DS.UpdateSEL();
        DS.waitForMotionComplete();

        SEL_Interface::Test("helloworld");

        Logger::info("Telling RC Not to move ");
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //  signals the RC controller not to move to a new position

        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P0 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {0, 0, 0, 0}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //0
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        // DS.waitForZMotionComplete();
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P1 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {0, 0, 0, 1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //1
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());


        Logger::info("Commanding RC to P2 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {0, 0, 1, 0}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //2
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        // DS.waitForZMotionComplete();
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        
        Logger::info("Commanding RC to P3 ");
        DS.MoveRC(3);
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P4 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {0, 1, 0, 0}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //4
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        // DS.waitForZMotionComplete();
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P5 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {0, 1, 0, 1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //5
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P6 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {0, 1, 1, 0}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //6
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P7 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {0, 1, 1, 1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //7
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P8 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {1, 0, 0, 0}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //8
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P9 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {1, 0, 0, 1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //9
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P10 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {1, 0, 1, 0}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //10
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P11 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {1, 0, 1, 1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //11
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P 12");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {1, 1, 0, 0}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //12
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P13 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {1, 1, 0, 1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //13
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P14 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {1, 1, 1, 0}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //14
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P 15");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {1, 1, 1, 1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //15
        // DS.waitForZMotionComplete();
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        Logger::info("Z Motion Complete: " + DS.zMotionComplete());

        Logger::info("Commanding RC to P0 ");
        SEL_Interface::SetOutputs({306, 305, 304, 303}, {0, 0, 0, 0}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {1}, DS.SEL_outputs);
        SEL_Interface::SetOutputs({302}, {0}, DS.SEL_outputs); //0
        DS.waitForZMotionComplete();
        // std::this_thread::sleep_for(std::chrono::milliseconds(10000));

        // SEL_Interface::Home(SEL_Interface::Axis::XY);
        // DS.waitForMotionComplete();

        // SEL_Interface::MoveToPosition({0.0, 0.0});
        // DS.waitForMotionComplete();

        // SEL_Interface::MoveToPosition({0.0, 200.0});
        // DS.waitForMotionComplete();

        // SEL_Interface::MoveToPosition({100.0, 200.0});
        // DS.waitForMotionComplete();

        // SEL_Interface::MoveToPosition({100.0, 0.0});
        // DS.waitForMotionComplete();

        // SEL_Interface::MoveToPosition({200.0, 0.0});
        // DS.waitForMotionComplete();

        // SEL_Interface::MoveToPosition({200.0, 200.0});
        // DS.waitForMotionComplete();

        // SEL_Interface::MoveToPosition({0.0, 0.0});
        // DS.waitForMotionComplete();

        // SEL_Interface::HaltAll();

        SEL->Close();
        Logger::info("All done!");
    }

    catch (const std::exception& e) {
        Logger::error("Exception caught. " + std::string(e.what()) + "\nAttempting to exit gracefully.");
        SEL_Interface::HaltAll();
        SEL->Close();
        Logger::info("Shutdown complete, re-throwing the error.");
        throw;
    }

    catch (...) {
        Logger::error("Unknown exception caught. Attempting to exit gracefully.");
        SEL_Interface::HaltAll();
        SEL->Close();
        Logger::info("Shutdown complete, re-throwing the error.");
        throw;
    }

    return 0;
}
