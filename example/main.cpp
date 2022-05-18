#include <iostream>
#include <unistd.h>
#include <ut/gpio/gpio.h>

using namespace UT;

int main(int argc, char *argv[]) {
    EventLoop ev;

    Gpio gpio;
    
    // Register error handler
    gpio.onError.addEventHandler(&ev, 
        [] (Gpio::OpCodes error) {
            std::cout << "error occurred: ";
            switch (error) {
            case Gpio::OpCodes::kFailedToOpen:
                std::cout << "failed to open device\n";
                break;
            
            case Gpio::OpCodes::kFailedToSetDirection:
                std::cout << "set pin mode failed\n";
                break;

            case Gpio::OpCodes::kFailedToSetValue:
                std::cout << "set pin value failed\n";
                break;

            case Gpio::OpCodes::kPinIsNotOutput:
                std::cout << "attempt to set value of non-output pin\n";
                break;

            default:
                std::cout << "unknown error\n";
                break;
            }
            exit(EXIT_FAILURE);
        }
    );

    // Register input changed handler
    gpio.onInputChanged.addEventHandler(&ev, 
        [] (int pin, Gpio::Value value) {
            switch (value) {
            case Gpio::kLow:
                std::cout << pin << " " << "low\n";
                break;

            case Gpio::kHigh:
                std::cout << pin << " " << "high\n";
                break;
            default:
                break;
            }
        }
    );

    // Open device
    gpio.open("/dev/gpiochip0");

    // Set gpio 4 output mode
    gpio.setDirection(4, Gpio::kOutput);

    // Set gpio 4 output value
    gpio.setValue(4, Gpio::kLow);

    // Set gpio 21 input mode
    gpio.setDirection(21, Gpio::kInput);

    // Set gpio 21 bias mode
    gpio.setBiasMode(21, Gpio::kPullUp);

    // Get current 21 value
    auto current = gpio.value(21);
    switch (current) {
        case Gpio::Value::kHigh:
            std::cout << "21 is high now\n";
            break;
        case Gpio::Value::kLow:
            std::cout << "21 is low now\n";
            break;
        default:
            break;
    }

    const char *help = 
        "Help:\n"
        "    y - switch pin to on\n"
        "    n - switch pin to off\n"
        "    h - print help\n"
        "    q - quit\n";

    char input;
    std::cout << help;
    while (true) {
        std::cin >> input;
        if (input == 'y') {
            std::cout << "switch to on\n";
            gpio.setValue(4, Gpio::Value::kHigh);
        } else if (input == 'n') {
            std::cout << "switch to off\n";
            std::cout << "n\n";
            gpio.setValue(4, Gpio::Value::kLow);
        } else if (input == 'h') {
            std::cout << help;
        } else if (input == 'q') {
            break;
        } else {
            std::cout << "wrong input\n";
        }
    }

    return EXIT_SUCCESS;
}