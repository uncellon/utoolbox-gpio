#include <iostream>
#include <unistd.h>
#include <ut/gpio/gpio.h>

using namespace UT;

int main(int argc, char *argv[]) {
    EventLoop ev;

    GPIO gpio;

    // Register input changed handler
    gpio.onInputChanged.addEventHandler(EventLoop::getMainInstance(), 
        [] (int pin, GPIO::Value value) {
            switch (value) {
            case GPIO::LOW:
                std::cout << pin << " " << "low\n";
                break;

            case GPIO::HIGH:
                std::cout << pin << " " << "high\n";
                break;
            default:
                break;
            }
        }
    );

    // Open device
    try {
        gpio.open("/dev/gpiochip0");
    } catch (const std::exception& e) {
        std::cerr << "GPIO error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Set gpio 4 output mode
    gpio.setDirection(4, GPIO::OUTPUT);

    // Set gpio 4 output value
    gpio.setValue(4, GPIO::LOW);

    // Set gpio 21 input mode
    gpio.setDirection(21, GPIO::INPUT);

    // Set gpio 21 bias mode
    gpio.setPullMode(21, GPIO::PULL_UP);

    // Get current 21 value
    auto current = gpio.getValue(21);
    switch (current) {
        case GPIO::Value::HIGH:
            std::cout << "21 is high now\n";
            break;
        case GPIO::Value::LOW:
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
            gpio.setValue(4, GPIO::Value::HIGH);
        } else if (input == 'n') {
            std::cout << "switch to off\n";
            std::cout << "n\n";
            gpio.setValue(4, GPIO::Value::LOW);
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