# Uncellon's Toolbox GPIO

![UToolbox Logo](logo.png)

- [Description](#description)
- [Prerequisites](#prerequisites)
- [Examples](#examples)
    - [Connect to the GPIO](#connect-to-the-gpio)
    - [Set pin direction](#set-pin-direction)
    - [Set pull mode](#set-pull-mode)
    - [Set output value](#set-output-value)
    - [Get output value](#get-output-value)
    - [Watch inputs changes](#watch-inputs-changes)
- [License](#license)

## Description

The UToolbox GPIO library implements an event-driven interface for accessing GPIOs (for example, Raspberry Pi) through the userspace ABI v2.

## Prerequisites

- Linux kernel >= 5.10
- C++17 or higher
- CMake >= 3.16
- [UToolbox Core](https://github.com/uncellon/utoolbox-core) >= 0.0.10

## Examples

### Connect to the GPIO

```cpp
...
UT::GPIO gpio;
try {
    gpio.open("/dev/gpiochip0");
} catch (const std::exception& e) {
    std::cerr << "GPIO error: " << e.what() << std::endl;
    return EXIT_FAILURE;
}
...
```

### Set pin direction

```cpp
...
gpio.setDirection(4, GPIO::OUTPUT); // Set pin with number 4 as output
gpio.setDirection(5, GPIO::INPUT); // Set pin with number 5 as input
...
```

### Set pull mode

```cpp
...
gpio.setPullMode(5, GPIO::PULL_UP); // Set pin 5 in pull-up mode
...
```

### Set output value

```cpp
gpio.setValue(4, GPIO::LOW);
```

### Get output value

```cpp
gpio.getValue(5);
```

### Watch inputs changes

```cpp
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
```

## License

<img align="right" src="https://www.gnu.org/graphics/lgplv3-with-text-154x68.png">

The library is licensed under [GNU Lesser General Public License 3.0](https://www.gnu.org/licenses/lgpl-3.0.txt):

Copyright © 2023 Dmitry Plastinin

UToolbox Timers is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as pubblished by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

UToolbox Timers is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser Public License for more details