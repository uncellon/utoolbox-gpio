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
auto ret = gpio.open("/dev/gpiochip0");
if (ret != UT::GPIO::Opcode::kSuccess) {
    std::cerr << "Failed to open UT::GPIO device\n";
    return -1;
}
...
```

### Set pin direction

```cpp
...
ret = gpio.setDirection(4, UT::GPIO::Direction::kOutput); // Set pin with number 4 as output
...
ret = gpio.setDirection(5, UT::GPIO::Direction::kInput); // Set pin with number 5 as input
...
```

### Set pull mode

```cpp
...
ret = gpio.setPullMode(5, UT::GPIO::PullMode::kPullUp); // Set pin 5 in pull-up mode
...
```

### Set output value

```cpp
ret = gpio.setValue(4, UT::GPIO::Value::kLow);
```

### Get output value

```cpp
auto value = gpio.getValue(5);
```

### Watch inputs changes

```cpp
gpio.onInputChanged.addEventHandler(EventLoop::getMainInstance(), 
    [] (int pin, UT::GPIO::Value value) {
        switch (value) {
        case UT::GPIO::Value::kLow:
            std::cout << pin << " " << "low\n";
            break;
        case UT::GPIO::Value::kHigh:
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