/******************************************************************************
 * 
 * Copyright (C) 2023 Dmitry Plastinin
 * Contact: uncellon@yandex.ru, uncellon@gmail.com, uncellon@mail.ru
 * 
 * This file is part of the UT GPIO library.
 * 
 * UT GPIO is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as pubblished by the
 * Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * UT GPIO is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser Public License for more
 * details
 * 
 * You should have received a copy of the GNU Lesset General Public License
 * along with UT GPIO. If not, see <https://www.gnu.org/licenses/>.
 * 
 *****************************************************************************/

#include "gpio.h"

#include <cstring>
#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace UT {

/******************************************************************************
 * Constructors / Destructors
 *****************************************************************************/

GPIO::~GPIO() {
    close();
}

/******************************************************************************
 * Methods
 *****************************************************************************/

GPIO::Opcode GPIO::open(const std::string& dev) {
    std::unique_lock lock(mOpenCloseMutex);

    if (mRunning) {
        return Opcode::kAlreadyOpen;
    }

    // Create pipe
    int ret = pipe(mPipe);
    if (ret == -1) {
        return Opcode::kSyscallError;
    }

    // Open device
    mFd = ::open(dev.c_str(), O_RDWR);
    if (mFd == -1) {
        ::close(mPipe[0]);
        ::close(mPipe[1]);
        mPipe[0] = -1;
        mPipe[1] = -1;
        switch (errno) {
        case ENOENT:
            return Opcode::kDeviceNotFound;
        default:
            return Opcode::kSyscallError;
        }
    }

    mPfds.clear();
    mPfds.emplace_back( pollfd { mPipe[0], POLLIN } );

    mRunning = true;
    mPollingThread = new std::thread(&GPIO::polling, this);

    return Opcode::kSuccess;
}

void GPIO::close() {
    std::unique_lock lock(mOpenCloseMutex);

    if (!mRunning) {
        return;
    }
    mRunning = false;

    char code = '0';
    write(mPipe[1], &code, sizeof(code));

    if (mPollingThread) {
        mPollingThread->join();
    }
    delete mPollingThread;
    mPollingThread = nullptr;

    for (auto it = mPinsByFds.begin(); it != mPinsByFds.end(); ++it) {
        ::close(it->first);
    }
    mPfds.clear();

    ::close(mPipe[0]);
    ::close(mPipe[1]);
    ::close(mFd);

    mPipe[0] = 0;
    mPipe[1] = 0;
    mFd = -1;
}

/******************************************************************************
 * Accessors / Mutators
 *****************************************************************************/

GPIO::Value GPIO::getValue(int pin) {
    if (mFd == -1) {
        return Value::kIdle;
    }

    struct gpio_v2_line_values lineValues;
    memset(&lineValues, 0, sizeof(lineValues));
    lineValues.mask = 1;

    int ret = ioctl(mFdsByPins[pin], GPIO_V2_LINE_GET_VALUES_IOCTL, &lineValues);
    if (ret == -1) {
        return Value::kIdle;
    }
    
    if (lineValues.bits) {
        return Value::kHigh;
    }
    return Value::kLow;
}

GPIO::Opcode GPIO::setValue(int pin, Value value) {
    if (!mRunning) {
        return Opcode::kDeviceNotOpen;
    }

    if (mFdsByPins.find(pin) == mFdsByPins.end() || 
        mDirectionsByPins[pin] != Direction::kOutput) {
        return Opcode::kPinNotOutput;
    }

    struct gpio_v2_line_config config;
    memset(&config, 0, sizeof(config));

    config.flags = GPIO_V2_LINE_FLAG_OUTPUT;

    switch (value) {
    case Value::kLow:
        config.flags &= ~GPIO_V2_LINE_FLAG_ACTIVE_LOW;
        break;

    case Value::kHigh:
        config.flags |= GPIO_V2_LINE_FLAG_ACTIVE_LOW;
        break;
    default:
        return Opcode::kInvalidValue;
    }

    int ret = ioctl(mFdsByPins[pin], GPIO_V2_LINE_SET_CONFIG_IOCTL, &config);
    if (ret == -1) {
        return Opcode::kSyscallError;
    }

    return Opcode::kSuccess;
}

GPIO::Opcode GPIO::setDirection(int pin, Direction mode) {
    if (!mRunning) {
        return Opcode::kDeviceNotOpen;
    }

    if (mFdsByPins.find(pin) != mFdsByPins.end()) {
        // Send "soft" interrupt to pause polling thread
        std::unique_lock lock(mInterruptMutex);
        char code = '0';
        write(mPipe[1], &code, sizeof(code));

        mPfdsMutex.lock();
        for (size_t i = 0; i < mPfds.size(); ++i) {
            if (mFdsByPins[pin] != mPfds[i].fd) {
                continue;
            }

            ::close(mFdsByPins[pin]);
            mFdsByPins.erase(pin);
            mPfds.erase(mPfds.begin() + i);

            break;
        }
        mPfdsMutex.unlock();
    }

    // Create empty request
    struct gpio_v2_line_request request;
    memset(&request, 0, sizeof(gpio_v2_line_request));

    request.offsets[0] = pin;
    request.num_lines = 1;

    switch (mode) {
    case Direction::kInput:
        request.config.flags = 
            GPIO_V2_LINE_FLAG_INPUT | 
            GPIO_V2_LINE_FLAG_EDGE_RISING | 
            GPIO_V2_LINE_FLAG_EDGE_FALLING;
        break;

    case Direction::kOutput:
        request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
        break;
    }

    int ret = ioctl(mFd, GPIO_V2_GET_LINE_IOCTL, &request);
    if (ret == -1 || request.fd == -1) {
        return Opcode::kSyscallError;
    }

    // Store info about pin
    mFdsByPins[pin] = request.fd;
    mPinsByFds[request.fd] = pin;
    mDirectionsByPins[pin] = mode;

    // Add polling if input
    if (mode != Direction::kInput) {
        return Opcode::kSuccess;
    }

    // Send "soft" interrupt to pause polling thread
    std::unique_lock lock(mInterruptMutex);
    char code = '0';
    write(mPipe[1], &code, sizeof(char));

    mPfdsMutex.lock();
    mPfds.push_back( pollfd { request.fd, POLLIN } );
    mPfdsMutex.unlock();

    return Opcode::kSuccess;
}

GPIO::Opcode GPIO::setPullMode(int pin, PullMode mode) {
    if (!mRunning) {
        return Opcode::kDeviceNotOpen;
    }

    if (mFdsByPins.find(pin) == mFdsByPins.end() || 
        mDirectionsByPins[pin] != Direction::kInput) {
        return Opcode::kPinNotInput;
    }

    struct gpio_v2_line_config config;
    memset(&config, 0, sizeof(config));

    config.flags = 
        GPIO_V2_LINE_FLAG_INPUT | 
        GPIO_V2_LINE_FLAG_EDGE_RISING | 
        GPIO_V2_LINE_FLAG_EDGE_FALLING;

    switch (mode) {
    case PullMode::kPullUp:
        config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
        break;
    case PullMode::kPullDown:
        config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
        break;
    }

    int ret = ioctl(mFdsByPins[pin], GPIO_V2_LINE_SET_CONFIG_IOCTL, &config);
    if (ret == -1) {        
        return Opcode::kSyscallError;
    }

    return Opcode::kSuccess;
}

/******************************************************************************
 * Methods (Protected)
 *****************************************************************************/

void GPIO::polling() {
    int ret = 0;
    size_t i = 0;
    struct gpio_v2_line_event event;
    char buf;

    while (mRunning) {
        mPfdsMutex.lock();
        ret = poll(mPfds.data(), mPfds.size(), -1);
        mPfdsMutex.unlock();

        if (ret == -1) {
            continue;
        }

        // Check "soft" interrupt
        if (mPfds[0].revents & POLLIN) {
            mPfds[0].revents = 0;
            mInterruptMutex.lock();
            read(mPipe[0], &buf, sizeof(buf));
            mInterruptMutex.unlock();
        }

        mPfdsMutex.lock();
        for (i = 1; i < mPfds.size(); ++i) {
            if (!(mPfds[i].revents & POLLIN)) {
                continue;
            }

            mPfds[i].revents = 0;

            memset(&event, 0 ,sizeof(gpio_v2_line_event));
            read(mPfds[i].fd, &event, sizeof(event));

            // Check previous pin value
            if (mValuesByFds[mPfds[i].fd] == event.id) {
                continue;
            }
            mValuesByFds[mPfds[i].fd] = event.id;

            switch (event.id) {
            case GPIO_V2_LINE_EVENT_RISING_EDGE:
                onInputChanged(mPinsByFds[mPfds[i].fd], Value::kHigh);
                break;

            case GPIO_V2_LINE_EVENT_FALLING_EDGE:
                onInputChanged(mPinsByFds[mPfds[i].fd], Value::kLow);
                break;
            }
        }
        mPfdsMutex.unlock();
    }
}

} // namespace UT