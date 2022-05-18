/******************************************************************************
 * 
 * Copyright (C) 2022 Dmitry Plastinin
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

Gpio::~Gpio() {
    close();
}

/******************************************************************************
 * Methods
 *****************************************************************************/

int Gpio::open(const std::string& dev) {
    std::unique_lock lock(m_workerThreadMutex);

    // Check opening
    if (m_fd) {
        return kAlreadyOpened;
    }

    // Create pipe
    int ret = pipe(m_pipe);
    if (ret == -1) {
        return kFailedToCreatePipe;
    }

    // Open device
    m_fd = ::open(dev.c_str(), O_RDWR);
    if (m_fd == -1) {
        m_fd = 0;
        return kFailedToOpen;
    }

    m_pollFds.clear();
    m_pollFds.emplace_back( pollfd { m_pipe[0], POLLIN } );

    m_threadRunning = true;
    m_pollingThread = new std::thread(&Gpio::polling, this);

    return kSuccess;
}

void Gpio::close() {
    std::unique_lock lock(m_workerThreadMutex);

    m_threadRunning = false;

    char code = '0';
    write(m_pipe[1], &code, sizeof(code));

    m_pollingThread->join();
    delete m_pollingThread;
    m_pollingThread = nullptr;

    ::close(m_pipe[0]);
    ::close(m_pipe[1]);
    ::close(m_fd);

    m_pipe[0] = 0;
    m_pipe[1] = 0;
    m_fd = 0;
}

/******************************************************************************
 * Accessors / Mutators
 *****************************************************************************/

Gpio::Value Gpio::value(int pin) {
    if (m_fd == 0) {
        onError(kDeviceNotOpened);
        return kIdle;
    }

    struct gpio_v2_line_values lineValues;
    memset(&lineValues, 0, sizeof(lineValues));
    lineValues.mask = 1;

    int ret = ioctl(m_fdsByPins[pin], GPIO_V2_LINE_GET_VALUES_IOCTL, &lineValues);
    if (ret == -1) {
        onError(kFailedToGetValue);
        return kIdle;
    }
    
    if (lineValues.bits) {
        return kHigh;
    }
    return kLow;
}

void Gpio::setValue(int pin, Value value) {
    if (m_fd == 0) {
        onError(kDeviceNotOpened);
        return;
    }

    if (m_fdsByPins.find(pin) == m_fdsByPins.end() 
        || m_directionsByPins[pin] != kOutput) {
        onError(kPinIsNotOutput);
        return;
    }

    struct gpio_v2_line_config config;
    memset(&config, 0, sizeof(config));

    config.flags = GPIO_V2_LINE_FLAG_OUTPUT;

    switch (value) {
    case kLow:
        config.flags &= ~GPIO_V2_LINE_FLAG_ACTIVE_LOW;
        break;

    case kHigh:
        config.flags |= GPIO_V2_LINE_FLAG_ACTIVE_LOW;
        break;

    default:
        onError(kInvalidValue);
        return;
    }

    int ret = ioctl(m_fdsByPins[pin], GPIO_V2_LINE_SET_CONFIG_IOCTL, &config);
    if (ret == -1) {
        onError(kFailedToSetValue);
        return;
    }
}

void Gpio::setDirection(int pin, Direction mode) {
    if (m_fd == 0) {
        onError(kDeviceNotOpened);
        return;
    }

    if (m_fdsByPins.find(pin) != m_fdsByPins.end()) {
        // Send "soft" interrupt to pause polling thread
        std::unique_lock lock(m_interruptMutex);
        char code = '0';
        write(m_pipe[1], &code, sizeof(char));

        m_pollfdsMutex.lock();
        for (size_t i = 0; i < m_pollFds.size(); ++i) {
            if (m_fdsByPins[pin] != m_pollFds[i].fd) {
                continue;
            }

            ::close(m_fdsByPins[pin]);
            m_fdsByPins.erase(pin);
            m_pollFds.erase(m_pollFds.begin() + i);

            break;
        }
        m_pollfdsMutex.unlock();
    }

    // Create empty request
    struct gpio_v2_line_request request;
    memset(&request, 0, sizeof(gpio_v2_line_request));

    request.offsets[0] = pin;
    request.num_lines = 1;

    switch (mode) {
    case kInput:
        request.config.flags = 
            GPIO_V2_LINE_FLAG_INPUT | 
            GPIO_V2_LINE_FLAG_EDGE_RISING | 
            GPIO_V2_LINE_FLAG_EDGE_FALLING;
        break;

    case Direction::kOutput:
        request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
        break;
    default:
        throw std::runtime_error("GPIO::SET_DIRECTION: Invalid direction");
        break;
    }

    int ret = ioctl(m_fd, GPIO_V2_GET_LINE_IOCTL, &request);
    if (ret == -1 || request.fd == -1) {
        onError(OpCodes::kFailedToSetDirection);
        return;
    }

    // Store info about pin
    m_fdsByPins[pin] = request.fd;
    m_pinsByFds[request.fd] = pin;
    m_directionsByPins[pin] = mode;

    // Add polling if input
    if (mode != Direction::kInput) {
        return;
    }

    // Send "soft" interrupt to pause polling thread
    std::unique_lock lock(m_interruptMutex);
    char code = '0';
    write(m_pipe[1], &code, sizeof(char));

    m_pollfdsMutex.lock();
    m_pollFds.push_back( pollfd { request.fd, POLLIN } );
    m_pollfdsMutex.unlock();
}

void Gpio::setBiasMode(int pin, BiasMode mode) {
    if (m_fd == 0) {
        onError(OpCodes::kDeviceNotOpened);
        return;
    }

    if (m_fdsByPins.find(pin) == m_fdsByPins.end()
        || m_directionsByPins[pin] != Direction::kInput) {
        onError(OpCodes::kPinIsNotInput);
        return;
    }

    struct gpio_v2_line_config config;
    memset(&config, 0, sizeof(config));

    config.flags = 
        GPIO_V2_LINE_FLAG_INPUT | 
        GPIO_V2_LINE_FLAG_EDGE_RISING | 
        GPIO_V2_LINE_FLAG_EDGE_FALLING;

    switch (mode) {
    case BiasMode::kPullUp:
        config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
        break;

    case BiasMode::kPullDown:
        config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
        break;
    }

    int ret = ioctl(m_fdsByPins[pin], GPIO_V2_LINE_SET_CONFIG_IOCTL, &config);
    if (ret == -1) {
        onError(OpCodes::kFailedToSetBiasMode);
        return;
    }
}

/******************************************************************************
 * Methods (Protected)
 *****************************************************************************/

void Gpio::polling() {
    int ret = 0;
    size_t i = 0;
    struct gpio_v2_line_event event;
    char buf;

    while (m_threadRunning) {
        m_pollfdsMutex.lock();
        ret = poll(m_pollFds.data(), m_pollFds.size(), -1);
        m_pollfdsMutex.unlock();

        if (ret == -1) {
            onError(OpCodes::kPollingError);
            continue;
        }

        // Check "soft" interrupt
        if (m_pollFds[0].revents & POLLIN) {
            m_pollFds[0].revents = 0;
            m_interruptMutex.lock();
            read(m_pipe[0], &buf, sizeof(buf));
            m_interruptMutex.unlock();
        }

        m_pollfdsMutex.lock();
        for (i = 1; i < m_pollFds.size(); ++i) {
            if (!(m_pollFds[i].revents & POLLIN)) {
                continue;
            }

            m_pollFds[i].revents = 0;

            memset(&event, 0 ,sizeof(gpio_v2_line_event));
            read(m_pollFds[i].fd, &event, sizeof(event));

            // Check previous pin value
            if (m_valuesByFds[m_pollFds[i].fd] == event.id) {
                continue;
            }
            m_valuesByFds[m_pollFds[i].fd] = event.id;

            switch (event.id) {
            case GPIO_V2_LINE_EVENT_RISING_EDGE:
                onInputChanged(m_pinsByFds[m_pollFds[i].fd], kHigh);
                break;

            case GPIO_V2_LINE_EVENT_FALLING_EDGE:
                onInputChanged(m_pinsByFds[m_pollFds[i].fd], kLow);
                break;
            }
        }
        m_pollfdsMutex.unlock();
    }
}

} // namespace UT