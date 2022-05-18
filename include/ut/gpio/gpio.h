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

#ifndef UT_GPIO_H
#define UT_GPIO_H

#include <map>
#include <sys/poll.h>
#include <ut/core/event.h>

namespace UT {

class Gpio {
public:
    enum OpCodes {
        kSuccess,
        kDeviceNotOpened,
        kFailedToOpen,
        kFailedToSetDirection,
        kFailedToSetValue,
        kPinIsNotOutput,
        kPinIsNotInput,
        kFailedToSetBiasMode,
        kFailedToGetValue,
        kInvalidValue,
        kAlreadyOpened,
        kFailedToCreatePipe,
        kPollingError
    };

    enum Value {
        kIdle,
        kLow,
        kHigh
    };

    enum Direction {
        kInput,
        kOutput
    };
    
    enum BiasMode {
        kPullDown,
        kPullUp
    };

    /**************************************************************************
     * Constructors / Destructors
     *************************************************************************/

    Gpio() = default;
    ~Gpio();

    /**************************************************************************
     * Methods
     *************************************************************************/

    int open(const std::string& dev);
    void close();

    /**************************************************************************
     * Accessors / Mutators
     *************************************************************************/

    Value value(int pin);
    void setValue(int pin, Value value);

    void setDirection(int pin, Direction direction);

    void setBiasMode(int pin, BiasMode mode);

    /**************************************************************************
     * Events
     *************************************************************************/

    Event<OpCodes> onError;
    Event<int, Value> onInputChanged;

protected:
    /**************************************************************************
     * Methods (Protected)
     *************************************************************************/

    void polling();

    /**************************************************************************
     * Members
     *************************************************************************/

    int m_fd = 0;
    int m_pipe[2] = { 0, 0 };

    bool m_threadRunning = false;

    std::map<int, int> m_fdsByPins; /// for quick access to GPIO num from polling
    std::map<int, int> m_pinsByFds; /// for quick access to GPIO num from polling
    std::map<int, Direction> m_directionsByPins;
    std::map<int, unsigned int> m_valuesByFds;

    std::thread* m_pollingThread = nullptr;
    std::vector<pollfd> m_pollFds;

    std::mutex m_interruptMutex;
    std::mutex m_workerThreadMutex;
    std::mutex m_pollfdsMutex;
}; // class Gpio

} // namespace UT

#endif // UT_GPIO_H