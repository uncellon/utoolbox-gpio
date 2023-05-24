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

#ifndef UT_GPIO_H
#define UT_GPIO_H

#include <map>
#include <sys/poll.h>
#include <ut/core/event.h>

namespace UT {

class GPIO {
public:
    enum Value {
        LOW,
        HIGH
    };

    enum Direction {
        INPUT,
        OUTPUT
    };
    
    enum Type {
        PULL_DOWN,
        PULL_UP
    };

    /**************************************************************************
     * Constructors / Destructors
     *************************************************************************/

    GPIO() = default;
    GPIO(const GPIO&) = delete;
    GPIO(GPIO&&) = delete;
    ~GPIO();

    /**************************************************************************
     * Methods
     *************************************************************************/

    void open(const std::string& dev);
    void close();

    /**************************************************************************
     * Accessors / Mutators
     *************************************************************************/

    Value getValue(int pin);
    void setValue(int pin, Value value);

    void setDirection(int pin, Direction direction);

    void setPullMode(int pin, Type mode);

    /**************************************************************************
     * Events
     *************************************************************************/

    Event<int, Value> onInputChanged;

protected:
    /**************************************************************************
     * Methods (Protected)
     *************************************************************************/

    void polling();

    /**************************************************************************
     * Members
     *************************************************************************/

    bool mRunning = false;
    int mFd = 0;
    int mPipe[2] = { 0, 0 };
    std::map<int, Direction> mDirectionsByPins;
    std::map<int, int> mFdsByPins; /// for quick access to GPIO num from polling
    std::map<int, int> mPinsByFds; /// for quick access to GPIO num from polling
    std::map<int, unsigned int> mValuesByFds;
    std::mutex mInterruptMutex;
    std::mutex mPfdsMutex;
    std::mutex mOpenCloseMutex;
    std::thread* mPollingThread = nullptr;
    std::vector<pollfd> mPfds;

}; // class GPIO

} // namespace UT

#endif // UT_GPIO_H