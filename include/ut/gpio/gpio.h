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
    enum class Opcode;
    enum class Value;
    enum class Direction;
    enum class PullMode;

    /**************************************************************************
     * Constructors / Destructors
     *************************************************************************/

    GPIO() = default;
    GPIO(const GPIO& other) = delete;
    GPIO(GPIO&& other) = delete;
    ~GPIO();

    /**************************************************************************
     * Methods
     *************************************************************************/

    /**
     * @brief Open GPIO device
     * 
     * @param dev path to the device
     * @return GPIO::Opcode
     *     - kSuccess - device successfully open
     *     - kAlreadyOpen - the instance has already opened a GPIO device
     *     - kSyscallError - pipe(...) or open(...) failed
     */
    Opcode open(const std::string& dev);

    void close();

    /**************************************************************************
     * Accessors / Mutators
     *************************************************************************/

    /**
     * @brief Get current value of input pin
     * 
     * @param pin pin to change
     * @return GPIO::Value
     *     - kLow - logical 0
     *     - kHigh - logical 1
     *     - kIdle - undefined value or error occured
     */
    Value getValue(int pin);

    /**
     * @brief Set output pin value
     * 
     * @param pin pin to change
     * @param value value to set
     * @return GPIO::Opcode 
     *     - kSuccess - value successfully set
     *     - kDeviceNotOpen - GPIO device is not open
     *     - kInvalidValue - attempt to set an invalid value
     *     - kPinNotOutput - pin is not configured as an output
     *     - kSyscallerror - ioctl(...) failed
     */
    Opcode setValue(int pin, Value value);

    /**
     * @brief Set direction (input or output) for pin
     * 
     * @param pin pin to change
     * @param direction direction
     * @return GPIO::Opcode
     *     - kSuccess - direction successfully changed
     *     - kDeviceNotOpen - GPIO device is not open
     *     - kSyscallerror - ioctl(...) failed
     */
    Opcode setDirection(int pin, Direction direction);

    /**
     * @brief Set pull mode (pull-up or pull-down) for a input pin
     * 
     * @param pin pin to change
     * @param mode pull mode
     * @return GPIO::Opcode
     *     - kSuccess - pull mode successfully set
     *     - kDeviceNotOpen - GPIO device is not open
     *     - kPinNotInput - pin is not configured as an input
     *     - kSyscallerror - ioctl(...) failed
     */
    Opcode setPullMode(int pin, PullMode mode);

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
    int mFd = -1;
    int mPipe[2] = { 0, 0 };
    std::map<int, Direction> mDirectionsByPins;
    std::map<int, int> mFdsByPins; /// for quick access to GPIO num from polling
    std::map<int, int> mPinsByFds; /// for quick access to GPIO num from polling
    std::map<int, unsigned int> mValuesByFds;
    std::mutex mInterruptMutex;
    std::mutex mOpenCloseMutex;
    std::mutex mPfdsMutex;
    std::thread* mPollingThread = nullptr;
    std::vector<pollfd> mPfds;

}; // class GPIO

enum class GPIO::Opcode {
    kSuccess,
    kAlreadyOpen,
    kDeviceNotOpen,
    kPinNotOutput,
    kPinNotInput,
    kInvalidValue,
    kSyscallError
}; // enum class GPIO::Opcode

enum class GPIO::Value {
    kLow,
    kHigh,
    kIdle
}; // enum class GPIO::Value

enum class GPIO::Direction {
    kInput,
    kOutput
}; // enum class GPIO::Direction

enum class GPIO::PullMode {
    kPullDown,
    kPullUp
}; // enum class GPIO::Type

} // namespace UT

#endif // UT_GPIO_H