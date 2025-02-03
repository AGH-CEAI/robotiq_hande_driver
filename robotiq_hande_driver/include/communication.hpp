#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <stdint.h>
#include <stddef.h>
#include <vector>
#include <modbus/modbus.h>

/**
 * @brief This class contains low level gripper commands and status
 */

namespace hande_driver
{
constexpr auto kRegisterWordLength = 3;

class Communication{
public:
    Communication();

    ~Communication();

    /**
     * @brief Connect to gripper using modbus rtu and virtual socket
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void Connect();

    /**
     * @brief Disconnect from gripper using modbus rtu and virtual socket
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void Disconnect();

    /**
     * @brief Read and write input.output registers at once
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void ReadWiteRegisters();

    uint16_t input_registers_[kRegisterWordLength];
    uint16_t output_registers_[kRegisterWordLength];

private:
    modbus_t *mb_;

};
}   // namespace hande_driver
#endif // COMMUNICATION_H_
