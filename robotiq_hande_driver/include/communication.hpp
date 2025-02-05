#ifndef COMMUNICATION_HPP_
#define COMMUNICATION_HPP_

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
enum OutputBytes{
    OUTPUT_BYTES_RESERVED_1,
    OUTPUT_BYTES_ACTION_REQUEST,
    OUTPUT_BYTES_POSITION_REQUEST,
    OUTPUT_BYTES_RESERVED_2,
    OUTPUT_BYTES_FORCE,
    OUTPUT_BYTES_SPEED,
    OUTPUT_BYTES_MAX
};

enum InputBytes{
    INPUT_BYTES_RESERVED_1,
    INPUT_BYTES_GRIPPER_STATUS,
    INPUT_BYTES_POSITION_REQUEST_ECHO,
    INPUT_BYTES_FAULT_STATUS,
    INPUT_BYTES_CURRENT,
    INPUT_BYTES_POSITION,
    INPUT_BYTES_MAX
};

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
    void connect();

    /**
     * @brief Disconnect from gripper using modbus rtu and virtual socket
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void disconnect();

    /**
     * @brief Read and write input.output registers at once
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void read_write_registers();

    /**
     * @brief Set output bytes to zeros
     *
     * @param none
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void clear_output_bytes();

    /**
     * @brief Get input byte value
     *
     * @param index InputBytes byte index
     * @return requested byte value
     * @note see status on success, exception thrown in case of communication issues
     */
    uint8_t get_input_byte(InputBytes index);

    /**
     * @brief Set output byte value
     *
     * @param index OutputBytes byte index
     * @param value to be set
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void set_output_byte(OutputBytes index, uint8_t value);

    // uint16_t input_registers_[kRegisterWordLength];
    // uint16_t output_registers_[kRegisterWordLength];

private:
    modbus_t *mb_;

    uint8_t input_bytes_[INPUT_BYTES_MAX];
    uint8_t output_bytes_[OUTPUT_BYTES_MAX];

};
}   // namespace hande_driver
#endif  // COMMUNICATION_HPP_
