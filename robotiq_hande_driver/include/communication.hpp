#ifndef COMMUNICATION_HPP_
#define COMMUNICATION_HPP_

#include <modbus/modbus.h>


namespace hande_driver
{
constexpr auto kRegisterWordLength = 3;
enum class OutputBytes : uint8_t {
    RESERVED_1 = 0u,
    ACTION_REQUEST,
    POSITION_REQUEST,
    RESERVED_2,
    FORCE,
    SPEED,
    BYTES_MAX
};

enum class InputBytes : uint8_t {
    RESERVED_1 = 0u,
    GRIPPER_STATUS,
    POSITION_REQUEST_ECHO,
    FAULT_STATUS,
    CURRENT,
    POSITION,
    BYTES_MAX
};


/**
 * @brief This class contains low level gripper commands and status
 */
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

    /**
     * @brief Set n-th bit to action request byte
     *
     * @param position_bit n-th bit in byte
     * @param value bool value
     * @return none
     */
    void write_action_bit(uint8_t position_bit, bool value);

    /**
     * @brief Set n-th bit to x value
     *
     * @param number
     * @param n n-th bit in number
     * @param x bool value
     * @return uint number with n-th bit set to x
     */
    uint bit_set_to(uint number, uint n, bool x);

private:
    modbus_t *mb_;

    uint8_t input_bytes_[static_cast<size_t>(InputBytes::BYTES_MAX)];
    uint8_t output_bytes_[static_cast<size_t>(OutputBytes::BYTES_MAX)];

};
}   // namespace hande_driver
#endif  // COMMUNICATION_HPP_
