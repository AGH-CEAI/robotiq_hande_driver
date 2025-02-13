#ifndef ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_
#define ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_

#include <modbus/modbus.h>
#include <cstring>
#include <string>

#include <rclcpp/rclcpp.hpp>


namespace robotiq_hande_driver
{

constexpr auto DEBUG_MODBUS = true;

constexpr uint16_t GRIPPER_OUTPUT_FIRST_REG = 0x07D0;
constexpr uint16_t GRIPPER_INPUT_FIRST_REG = 0x03E8;

enum class OutputBytes : uint8_t {
    RESERVED_1 = 0u,
    ACTION_REQUEST,
    POSITION_REQUEST,
    RESERVED_2,
    FORCE,
    SPEED,
    BYTES_MAX
};
constexpr auto OUTPUT_REGISTER_WORD_LENGTH = static_cast<uint>(OutputBytes::BYTES_MAX) / 2;

enum class InputBytes : uint8_t {
    RESERVED_1 = 0u,
    GRIPPER_STATUS,
    POSITION_REQUEST_ECHO,
    FAULT_STATUS,
    CURRENT,
    POSITION,
    BYTES_MAX
};
constexpr auto INPUT_REGISTER_WORD_LENGTH = static_cast<uint>(InputBytes::BYTES_MAX) / 2;


/**
 * @brief This class contains low level gripper commands and status
 */
class Communication{
public:
    Communication();

    ~Communication() {
        cleanup();
    };

    /**
     * @brief Initializes driver parameters.
     *
     * @param tty_port Modbus virtual port.
     * @param baudrate Modbus serial baudrate.
     * @param parity Modbus serial parity.
     * @param data_bits Modbus serial data bits.
     * @param stop_bit Modbus serial stopbit.
     * @param slave_id Modbus slave id.
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void initialize(std::string tty_port, int baudrate, char parity, int data_bits, int stop_bit, int slave_id) {
        tty_port_ = tty_port.c_str();
        baudrate_ = baudrate;
        parity_ = parity;
        data_bits_ = data_bits;
        stop_bit_ = stop_bit;
        slave_id_ = slave_id;
    };

    /**
     * @brief Initializes communication layer.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void configure() {
        mb_ = modbus_new_rtu(tty_port_, baudrate_, parity_, data_bits_, stop_bit_);
        modbus_set_slave(mb_, slave_id_);
        modbus_set_debug(mb_, DEBUG_MODBUS);
        connect();
    };

    /**
     * @brief Deinitializes communication layer.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void cleanup() {
        disconnect();
        modbus_free(mb_);
    };

    /**
     * @brief Connects to the gripper using Modbus RTU and a virtual socket.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void connect();

    /**
     * @brief Disconnects from the gripper using Modbus RTU and a virtual socket.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void disconnect() {
        modbus_close(mb_);
    };

    /**
     * @brief Reads and writes input/output registers at once.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void read_write_registers() {
    /**
     * @brief Read and write modbus registers at once
     *
     * @param modbus_t *ctx
     * @param int write_addr
     * @param int write_nb
     * @param uint16_t *src
     * @param int read_addr
     * @param int read_nb
     * @param uint16_t *dest
     * @return int >0 on success
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    modbus_write_and_read_registers(mb_,
                                    GRIPPER_INPUT_FIRST_REG,
                                    OUTPUT_REGISTER_WORD_LENGTH,
                                    reinterpret_cast<uint16_t*>(output_bytes_),
                                    GRIPPER_OUTPUT_FIRST_REG,
                                    INPUT_REGISTER_WORD_LENGTH,
                                    reinterpret_cast<uint16_t*>(input_bytes_));
    };

    /**
     * @brief Sets output bytes to zeros.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void clear_output_bytes() {
        memset(output_bytes_, 0, sizeof(output_bytes_));
    };

    /**
     * @brief Retrieves the input byte value at the specified index.
     *
     * @param index The InputBytes byte index.
     * @return Requested byte value.
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    uint8_t get_input_byte(InputBytes index) {
        return input_bytes_[static_cast<uint>(index)];
    };

    /**
     * @brief Sets the output byte value at the specified index.
     *
     * @param index OutputBytes byte index
     * @param value The value to be set.
     * @return none
     * @note The status should be checked to verify successful execution. An exception is thrown if communication issues occur.
     */
    void set_output_byte(OutputBytes index, uint8_t value) {
        output_bytes_[static_cast<uint>(index)] = value;
    };

    /**
     * @brief Sets the n-th bit of the action request byte.
     *
     * @param position_bit n-th bit in byte
     * @param value The boolean value to set for the bit.
     * @return None.
     */
    void write_action_bit(uint8_t position_bit, bool value) {
        output_bytes_[static_cast<uint>(OutputBytes::ACTION_REQUEST)] = bit_set_to(
            output_bytes_[static_cast<uint>(OutputBytes::ACTION_REQUEST)], position_bit, value);
    };

    /**
     * @brief Sets the n-th bit of a number to the specified boolean value.
     *
     * @param number
     * @param n The n-th bit position to set.
     * @param x The boolean value to set the bit to.
     * @return The modified number with the n-th bit set to the specified value.
     */
    uint bit_set_to(uint value, uint n, bool x) {
        uint reset_n_bit = ~(1u << n);
        uint set_n_bit = static_cast<uint>(x) << n;
        return (value & reset_n_bit) | set_n_bit;
    };

private:
    const char* tty_port_;
    int baudrate_;
    char parity_;
    int data_bits_;
    int stop_bit_;
    int slave_id_;

    modbus_t *mb_;

    uint8_t input_bytes_[static_cast<size_t>(InputBytes::BYTES_MAX)];
    uint8_t output_bytes_[static_cast<size_t>(OutputBytes::BYTES_MAX)];

};
}   // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_
