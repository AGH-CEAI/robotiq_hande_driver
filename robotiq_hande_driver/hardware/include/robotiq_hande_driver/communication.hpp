#ifndef ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_
#define ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_

#include <atomic>
#include <optional>
#include <string>
#include <thread>

#include <modbus/modbus.h>

namespace robotiq_hande_driver {

static constexpr auto DEBUG_MODBUS = false;
static constexpr auto FAILURE_MODBUS = -1;

static constexpr uint16_t GRIPPER_OUTPUT_FIRST_REG = 0x07D0;
static constexpr uint16_t GRIPPER_INPUT_FIRST_REG = 0x03E8;

enum class OutputBytes : uint8_t {
    RESERVED_1 = 0u,
    ACTION_REQUEST,
    POSITION_REQUEST,
    RESERVED_2,
    FORCE,
    SPEED,
    BYTES_MAX
};
static constexpr auto NUM_OF_OUTPUT_BYTES = static_cast<size_t>(OutputBytes::BYTES_MAX);
static constexpr auto OUTPUT_REGISTER_WORD_LENGTH = static_cast<uint>(OutputBytes::BYTES_MAX) / 2;

enum class InputBytes : uint8_t {
    RESERVED_1 = 0u,
    GRIPPER_STATUS,
    POSITION_REQUEST_ECHO,
    FAULT_STATUS,
    CURRENT,
    POSITION,
    BYTES_MAX
};
static constexpr auto NUM_OF_INPUT_BYTES = static_cast<size_t>(OutputBytes::BYTES_MAX);
static constexpr auto INPUT_REGISTER_WORD_LENGTH = static_cast<uint>(InputBytes::BYTES_MAX) / 2;

/**
 * @brief This class contains low level gripper commands and status
 */
class Communication {
   public:
    Communication();

    ~Communication();

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
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void initialize(
        const std::string& tty_port,
        int baudrate,
        char parity,
        int data_bits,
        int stop_bit,
        int slave_id);

    /**
     * @brief Reads and writes input/output registers at once.
     *
     * @param none
     * @return None.
     * @note Used in external thread.
     */
    void read_write_registers();

    /**
     * @brief Initializes communication layer.
     *
     * @param none
     * @return int Connection status code.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    int configure();

    /**
     * @brief Deinitializes communication layer.
     *
     * @param none
     * @return None.
     */
    void cleanup();

    /**
     * @brief Connects to the gripper using Modbus RTU and a virtual socket.
     *
     * @param none
     * @return connection status code.
     * @note The status should be checked to verify successful execution.
     */
    int connect();

    /**
     * @brief Disconnects from the gripper using Modbus RTU and a virtual socket.
     *
     * @param none
     * @return None.
     */
    void disconnect();

    /**
     * @brief Sets output bytes to zeros.
     *
     * @param none
     * @return None.
     */
    void clear_output_bytes();

    /**
     * @brief Retrieves the input byte value at the specified index.
     *
     * @param index The InputBytes byte index.
     * @return Requested byte value.
     */
    uint8_t get_input_byte(InputBytes index);

    /**
     * @brief Sets the output byte value at the specified index.
     *
     * @param index OutputBytes byte index
     * @param value The value to be set.
     * @return none
     */
    void set_output_byte(OutputBytes index, uint8_t value);

    /**
     * @brief Sets the n-th bit of the action request byte.
     *
     * @param position_bit n-th bit in byte
     * @param value The boolean value to set for the bit.
     * @return None.
     */
    void write_action_bit(uint8_t position_bit, bool value);

    /**
     * @brief Sets the n-th bit of a number to the specified boolean value.
     *
     * @param number
     * @param n The n-th bit position to set.
     * @param x The boolean value to set the bit to.
     * @return The modified number with the n-th bit set to the specified value.
     */
    uint bit_set_to(uint value, uint n, bool x);

   private:
    std::string tty_port_;
    int baudrate_;
    char parity_;
    int data_bits_;
    int stop_bit_;
    int slave_id_;

    modbus_t* mb_;

    uint8_t input_bytes_modbus_[NUM_OF_INPUT_BYTES];
    uint8_t output_bytes_modbus_[NUM_OF_OUTPUT_BYTES];

    std::atomic<uint8_t> input_bytes_[NUM_OF_INPUT_BYTES];
    std::atomic<uint8_t> output_bytes_[NUM_OF_OUTPUT_BYTES];

    std::atomic<bool> bg_comm_enabled_;
    std::optional<std::thread> bg_comm_;
};
}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_
