#ifndef ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_
#define ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_

#include <atomic>
#include <cstring>
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
static constexpr auto INPUT_REGISTER_WORD_LENGTH = static_cast<uint>(InputBytes::BYTES_MAX) / 2;

/**
 * @brief This class contains low level gripper commands and status
 */
class Communication {
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
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void initialize(
        const std::string& tty_port,
        int baudrate,
        char parity,
        int data_bits,
        int stop_bit,
        int slave_id) {
        tty_port_ = tty_port.c_str();
        baudrate_ = baudrate;
        parity_ = parity;
        data_bits_ = data_bits;
        stop_bit_ = stop_bit;
        slave_id_ = slave_id;
    };

    /**
     * @brief Reads and writes input/output registers at once.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
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
         * @return int FAILURE_MODBUS on failure, nonnegative value for success
         * @note The status should be checked to verify successful execution. An exception is thrown
         * if communication issues occur.
         */
        // int result;

        while(running) {
            // sleep for 100ms
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            for(size_t i = 0; i < static_cast<size_t>(OutputBytes::BYTES_MAX); ++i) {
                output_bytes_modbus_[i] = output_bytes_[i].load(std::memory_order_relaxed);
            }

            modbus_write_and_read_registers(
                mb_,
                GRIPPER_INPUT_FIRST_REG,
                OUTPUT_REGISTER_WORD_LENGTH,
                reinterpret_cast<uint16_t*>(output_bytes_modbus_),
                GRIPPER_OUTPUT_FIRST_REG,
                INPUT_REGISTER_WORD_LENGTH,
                reinterpret_cast<uint16_t*>(input_bytes_modbus_));

            for(size_t i = 0; i < static_cast<size_t>(InputBytes::BYTES_MAX); ++i) {
                input_bytes_[i].store(input_bytes_modbus_[i], std::memory_order_relaxed);
            }
        }

        // return result;
    };

    /**
     * @brief Initializes communication layer.
     *
     * @param none
     * @return int Connection status code.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    int configure() {
        int result;

        mb_ = modbus_new_rtu(tty_port_, baudrate_, parity_, data_bits_, stop_bit_);
        modbus_set_slave(mb_, slave_id_);
        modbus_set_debug(mb_, DEBUG_MODBUS);
        result = connect();

        my_thread.emplace(&Communication::read_write_registers, this);

        return result;
    };

    /**
     * @brief Deinitializes communication layer.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void cleanup() {
        if(my_thread && my_thread->joinable()) {
            my_thread->join();
        }

        disconnect();
        modbus_free(mb_);
    };

    /**
     * @brief Connects to the gripper using Modbus RTU and a virtual socket.
     *
     * @param none
     * @return connection status code.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    int connect();

    /**
     * @brief Disconnects from the gripper using Modbus RTU and a virtual socket.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void disconnect() {
        modbus_close(mb_);
    };

    /**
     * @brief Sets output bytes to zeros.
     *
     * @param none
     * @return None.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void clear_output_bytes() {
        // memset(output_bytes_, 0, sizeof(output_bytes_));
        for(size_t i = 0; i < static_cast<size_t>(OutputBytes::BYTES_MAX); ++i) {
            output_bytes_[i].store(0, std::memory_order_relaxed);
        }
    };

    /**
     * @brief Retrieves the input byte value at the specified index.
     *
     * @param index The InputBytes byte index.
     * @return Requested byte value.
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    uint8_t get_input_byte(InputBytes index) {
        return input_bytes_[static_cast<uint>(index)].load(std::memory_order_relaxed);
    };

    /**
     * @brief Sets the output byte value at the specified index.
     *
     * @param index OutputBytes byte index
     * @param value The value to be set.
     * @return none
     * @note The status should be checked to verify successful execution. An exception is thrown if
     * communication issues occur.
     */
    void set_output_byte(OutputBytes index, uint8_t value) {
        output_bytes_[static_cast<uint>(index)].store(value, std::memory_order_relaxed);
    };

    /**
     * @brief Sets the n-th bit of the action request byte.
     *
     * @param position_bit n-th bit in byte
     * @param value The boolean value to set for the bit.
     * @return None.
     */
    void write_action_bit(uint8_t position_bit, bool value) {
        output_bytes_[static_cast<uint>(OutputBytes::ACTION_REQUEST)].store(
            bit_set_to(
                output_bytes_[static_cast<uint>(OutputBytes::ACTION_REQUEST)].load(
                    std::memory_order_relaxed),
                position_bit,
                value),
            std::memory_order_relaxed);
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

    modbus_t* mb_;

    uint8_t input_bytes_modbus_[static_cast<size_t>(InputBytes::BYTES_MAX)];
    uint8_t output_bytes_modbus_[static_cast<size_t>(OutputBytes::BYTES_MAX)];

    std::atomic<uint8_t> input_bytes_[static_cast<size_t>(InputBytes::BYTES_MAX)];
    std::atomic<uint8_t> output_bytes_[static_cast<size_t>(OutputBytes::BYTES_MAX)];

    std::atomic<bool> running{true};
    std::optional<std::thread> my_thread;
};
}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_
