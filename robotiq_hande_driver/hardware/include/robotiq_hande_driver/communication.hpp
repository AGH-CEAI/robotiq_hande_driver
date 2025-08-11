#ifndef ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_
#define ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <exception>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <modbus/modbus.h>

namespace robotiq_hande_driver {

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
using OutputBuffer = std::array<uint8_t, NUM_OF_OUTPUT_BYTES>;

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
using InputBuffer = std::array<uint8_t, NUM_OF_INPUT_BYTES>;

// Define a custom exception by inheriting from std::exception
class CommunicationError : public std::exception {
   private:
    std::string message_;

   public:
    CommunicationError(const std::string& msg) : message_(msg) {}

    const char* what() const noexcept override {
        return message_.c_str();
    }
};

/**
 * @brief Struct to hold all Modbus Communication class config
 * @param tty_port Modbus virtual port.
 * @param baudrate Modbus serial baudrate.
 * @param parity Modbus serial parity.
 * @param data_bits Modbus serial data bits.
 * @param stop_bit Modbus serial stopbit.
 * @param slave_id Modbus slave id.
 * @param th_sleep_rate The Modbus communication sleep rate.
 */
struct CommunicationConfig {
    std::string tty_port;
    int baudrate;
    char parity;
    int data_bits;
    int stop_bit;
    int slave_id;
    std::chrono::milliseconds th_sleep_rate;
};

/**
 * @brief This class contains low level gripper commands and status
 */
class Communication {
    static constexpr auto DEBUG_MODBUS = false;
    static constexpr auto FAILURE_MODBUS = -1;

   public:
    Communication();

    ~Communication();

    /**
     * @brief Initializes driver parameters.
     *
     * @param cfg Communication config
     * @return None.
     */
    void initialize(const CommunicationConfig& cfg);

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
     */
    void configure();

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
    void connect();

    /**
     * @brief Disconnects from the gripper using Modbus RTU and a virtual socket.
     *
     * @param none
     * @return None.
     */
    void disconnect();

    /**
     * @brief Retrieves all of the input bytes values.
     *
     * @param none
     * @return Copy of the input bytes.
     */
    InputBuffer get_input_bytes() const;

    /**
     * @brief Sets all of the output bytes values.
     *
     * @param vals The values to be set.
     * @return none
     */
    void set_output_bytes(const OutputBuffer& vals);

   private:
    CommunicationConfig cfg_;
    modbus_t* mb_;

    InputBuffer input_bytes_;
    OutputBuffer output_bytes_;

    mutable std::mutex mtx_;
    std::atomic<bool> th_comm_enabled_;
    std::optional<std::thread> th_comm_;
};
}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_
