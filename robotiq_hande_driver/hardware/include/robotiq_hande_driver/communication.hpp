#ifndef ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_
#define ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_

#include <array>
#include <exception>
#include <string>

#include <modbus/modbus.h>

namespace robotiq_hande_driver {

static constexpr uint16_t SERIAL_OUTPUT_FIRST_REG = 0x03E8;
static constexpr uint16_t SERIAL_INPUT_FIRST_REG = 0x07D0;

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
 * @brief Struct to hold all modbus communication class config
 * @param tty_port Modbus virtual port.
 * @param baudrate Modbus serial baudrate.
 * @param parity Modbus serial parity.
 * @param data_bits Modbus serial data bits.
 * @param stop_bit Modbus serial stopbit.
 * @param slave_id Modbus slave id.
 */
struct CommunicationConfig {
    std::string tty_port;
    int baudrate;
    char parity;
    int data_bits;
    int stop_bit;
    int slave_id;
};

/**
 * @brief This class wrappers the low level serial modubus communication with the gripper
 */
class Communication {
   public:
    Communication();

    ~Communication();

    /**
     * @brief Initializes driver parameters.
     *
     * @param cfg Communication config
     */
    void initialize(const CommunicationConfig& cfg);

    /**
     * @brief Reads all of the input bytes values.
     *
     * @param regs Place to read the input bytes.
     */
    void read(InputBuffer& regs) const;

    /**
     * @brief Writes all of the output bytes values.
     *
     * @param regs The values to be set.
     */
    void write(OutputBuffer& regs) const;

    /**
     * @brief Initializes communication layer.
     */
    void configure();

    /**
     * @brief Deinitializes communication layer.
     */
    void cleanup();

    /**
     * @brief Connects to the gripper using Modbus RTU and a virtual socket.
     */
    void connect();

    /**
     * @brief Disconnects from the gripper using Modbus RTU and a virtual socket.
     */
    void disconnect();

   private:
    CommunicationConfig cfg_;
    modbus_t* mb_;

    static constexpr auto DEBUG_MODBUS = false;
    static constexpr auto FAILURE_MODBUS = -1;
};
}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_
