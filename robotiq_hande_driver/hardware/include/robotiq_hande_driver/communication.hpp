#ifndef ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_
#define ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_

#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

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
 * @brief An auxialry struct to hold and interpret the registers data.
 */
struct OutputRegisters {
    // TODO add if for checking the reg range
    uint8_t get(OutputBytes reg) {
        std::lock_guard<std::mutex> lock(mtx_);
        return regs_[reg];
    }

    void set(OutputBytes reg, uint8_t val) {
        std::lock_guard<std::mutex> lock(mtx_);
        regs_[reg] = val;
    }

    uint8_t regs_[NUM_OF_OUTPUT_BYTES];

   private:
    mutable std::mutex mtx_;
};
struct InputRegisters {
    // TODO add if for checking the reg range
    uint8_t get(InputBytes reg) {
        std::lock_guard<std::mutex> lock(mtx_);
        return regs_[reg];
    }

    void set(InputBytes reg, uint8_t val) {
        std::lock_guard<std::mutex> lock(mtx_);
        regs_[reg] = val;
    }

    uint8_t regs_[NUM_OF_INPUT_BYTES];

   private:
    mutable std::mutex mtx_;
};

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
     * @brief Retrieves all of the input bytes values.
     *
     * @param none
     * @return Copy of the input bytes.
     */
    std::array<uint8_t, NUM_OF_INPUT_BYTES> get_input_bytes() const;

    /**
     * @brief Sets all of the output bytes values.
     *
     * @param vals The values to be set.
     * @return none
     */
    void set_output_bytes(const std::array<uint8_t, NUM_OF_OUTPUT_BYTES>& vals);

   private:
    std::string tty_port_;
    int baudrate_;
    char parity_;
    int data_bits_;
    int stop_bit_;
    int slave_id_;

    modbus_t* mb_;

    std::array<uint8_t, NUM_OF_INPUT_BYTES> input_bytes_;
    std::array<uint8_t, NUM_OF_OUTPUT_BYTES> output_bytes_;

    mutable std::mutex mtx_;
    std::atomic<bool> bg_comm_enabled_;
    std::optional<std::thread> bg_comm_;
};
}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__COMMUNICATION_HPP_
