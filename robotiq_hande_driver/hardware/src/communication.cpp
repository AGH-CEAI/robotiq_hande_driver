#include "robotiq_hande_driver/communication.hpp"

#include <cstring>
#include <iostream>

namespace robotiq_hande_driver {

Communication::Communication()
    : tty_port_{},
      baudrate_{},
      parity_{},
      data_bits_{},
      stop_bit_{},
      slave_id_{},
      mb_{nullptr},
      input_bytes_modbus_{},
      output_bytes_modbus_{},
      input_bytes_{},  // TODO setup proper initial values
      output_bytes_{},
      bg_comm_enabled_{false} {}

Communication::~Communication() {
    cleanup();
}

void Communication::initialize(
    const std::string& tty_port,
    int baudrate,
    char parity,
    int data_bits,
    int stop_bit,
    int slave_id) {
    tty_port_ = tty_port;
    baudrate_ = baudrate;
    parity_ = parity;
    data_bits_ = data_bits;
    stop_bit_ = stop_bit;
    slave_id_ = slave_id;
}

void Communication::read_write_registers() {
    int result = 0;
    while(bg_comm_enabled_) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            result = modbus_write_and_read_registers(
                mb_,
                GRIPPER_INPUT_FIRST_REG,
                OUTPUT_REGISTER_WORD_LENGTH,
                reinterpret_cast<uint16_t*>(output_bytes_.data()),
                GRIPPER_OUTPUT_FIRST_REG,
                INPUT_REGISTER_WORD_LENGTH,
                reinterpret_cast<uint16_t*>(input_bytes_.data()));
        }
        if(result == FAILURE_MODBUS)
            std::cout << "[WARNING] Failed to read & write Hand-e registers via modbus\n";

        // TODO parametrize the sleep time
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int Communication::configure() {
    if(mb_ == nullptr) {
        mb_ = modbus_new_rtu(tty_port_.c_str(), baudrate_, parity_, data_bits_, stop_bit_);
        modbus_set_slave(mb_, slave_id_);
        modbus_set_debug(mb_, DEBUG_MODBUS);
    }
    // TODO: asynchronously connect to the modbus TCP (wait for a virtual serial port creation from
    // socat)
    auto result = connect();
    if(result == FAILURE_MODBUS) {
        std::cout << "[WARNING] [robotiq_hande_driver] Failed to establish Modbus connection.\n";
        return FAILURE_MODBUS;
    }

    if(!bg_comm_enabled_) {
        bg_comm_enabled_.store(true, std::memory_order_relaxed);
        bg_comm_.emplace(&Communication::read_write_registers, this);
    }

    // TODO do we really need the result?
    // Can we switch to more roboust throw & catch?
    return result;
}

void Communication::cleanup() {
    bg_comm_enabled_.store(false, std::memory_order_relaxed);
    if(bg_comm_ && bg_comm_->joinable()) {
        bg_comm_->join();
    }

    if(mb_ == nullptr) return;
    disconnect();
    modbus_free(mb_);
    mb_ = nullptr;
}

int Communication::connect() {
    auto result = modbus_connect(mb_);
    return result;
}

void Communication::disconnect() {
    modbus_close(mb_);
}

std::array<uint8_t, NUM_OF_INPUT_BYTES> Communication::get_input_bytes() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return input_bytes_;
}

void Communication::set_output_bytes(const std::array<uint8_t, NUM_OF_OUTPUT_BYTES>& vals) {
    std::lock_guard<std::mutex> lock(mtx_);
    output_bytes_ = vals;
}

}  // namespace robotiq_hande_driver
