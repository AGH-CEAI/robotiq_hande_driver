#include "robotiq_hande_driver/communication.hpp"

#include <cstring>
#include <iostream>

namespace robotiq_hande_driver {

Communication::Communication()
    : cfg_{},
      mb_{nullptr},
      input_bytes_{},  // TODO setup proper initial values
      output_bytes_{},
      th_comm_enabled_{false} {}

Communication::~Communication() {
    cleanup();
}

void Communication::initialize(const CommunicationConfig& cfg) {
    cfg_ = cfg;
}

void Communication::read_write_registers() {
    int result = 0;
    while(th_comm_enabled_) {
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

        std::this_thread::sleep_for(cfg_.th_sleep_rate);
    }
}

int Communication::configure() {
    if(mb_ == nullptr) {
        mb_ = modbus_new_rtu(
            cfg_.tty_port.c_str(), cfg_.baudrate, cfg_.parity, cfg_.data_bits, cfg_.stop_bit);
        modbus_set_slave(mb_, cfg_.slave_id);
        modbus_set_debug(mb_, DEBUG_MODBUS);
    }
    // TODO: asynchronously connect to the modbus TCP (wait for a virtual serial port creation from
    // socat)
    auto result = connect();
    if(result == FAILURE_MODBUS) {
        std::cout << "[WARNING] [robotiq_hande_driver] Failed to establish Modbus connection.\n";
        return FAILURE_MODBUS;
    }

    if(!th_comm_enabled_) {
        th_comm_enabled_.store(true, std::memory_order_relaxed);
        th_comm_.emplace(&Communication::read_write_registers, this);
    }

    // TODO do we really need the result?
    // Can we switch to more roboust throw & catch?
    return result;
}

void Communication::cleanup() {
    th_comm_enabled_.store(false, std::memory_order_relaxed);
    if(th_comm_ && th_comm_->joinable()) {
        th_comm_->join();
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

InputBuffer Communication::get_input_bytes() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return input_bytes_;
}

void Communication::set_output_bytes(const OutputBuffer& vals) {
    std::lock_guard<std::mutex> lock(mtx_);
    output_bytes_ = vals;
}

}  // namespace robotiq_hande_driver
