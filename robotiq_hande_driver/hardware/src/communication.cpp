#include "robotiq_hande_driver/communication.hpp"

#include <cstdio>

namespace robotiq_hande_driver {

Communication::Communication()
    : input_bytes_modbus_{}, output_bytes_modbus_{}, input_bytes_{}, output_bytes_{} {}

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
    tty_port_ = tty_port.c_str();
    baudrate_ = baudrate;
    parity_ = parity;
    data_bits_ = data_bits;
    stop_bit_ = stop_bit;
    slave_id_ = slave_id;
}

void Communication::read_write_registers() {
    while(bg_comm_enabled_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for(size_t i = 0; i < NUM_OF_OUTPUT_BYTES; ++i) {
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

        for(size_t i = 0; i < NUM_OF_INPUT_BYTES; ++i) {
            input_bytes_[i].store(input_bytes_modbus_[i], std::memory_order_relaxed);
        }
    }
}

int Communication::configure() {
    mb_ = modbus_new_rtu(tty_port_, baudrate_, parity_, data_bits_, stop_bit_);
    modbus_set_slave(mb_, slave_id_);
    modbus_set_debug(mb_, DEBUG_MODBUS);
    auto result = connect();

    bg_comm_.emplace(&Communication::read_write_registers, this);

    return result;
}

void Communication::cleanup() {
    bg_comm_enabled_.store(false, std::memory_order_relaxed);
    if(bg_comm_ && bg_comm_->joinable()) {
        bg_comm_->join();
    }

    disconnect();
    modbus_free(mb_);
}

int Communication::connect() {
    return modbus_connect(mb_);
}

void Communication::disconnect() {
    modbus_close(mb_);
}

void Communication::clear_output_bytes() {
    for(size_t i = 0; i < NUM_OF_OUTPUT_BYTES; ++i) {
        output_bytes_[i].store(0, std::memory_order_relaxed);
    }
    memset(output_bytes_modbus_, 0, sizeof(output_bytes_modbus_));
}

uint8_t Communication::get_input_byte(InputBytes index) {
    auto idx = static_cast<uint>(index);
    return input_bytes_[idx].load(std::memory_order_relaxed);
}

void Communication::set_output_byte(OutputBytes index, uint8_t value) {
    auto idx = static_cast<uint>(index);
    output_bytes_[idx].store(value, std::memory_order_relaxed);
}

void Communication::write_action_bit(uint8_t position_bit, bool value) {
    auto idx = static_cast<uint>(OutputBytes::ACTION_REQUEST;
    auto reg = output_bytes_[idx].load(std::memory_order_relaxed);
    auto result = bit_set_to(reg, position_bit, value);

    output_bytes_[idx].store(result, std::memory_order_relaxed);
}

uint Communication::bit_set_to(uint value, uint n, bool x) {
    uint reset_n_bit = ~(1u << n);
    uint set_n_bit = static_cast<uint>(x) << n;
    return (value & reset_n_bit) | set_n_bit;
}
}  // namespace robotiq_hande_driver
