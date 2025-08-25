#include "robotiq_hande_driver/communication.hpp"

namespace robotiq_hande_driver {

Communication::Communication() : cfg_{}, mb_{nullptr} {}

Communication::~Communication() {
    cleanup();
}

void Communication::initialize(const CommunicationConfig& cfg) {
    cfg_ = cfg;
}

void Communication::read(InputBuffer& regs) const {
    auto result = modbus_read_registers(
        mb_,
        SERIAL_INPUT_FIRST_REG,
        INPUT_REGISTER_WORD_LENGTH,
        reinterpret_cast<uint16_t*>(regs.data()));

    if(result == FAILURE_MODBUS)
        throw CommunicationError("Failed to read registers (Modbus failure)");
    if(result != INPUT_REGISTER_WORD_LENGTH)
        throw CommunicationError("Failed to read all requested registers");
}

void Communication::write(OutputBuffer& regs) const {
    auto result = modbus_write_registers(
        mb_,
        SERIAL_OUTPUT_FIRST_REG,
        OUTPUT_REGISTER_WORD_LENGTH,
        reinterpret_cast<uint16_t*>(regs.data()));

    if(result == FAILURE_MODBUS)
        throw CommunicationError("Failed to write registers (Modbus failure)");
    if(result != OUTPUT_REGISTER_WORD_LENGTH)
        throw CommunicationError("Failed to write all requested registers");
}

void Communication::configure() {
    if(mb_ == nullptr) {
        mb_ = modbus_new_rtu(
            cfg_.tty_port.c_str(), cfg_.baudrate, cfg_.parity, cfg_.data_bits, cfg_.stop_bit);
        modbus_set_slave(mb_, cfg_.slave_id);
        modbus_set_debug(mb_, DEBUG_MODBUS);
    }

    connect();
}

void Communication::cleanup() {
    if(mb_ == nullptr) return;

    disconnect();
    modbus_free(mb_);
    mb_ = nullptr;
}

void Communication::connect() {
    auto result = modbus_connect(mb_);
    if(result == FAILURE_MODBUS) throw CommunicationError("Failed to establish Modbus connection");
}

void Communication::disconnect() {
    modbus_close(mb_);
}

}  // namespace robotiq_hande_driver
