#include <stdint.h>

#include "communication.hpp"


namespace hande_driver
{

constexpr auto deviceName = "/tmp/ttyUR";
constexpr auto baudrate = 115200;
constexpr auto timeoutMs = 500;

constexpr uint8_t slaveID = 0x09;

constexpr uint8_t readHoldingReg = 0x03;
constexpr uint16_t gripperOutputFirstReg = 0x07D0;

constexpr uint8_t readInputReg = 0x04;
constexpr uint16_t gripperInputFirstReg = 0x03E8;

Communication::Communication(){}

Communication::~Communication(){}

void Communication::connect(){}

void Communication::disconnect(){}

void Communication::send_command(const std::vector<uint8_t>& data, size_t resp_data_len){}

void Communication::get_status(size_t data_len){}

void Communication::arm_get_status_callback(uint16_t freq){}

}   // namespace hande_driver