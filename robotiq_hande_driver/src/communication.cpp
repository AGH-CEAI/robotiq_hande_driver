#include <stdint.h>

#include "communication.hpp"


namespace hande_driver
{

constexpr auto deviceName = "/tmp/ttyUR";
constexpr auto baudrate = 115200;
constexpr auto parity = 'N';
constexpr auto dataBits = 8;
constexpr auto stopBit = 1;
constexpr auto debugModbus = false;
constexpr uint8_t slaveID = 0x09;

constexpr uint16_t gripperOutputFirstReg = 0x07D0;
constexpr uint16_t gripperInputFirstReg = 0x03E8;

Communication::Communication(){
    mb = modbus_new_rtu(deviceName, baudrate, parity, dataBits, stopBit);
    modbus_set_slave(mb, slaveID);
    modbus_set_debug(mb, debugModbus);
    connect();
}

Communication::~Communication(){
    disconnect();
    modbus_free(mb);
}

void Communication::connect(){
    modbus_connect(mb);
}

void Communication::disconnect(){
    modbus_close(mb);
}

void Communication::read_write_registers(){
    modbus_write_and_read_registers(mb,
                                    gripperOutputFirstReg,
                                    registerWordLength,
                                    output_registers,
                                    gripperInputFirstReg,
                                    registerWordLength,
                                    input_registers);
}
}   // namespace hande_driver
