#include "communication.hpp"

#include <cstdio>
#include <stdint.h>
#include <string.h>


namespace hande_driver  {

constexpr auto kDeviceName = "/tmp/ttyUR";
constexpr auto kBaudrate = 115200;
constexpr auto kParity = 'N';
constexpr auto kDataBits = 8;
constexpr auto kStopBit = 1;
constexpr auto kDebugModbus = true;
constexpr uint8_t kSlaveID = 0x09;

constexpr uint16_t kGripperOutputFirstReg = 0x07D0;
constexpr uint16_t kGripperInputFirstReg = 0x03E8;

Communication::Communication()
// :    input_registers_{0, 0, 0}
// ,    output_registers_{0, 0, 0}
:    input_bytes_{0, 0, 0, 0, 0, 0}
,    output_bytes_{0, 0, 0, 0, 0, 0}
{
    printf("Communication constructor\n");
    mb_ = modbus_new_rtu(kDeviceName, kBaudrate, kParity, kDataBits, kStopBit);
    modbus_set_slave(mb_, kSlaveID);
    modbus_set_debug(mb_, kDebugModbus);
    connect();
}

Communication::~Communication(){
    printf("Communication destructor\n");
    disconnect();
    modbus_free(mb_);
}

void Communication::connect(){
    modbus_connect(mb_);

    //TODO: send read/write and verify gripper is responding
}

void Communication::disconnect(){
    modbus_close(mb_);
}

void Communication::read_write_registers(){
    // TODO: add description
    modbus_write_and_read_registers(mb_,
                                    kGripperInputFirstReg,
                                    kRegisterWordLength,
                                    (uint16_t *)output_bytes_,
                                    kGripperOutputFirstReg,
                                    kRegisterWordLength,
                                    (uint16_t *)input_bytes_);
}

void Communication::clear_output_bytes(){
    memset(output_bytes_, 0, sizeof(output_bytes_));
}

uint8_t Communication::get_input_byte(InputBytes index){
    return input_bytes_[index];
}

void Communication::set_output_byte(OutputBytes index, uint8_t value){
    output_bytes_[index] = value;
}
}   // namespace hande_driver
