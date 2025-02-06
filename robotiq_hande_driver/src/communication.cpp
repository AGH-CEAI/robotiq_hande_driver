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
    uint16_t activation_status[1] = {0x0000};
    int result;

    modbus_connect(mb_);

    result = modbus_read_registers(mb_, kGripperOutputFirstReg, 1, activation_status);

    if (result > 0) {
        printf("Connected successfully: %d\n", result);
    } else {
        printf("Couldn't connect: %d\n", result);
    }
}

void Communication::disconnect(){
    modbus_close(mb_);
}

void Communication::read_write_registers(){
    /**
     * @brief Read and write modbus registers at once
     *
     * @param modbus_t *ctx
     * @param int write_addr
     * @param int write_nb
     * @param uint16_t *src
     * @param int read_addr
     * @param int read_nb
     * @param uint16_t *dest
     * @return int >0 on success
     * @note see status on success, exception thrown in case of communication issues
     */
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

void Communication::write_action_bit(uint8_t position_bit, bool value){
    output_bytes_[OUTPUT_BYTES_ACTION_REQUEST] = bit_set_to(
        output_bytes_[OUTPUT_BYTES_ACTION_REQUEST], position_bit, value);
}

inline uint Communication::bit_set_to(uint number, uint n, bool x) {
    return (number & ~((uint)1 << n)) | ((uint)x << n);
}
}   // namespace hande_driver
