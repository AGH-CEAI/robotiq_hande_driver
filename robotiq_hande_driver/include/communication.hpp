#include <stdint.h>
#include <stddef.h>
#include <vector>
#include <modbus/modbus.h>

/**
 * @brief This class contains low level gripper commands and status
 */

namespace hande_driver
{
constexpr auto registerWordLength = 3;

class Communication{
public:
    Communication();

    ~Communication();

    /**
     * @brief Connect to gripper using modbus rtu and virtual socket
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void connect();

    /**
     * @brief Disconnect from gripper using modbus rtu and virtual socket
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void disconnect();

    /**
     * @brief Read and write input.output registers at once
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void read_write_registers();

    uint16_t input_registers[registerWordLength];
    uint16_t output_registers[registerWordLength];

private:
    modbus_t *mb;

};
}   // namespace hande_driver
