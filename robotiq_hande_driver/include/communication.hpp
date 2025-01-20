#include <stdint.h>
#include <stddef.h>
#include <vector>
#include <serial.h>

/**
 * @brief This class contains low level gripper commands and status
 */

namespace hande_driver
{

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
     * @brief Send command to gripper
     * 
     * @param data  Data sent to gripper
     * @param resp_data_len Expected data length of received response
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void send_command(const std::vector<uint8_t>& data, size_t resp_data_len);

    /**
     * @brief Get status frame from gripper
     * 
     * @param data_len Data length to read
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void get_status(size_t data_len);

    /**
     * @brief Set callback to be triggered by a timer to update the gripper status
     * TODO: make a thread?
     * 
     * @param freq Refresh rate frequency, no higher than 200Hz including commands
     * @return none
     * @note see status on success, exception thrown in case of communication issues
     */
    void arm_get_status_callback(uint16_t freq);

protected:
    serial::Serial serial_;

private:
};
}   // namespace hande_driver