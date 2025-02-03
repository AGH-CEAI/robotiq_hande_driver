#include <stdint.h>

#include "protocol_logic.hpp"

/**
 * @brief This class contains high level gripper commands and status
 */

namespace hande_driver
{

class ApplicationLayer{
public:

    struct Status{
        bool is_reset;
        bool is_ready;
        bool is_moving;
        bool is_stopped;
        bool is_opened;
        bool is_closed;
        bool object_detected;
    };

    struct FaultStatus{
        bool is_error;
    };

    ApplicationLayer();

    ~ApplicationLayer();

    /**
     * @brief Stops movement of the gripper
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void Stop();

    /**
     * @brief Resets the gripper: deactivate and activate again
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void Reset();

    /**
     *  Emergency auto-release, gripper fingers are slowly opened, reactivation necessary
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void AutoRelease();

    /**
     * @brief Activates the gripper, after that it can be used 
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void Activate();

    /**
     * @brief Opens the gripper
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void Open();

    /**
     * @brief Closes the gripper
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void Close();

    /**
     * @brief Updates gripper status
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void UpdateStatus();

    /**
     * @brief Returns the gripper status
     * 
     * @param none
     * @return Gripper status
     * @note see status on success, exception thrown if communicatoin issues
     */
    Status GetStatus(); 

    /**
     * @brief Returns the gripper fault status
     * 
     * @param none
     * @return Fault Status
     * @note see status on success, exception thrown if communicatoin issues
     */
    FaultStatus GetFaultStatus(); 

    /**
     * @brief Returns the gripper requsted position
     * 
     * @param none
     * @return Gripper requested position 
     * @note see status on success, exception thrown if communicatoin issues
     */
    uint8_t RequestedPosition();

    /**
     * @brief Returns the gripper position
     * 
     * @param none
     * @return Gripper position in 
     * @note see status on success, exception thrown if communicatoin issues
     */
    uint8_t Position();

    /**
     * @brief Moves the gripper to requested position
     * 
     * @param position to which the gripper has to move
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void SetPosition(uint8_t position);

    /**
     * @brief Returns the gripper current
     * 
     * @param none
     * @return Gripper current in mA (range: 0-2550mA) 
     * @note see status on success, exception thrown if communicatoin issues
     */
    uint16_t Current();


    void Read();

    void Write();

private:
    /**
     * Protocol logic, used for middle level abstraction
     */
    ProtocolLogic protocol_logic_;

    /**
     * Struct with status bits
     */
    Status status_;

    /**
     * Struct with fault status bits
     */
    FaultStatus fault_status_;

    /**
     * Requested position of the gripper, in [m]
     */
    uint8_t requested_position_;

    /**
     * POsition of the gripper, in [m]
     */
    uint8_t position_;
    
    /**
     * Current flowin through the gripper, in [mA]
     */
    uint16_t current_;
    
};
}   // namespace hande_driver