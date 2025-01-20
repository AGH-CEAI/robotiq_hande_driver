#include <stdint.h>

#include "protocol_logic.hpp"

/**
 * @brief This class contains high level gripper commands and status
 */

namespace hande_driver
{
struct {
    bool isReset;
    bool isReady;
    bool isMoving;
    bool isStopped;
    bool isOpened;
    bool isClosed;
    bool objectDetected;
} Status;

struct {             // Structure declaration
    int myNum;         // Member (int variable)
    string myString;   // Member (string variable)
} FaultStatus;

class ApplicationLayer{
    public:
    ApplicationLayer();

    ~ApplicationLayer();

    /**
     * @brief Stops movement of the gripper
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void stop();

    /**
     * @brief Resets the gripper: deactivate and activate again
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void reset();

    /**
     *  Emergency auto-release, gripper fingers are slowly opened, reactivation necessary
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void auto_release();

    /**
     * @brief Activates the gripper, after that it can be used 
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void activate();

    /**
     * @brief Opens the gripper
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void open();

    /**
     * @brief Closes the gripper
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void close();

    /**
     * @brief Updates gripper status
     * 
     * @param none
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void update_status()

    /**
     * @brief Returns the gripper status
     * 
     * @param none
     * @return Gripper status
     * @note see status on success, exception thrown if communicatoin issues
     */
    Status status(); 

    /**
     * @brief Returns the gripper fault status
     * 
     * @param none
     * @return Fault Status
     * @note see status on success, exception thrown if communicatoin issues
     */
    FaultStatus fault_status(); 

    /**
     * @brief Returns the gripper requsted position
     * 
     * @param none
     * @return Gripper requested position 
     * @note see status on success, exception thrown if communicatoin issues
     */
    uint8_t requested_position();

    /**
     * @brief Returns the gripper position
     * 
     * @param none
     * @return Gripper position in 
     * @note see status on success, exception thrown if communicatoin issues
     */
    uint8_t position();

    /**
     * @brief Moves the gripper to requested position
     * 
     * @param position to which the gripper has to move
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void ApplicationLayer::set_position(uint8_t position)

    /**
     * @brief Returns the gripper current
     * 
     * @param none
     * @return Gripper current in mA (range: 0-2550mA) 
     * @note see status on success, exception thrown if communicatoin issues
     */
    uint16_t current();

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