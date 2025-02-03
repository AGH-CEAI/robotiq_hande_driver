#ifndef APPLICATION_HPP_
#define APPLICATION_HPP_

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
     * @brief Returns the gripper requested position
     *
     * @param none
     * @return Gripper requested position
     * @note see status on success, exception thrown if communicatoin issues
     */
    double RequestedPosition();

    /**
     * @brief Returns the gripper position
     *
     * @param none
     * @return Gripper position in [m]
     * @note see status on success, exception thrown if communicatoin issues
     */
    double Position();

    /**
     * @brief Moves the gripper to requested position
     *
     * @param position to which the gripper has to move, in [m]
     * @return none
     * @note see status on success, exception thrown if communicatoin issues
     */
    void SetPosition(double position);

    /**
     * @brief Returns the gripper current
     *
     * @param none
     * @return Gripper current in [A] (range: 0-2.55A)
     * @note see status on success, exception thrown if communicatoin issues
     */
    double Current();


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
    double requested_position_;

    /**
     * POsition of the gripper, in [m]
     */
    double position_;

    /**
     * Current flowin through the gripper, in [A]
     */
    double current_;

};
}   // namespace hande_driver
#endif // APPLICATION_HPP_
