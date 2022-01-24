#ifndef PMPL_ACTUATOR_INTERFACE_H
#define PMPL_ACTUATOR_INTERFACE_H

#include "HardwareInterface.h"

class MotionCommand;


////////////////////////////////////////////////////////////////////////////////
/// An abstract interface for a physical robot's actuators.
////////////////////////////////////////////////////////////////////////////////
class ActuatorInterface : public HardwareInterface {

  public:

    ///@name Local Types
    ///@{

    using HardwareInterface::HardwareType;

    ///@}
    ///@name Construction
    ///@{

    using HardwareInterface::HardwareInterface;

    virtual ~ActuatorInterface();

    ///@}
    ///@name Hardware Interface
    ///@{

    virtual HardwareType GetHardwareType() const noexcept override;

    ///@}
    ///@name Actuator Interface
    ///@{

    /// Send a motion command to the hardware actuator.
    /// @param _c The command to send.
    virtual void SendCommand(const MotionCommand& _c) = 0;

    /// Order the hardware actuator to reach full stop as fast as possible.
    /// @return True if the hardware acknowledged the command.
    virtual bool FullStop() = 0;

    ///@}

};

#endif
