#ifndef PMPL_ICREATE_INTERFACE_H
#define PMPL_ICREATE_INTERFACE_H

#include "ActuatorInterface.h"

#include "MPProblem/Robot/Control.h"

namespace PlayerCc
{
  class PlayerClient;
  class Position2dProxy;
}


////////////////////////////////////////////////////////////////////////////////
/// An interface for the iCreate robot which uses the player library to
/// implement the connection. The robot's on-board controller in this case is a
/// netbook riding on top of the create running the client-side player software.
////////////////////////////////////////////////////////////////////////////////
class ICreateInterface : public ActuatorInterface
{

  public:

    ///@name Construction
    ///@{

    /// Construct an iCreate interface with server-side queue management.
    /// @param _ip The hardware controller's IP address.
    /// @param _port The hardware controller's IP port.
    ICreateInterface(const std::string& _ip, const unsigned short _port = 6665);

    virtual ~ICreateInterface();

    ///@}
    ///@name ActuatorInterface overrides
    ///@{

    /// Send command to iCreate
    /// @param _command Motion command to send
    virtual void SendCommand(const MotionCommand& _command) override;

    virtual bool FullStop() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    // Player components
    PlayerCc::PlayerClient* m_client{nullptr};        ///< Player client object.
    PlayerCc::Position2dProxy* m_position2d{nullptr}; ///< Position control.

    ControlSet m_lastControls;   ///< Last sent controls.

    ///@}

};

#endif
