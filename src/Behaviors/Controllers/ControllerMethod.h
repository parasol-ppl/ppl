#ifndef PMPL_CONTROLLER_METHOD_H_
#define PMPL_CONTROLLER_METHOD_H_

#include <vector>

#include "MPProblem/Robot/Control.h"

class Cfg;
class Robot;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Controllers determine the best control needed to move a robot from one
/// configuration to another. They require the standard Cfg as the planning space
/// representation because that's what we can support in simulation.
///
/// @TODO Implement simultaneous use of controls from more than one actuator.
////////////////////////////////////////////////////////////////////////////////
class ControllerMethod {

  public:

    ///@name Construction
    ///@{

    /// Construct a controller for a given robot.
    /// @param _r The robot to control.
    ControllerMethod(Robot* const _r);

    /// Construct a controller for a given robot from an XML node.
    /// @param _r The robot to control.
    /// @param _node The XML node to read parameters from.
    ControllerMethod(Robot* const _r, XMLNode& _node);

    /// Copy a controller for another robot.
    /// @param _r The destination robot.
    /// @param _c The controller to copy.
    ControllerMethod(Robot* const _r, const ControllerMethod& _c);

    /// Create a dynamically-allocated controller from an XML node.
    /// @param _r The robot to control.
    /// @param _node The XML node to read parameters from.
    /// @return A controller of the type described by _node.
    static std::unique_ptr<ControllerMethod> Factory(Robot* const _r,
        XMLNode& _node);

    /// Create a copy of this controller for another robot. This is provided so
    /// that we can copy controllers without knowing their derived type.
    /// @param _r The robot receiving the cloned controller.
    /// @return The cloned controller.
    virtual std::unique_ptr<ControllerMethod> Clone(Robot* const _r) const = 0;

    virtual ~ControllerMethod();

    ///@}
    ///@name Disabled Functions
    ///@{
    /// Regular copy/move is not allowed because there can only be one
    /// controller per robot, and it must be constructed with the robot pointer
    /// present.

    ControllerMethod(const ControllerMethod&) = delete;
    ControllerMethod(ControllerMethod&&) = delete;

    ControllerMethod& operator=(const ControllerMethod&) = delete;
    ControllerMethod& operator=(ControllerMethod&&) = delete;

    ///@}
    ///@name Interface
    ///@{

    /// Find the best available control to steer a robot from a starting
    /// configuration to a target configuration.
    /// @param _current The current configuration.
    /// @param _target The target configuration.
    /// @param _dt The timestep length.
    /// @return The best available control for steering from _current to _target.
    virtual Control operator()(const Cfg& _current, const Cfg& _target,
        const double _dt);

    /// Find a random control.
    /// @param _current The current configuration.
    /// @param _dt The timestep length.
    /// @return A random control that can be executed from _current for _dt
    ///         steps.
    virtual Control GetRandomControl(const Cfg& _current, const double _dt) const
        noexcept;

    /// Get the discrete set of controls that this controller can use, if any.
    /// @return The discrete set of controls for this controller, or null if it
    ///         uses a continuous space(s) of controls.
    ControlSet* GetControlSet() noexcept;

    /// Set a discrete set of controls for this controller to use. This limits
    /// the controller to a subset of all possible controls accepted by a
    /// robot's actuators.
    /// @param _c The control set to use.
    void SetControlSet(ControlSet* const _c) noexcept;

    ///@}

  protected:

    ///@name Control Selection
    ///@{

    /// Compute the desired generalized force (or velocity) in the robot's local
    /// frame to move from the current position to the target.
    /// @param _current The current configuration.
    /// @param _target The target configuration.
    /// @param _dt The timestep length.
    /// @return The ideal generalized force.
    virtual std::vector<double> ComputeDesiredForce(const Cfg& _current,
        const Cfg& _target, const double _dt) = 0;

    /// Compute the control that produces the closest force to the ideal.
    /// @param _current The current configuration.
    /// @param _force The desired force in the robot's local frame.
    /// @return The control whose result is nearest to _force.
    virtual Control ComputeNearestControl(const Cfg& _current,
        std::vector<double>&& _force);

    /// Compute the continuous control that produces the closest force to the
    /// ideal.
    /// @param _current The current configuration.
    /// @param _force The desired force in the robot's local frame.
    /// @return The control whose result is nearest to _force.
    virtual Control ComputeNearestContinuousControl(const Cfg& _current,
        std::vector<double>&& _force);

    /// Compute the discrete control that produces the closest force to the
    /// ideal.
    /// @param _current The current configuration.
    /// @param _force The desired force in the robot's local frame.
    /// @return The control whose result is nearest to _force.
    virtual Control ComputeNearestDiscreteControl(const Cfg& _current,
        std::vector<double>&& _force);

    ///@}
    ///@name Internal State
    ///@{

    Robot* const m_robot;            ///< The owning robot object.
    ControlSet* m_controls{nullptr}; ///< The discrete controls, if any.
    bool m_debug{false};             ///< Show debug messages?

    ///@}

};

#endif
