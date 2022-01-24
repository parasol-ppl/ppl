#ifndef PMPL_MATLAB_NEEDLE_CONTROLLER_H_
#define PMPL_MATLAB_NEEDLE_CONTROLLER_H_

#include "ControllerMethod.h"

#include <limits>


////////////////////////////////////////////////////////////////////////////////
/// A controller for a steerable needle using a matlab mechanics model.
///
/// @warning Controls mean something entirely different for this kind of robot -
///          they are not generalized DOF forces/velocities.
///
/// @note This currently only works with the problem at envs/2D/HingedNeedle.
////////////////////////////////////////////////////////////////////////////////
class MatlabNeedleController : public ControllerMethod {

  public:

    ///@name Construction
    ///@{

    /// Construct a simple controller.
    /// @param _r The robot to control.
    MatlabNeedleController(Robot* const _r);

    /// Construct a simple controller from an XML node.
    /// @param _r The robot to control.
    /// @param _node The XML node to parse.
    MatlabNeedleController(Robot* const _r, XMLNode& _node);

    /// Copy a controller for another robot.
    /// @param _r The destination robot.
    /// @param _c The controller to copy.
    MatlabNeedleController(Robot* const _r, const MatlabNeedleController& _c);

    virtual std::unique_ptr<ControllerMethod> Clone(Robot* const _r) const
        override;

    virtual ~MatlabNeedleController();

    ///@}
    ///@name ControllerMethod Overrides
    ///@{

    /// This is different from other controllers - it takes a number of steps
    /// instead of a time span.
    /// @param _current The current configuration.
    /// @param _target The target configuration.
    /// @param _steps The number of steps to take.
    /// @return The computed control.
    virtual Control operator()(const Cfg& _current, const Cfg& _target,
        const double _steps) override;

    /// This is different from other controllers - it takes a number of steps
    /// instead of a time span.
    /// @param _current The current configuration.
    /// @param _steps The number of steps to take.
    /// @return The computed control.
    virtual Control GetRandomControl(const Cfg& _current, const double _steps)
        const noexcept override;

    ///@}

  protected:

    ///@name Control Selection Overrides
    ///@{

    /// Not used within this class.
    virtual std::vector<double> ComputeDesiredForce(const Cfg& _current,
        const Cfg& _target, const double _steps) override;

    ///@}

};

#endif
