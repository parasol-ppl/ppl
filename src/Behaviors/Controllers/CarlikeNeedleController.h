#ifndef PMPL_CARLIKE_NEEDLE_CONTROLLER_H_
#define PMPL_CARLIKE_NEEDLE_CONTROLLER_H_

#include "SimpleController.h"

#include <limits>


////////////////////////////////////////////////////////////////////////////////
/// A simple controller for a steerable needle using car-like dynamics.
///
/// The difference from SimpleController is that it forces the needle to follow
/// the direction of the tip angle.
///
/// @note This currently only works with the problem at envs/2D/HingedNeedle.
////////////////////////////////////////////////////////////////////////////////
class CarlikeNeedleController : public SimpleController {

  public:

    ///@name Construction
    ///@{

    /// Construct a simple controller.
    /// @param _r The robot to control.
    /// @param _gain The direct error gain.
    /// @param _max  The maximum force to request.
    CarlikeNeedleController(Robot* const _r, const double _gain,
        const double _max = std::numeric_limits<double>::infinity());

    /// Construct a simple controller from an XML node.
    /// @param _r The robot to control.
    /// @param _node The XML node to parse.
    CarlikeNeedleController(Robot* const _r, XMLNode& _node);

    /// Copy a controller for another robot.
    /// @param _r The destination robot.
    /// @param _c The controller to copy.
    CarlikeNeedleController(Robot* const _r, const CarlikeNeedleController& _c);

    virtual std::unique_ptr<ControllerMethod> Clone(Robot* const _r) const
        override;

    virtual ~CarlikeNeedleController();

    ///@}
    ///@name ControllerMethod Overrides
    ///@{

    virtual Control GetRandomControl(const Cfg& _current, const double _dt) const
        noexcept override;

    ///@}

  protected:

    ///@name Control Selection Overrides
    ///@{

    virtual std::vector<double> ComputeDesiredForce(const Cfg& _current,
        const Cfg& _target, const double _dt) override;

    virtual Control ComputeNearestContinuousControl(const Cfg& _current,
        std::vector<double>&& _force) override;

    ///@}
    ///@name Helpers
    ///@{

    /// Adjust a control to account for the needle tip direction.
    /// @param _cfg The current configuration of the controller considered.
    /// @param _c The control to potentially adjust.
    void AdjustControl(const Cfg& _cfg, Control& _c) const noexcept;

    ///@}

};

#endif
