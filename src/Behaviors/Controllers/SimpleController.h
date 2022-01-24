#ifndef PMPL_SIMPLE_CONTROLLER_H_
#define PMPL_SIMPLE_CONTROLLER_H_

#include "ControllerMethod.h"

#include <limits>


////////////////////////////////////////////////////////////////////////////////
/// Chooses the control that drives most directly towards the target.
////////////////////////////////////////////////////////////////////////////////
class SimpleController : public ControllerMethod {

  public:

    ///@name Construction
    ///@{

    /// Construct a simple controller.
    /// @param _r The robot to control.
    /// @param _gain The direct error gain.
    /// @param _max  The maximum force to request.
    SimpleController(Robot* const _r, const double _gain,
        const double _max = std::numeric_limits<double>::infinity());

    /// Construct a simple controller from an XML node.
    /// @param _r The robot to control.
    /// @param _node The XML node to parse.
    SimpleController(Robot* const _r, XMLNode& _node);

    /// Copy a controller for another robot.
    /// @param _r The destination robot.
    /// @param _c The controller to copy.
    SimpleController(Robot* const _r, const SimpleController& _c);

    virtual std::unique_ptr<ControllerMethod> Clone(Robot* const _r) const
        override;

    virtual ~SimpleController();

    ///@}

  protected:

    ///@name Control Selection Overrides
    ///@{

    virtual std::vector<double> ComputeDesiredForce(const Cfg& _current,
        const Cfg& _target, const double _dt) override;

    ///@}
    ///@name Internal State
    ///@{

    double m_gain; ///< The proportional error gain.
    double m_max;  ///< The maximum force to exert.

    ///@}

};

#endif
