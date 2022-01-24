#include "MatlabNeedleController.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Actuator.h"
#include "MPProblem/Robot/Robot.h"
#include "Simulator/MatlabMicroSimulator.h"
#include "Utilities/XMLNode.h"

#include "nonstd/container_ops.h"

#include <algorithm>


/*------------------------------ Construction --------------------------------*/

MatlabNeedleController::
MatlabNeedleController(Robot* const _r)
  : ControllerMethod(_r)
{ }


MatlabNeedleController::
MatlabNeedleController(Robot* const _r, XMLNode& _node)
  : ControllerMethod(_r, _node)
{ }


MatlabNeedleController::
MatlabNeedleController(Robot* const _r, const MatlabNeedleController& _c)
  : ControllerMethod(_r, _c)
{ }


std::unique_ptr<ControllerMethod>
MatlabNeedleController::
Clone(Robot* const _r) const {
  return std::unique_ptr<MatlabNeedleController>(
      new MatlabNeedleController(_r, *this));
}


MatlabNeedleController::
~MatlabNeedleController() = default;

/*----------------------- ControllerMethod Overrides -------------------------*/

Control
MatlabNeedleController::
operator()(const Cfg& _current, const Cfg& _target, const double _steps) {
  // Make sure we are using continuous controls.
  if(m_controls)
    throw RunTimeException(WHERE) << "This object should use continuous controls."
                                  << " It has not been designed with discrete "
                                  << "control sets in mind.";

  // Get the space of allowed controls at _current.
  auto simulator = m_robot->GetMatlabMicroSimulator();
  const ControlSpace controlSpace = simulator->GetControlSpace(_current);

  // Determine the insertion depth that will be used. Initialize a control
  // signal with this depth.
  const double insertion = std::min<double>(.005, controlSpace.GetRange(2).max);

  std::vector<double> signal{0, 0, insertion};

#if 0
  // Determine whether the target lies above or beneath the needle tip in the Y
  // direction.
  const double yDiff = _target[1] - _current[1];

  // If the tip is below the target, pull on the second tendon to drive it
  // upward.
  const bool belowTarget = yDiff > 0;
  if(belowTarget)
  {
    signal[0] = controlSpace.GetRange(0).max * .7;
    signal[1] = -signal[0];
  }
  else
  {
    signal[1] = controlSpace.GetRange(1).max * .7;
    signal[0] = -signal[1];
  }
#else
  // Determine wheter the needle is aimed at the target. If not, try to
  // straighten it out.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  // Get the x and y difference to the target.
  const double xDiff = _target[0] - _current[0],
               yDiff = _target[1] - _current[1],
               angle = std::atan2(yDiff, xDiff);

  // Get the base angle and joint angle.
  const double baseAngle = _current[2] * PI,
               jointAngle = _current[3] * PI,
               totalJointAngle = baseAngle + jointAngle;

  // Get the angle of the previous extension.
  double prevAngle = totalJointAngle;
  if(_current.m_witnessCfg.get())
  {
    const Cfg& parent = *_current.m_witnessCfg;
    prevAngle = std::atan2(_current[1] - parent[1],
                           _current[0] - parent[0]);
  }

  // Try to get the base angle to 'angle' and the joint angle to 0.
  const double //maxJointAngle = 30. * PI / 180.,
               //angleDiff = std::abs(totalJointAngle - angle),
               //angleFrac = .9 * std::min(angleDiff, maxJointAngle) / maxJointAngle;
               angleFrac = 1;//.8 + .15 * DRand();
  if(baseAngle < angle)
  {
    const double desiredControl = controlSpace.GetRange(0).max * angleFrac;

    // The motor can't pull less than 100um.
    if(desiredControl >= 1e-6)
    {
      signal[0] = desiredControl;
      signal[1] = -desiredControl;
    }
  }
  else
  {
    const double desiredControl = controlSpace.GetRange(1).max * angleFrac;

    // The motor can't pull less than 100um.
    if(desiredControl >= 1e-6)
    {
      signal[1] = desiredControl;
      signal[0] = -desiredControl;
    }
  }

#endif


  return Control{nullptr, signal};
#pragma GCC diagnostic pop
}


Control
MatlabNeedleController::
GetRandomControl(const Cfg& _current, const double _steps) const noexcept {
  // Make sure we are using continuous controls.
  if(m_controls)
    throw RunTimeException(WHERE) << "This object should use continuous controls."
                                  << " It has not been designed with discrete "
                                  << "control sets in mind.";

  // Get the space of allowed controls at _current.
  auto simulator = m_robot->GetMatlabMicroSimulator();
  const ControlSpace controlSpace = simulator->GetControlSpace(_current);

  // Sample the control space.
  std::vector<double> signal = controlSpace.Sample();

  // Select either first (0) or second (1) tendon to pull. Relax the other one
  // by the same amount.
  const bool tendon = DRand() > .5;
  // Make sure we are sampling a large control.
  if(signal[tendon] < .8 * controlSpace.GetRange(tendon).max)
  {
    signal[tendon] = (.8 + .15 * DRand()) * controlSpace.GetRange(tendon).max;
  }

  // The motor can't pull less than 100um.
  if(signal[tendon] < 1e-6)
  {
    if(1e-6 > controlSpace.GetRange(tendon).max)
      signal[tendon] = 0.;
    else
      signal[tendon] = 1e-6;
  }
  signal[!tendon] = -signal[tendon];

  return Control{nullptr, signal};
}

/*----------------------------- Control Selection ----------------------------*/

std::vector<double>
MatlabNeedleController::
ComputeDesiredForce(const Cfg& _current, const Cfg& _target, const double) {
  throw RunTimeException(WHERE) << "Not used in this class.";
  return {};
}

/*----------------------------------------------------------------------------*/
