#include "CarlikeNeedleController.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Actuator.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/XMLNode.h"

#include <algorithm>

#include "nonstd/container_ops.h"


/*------------------------------ Construction --------------------------------*/

CarlikeNeedleController::
CarlikeNeedleController(Robot* const _r, const double _gain, const double _max)
  : SimpleController(_r, _gain, _max)
{ }


CarlikeNeedleController::
CarlikeNeedleController(Robot* const _r, XMLNode& _node)
  : SimpleController(_r, _node)
{ }


CarlikeNeedleController::
CarlikeNeedleController(Robot* const _r, const CarlikeNeedleController& _c)
  : SimpleController(_r, _c)
{ }


std::unique_ptr<ControllerMethod>
CarlikeNeedleController::
Clone(Robot* const _r) const {
  return std::unique_ptr<CarlikeNeedleController>(
      new CarlikeNeedleController(_r, *this));
}


CarlikeNeedleController::
~CarlikeNeedleController() = default;

/*----------------------- ControllerMethod Overrides -------------------------*/

Control
CarlikeNeedleController::
GetRandomControl(const Cfg& _current, const double _dt) const noexcept {
  // Make sure we are using continuous controls.
  if(m_controls)
    throw RunTimeException(WHERE) << "This object should use continuous controls."
                                  << " It has not been designed with discrete "
                                  << "control sets in mind.";

  // First choose a random actuator (there should be two, one for the tip
  // actuation and one for insertion).
  const auto& actuators = m_robot->GetActuators();
  const size_t actuatorIndex = LRand() % actuators.size();

  auto iter = actuators.begin();
  std::advance(iter, actuatorIndex);

  // Get a random control from this actuator and adjust it to account for the
  // needle tip direction at the current cfg.
  Control c = iter->second->GetRandomControl();
  AdjustControl(_current, c);

  return c;
}

/*----------------------------- Control Selection ----------------------------*/

std::vector<double>
CarlikeNeedleController::
ComputeDesiredForce(const Cfg& _current, const Cfg& _target, const double) {
  // Compute the shortest c-space direction from _current to _target.
  std::vector<double> delta = _current.DirectionInLocalFrame(_target);

  // Determine the gain coefficient for delta.
  const double magnitude = nonstd::magnitude<double>(delta),
               coefficient = (magnitude * m_gain > m_max) ? m_max / magnitude
                                                          : m_gain;

  // Compute the desired force from delta and coefficient.
  std::for_each(delta.begin(), delta.end(),
      [coefficient](double& _val) {_val *= coefficient;});

  return delta;
}


Control
CarlikeNeedleController::
ComputeNearestContinuousControl(const Cfg& _current,
    std::vector<double>&& _force) {
  Control best;
  double bestDot = -1;

  // Find the unit vector of the desired force in the robot's local coordinates.
  auto desiredDirection = nonstd::unit(_force);

  // Check each actuator and find the nearest control.
  for(const auto& actuatorPair : m_robot->GetActuators()) {
    const auto& actuator = actuatorPair.second.get();

    // Compute the nearest
    const auto signal = actuator->ComputeNearestSignal(_force);
    Control c(actuator, signal);
    AdjustControl(_current, c);
    const auto unitForce = nonstd::unit(actuator->ComputeOutput(signal));

    const double dot = nonstd::dot<double>(desiredDirection, unitForce);
    if(dot > bestDot) {
      best = c;
      bestDot = dot;
    }
  }

  // Debug.
  if(m_debug) {
    std::cout << "Computing best continuous control..."
              << "\n\tdesired force (local): " << _force
              << "\n\tbest control:          " << best
              << std::endl;
    if(best.actuator)
      std::cout << "\tbest force (local):    "
                << best.actuator->ComputeOutput(best.signal) << std::endl;
    std::cout << "\tnearest control has directional similarity " << bestDot
              << std::endl;
  }

  // Assert that the selected control is sensible.
  if(bestDot == -1)
    throw RunTimeException(WHERE) << "Best control is directly counter-"
                                  << "productive. This is probably an error in "
                                  << "the control set definition.";

  return best;
}

/*--------------------------------- Helpers ----------------------------------*/

void
CarlikeNeedleController::
AdjustControl(const Cfg& _cfg, Control& _c) const noexcept {
  // If this is the tip actuator, we can just execute the control.
  const size_t jointIndex = _cfg.PosDOF() + _cfg.OriDOF();
  const bool isTip = _c.actuator->ControlMask()[jointIndex];
  if(isTip)
    return;

  // Otherwise, we need to set the orientation velocity to match the joint angle
  // so that the insertion follows the tip direction.
  const size_t orientationIndex = _cfg.PosDOF();
  const double jointAngle = _cfg[jointIndex];

  // Arg, there is some kind of problem with converting our joints to bullet
  // representation. Need to flip the velocity here until we figure it out.
  _c.signal[orientationIndex] = -jointAngle;
}

/*----------------------------------------------------------------------------*/
