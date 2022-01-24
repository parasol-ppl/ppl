#include "ICreateController.h"

#include "ConfigurationSpace/Cfg.h"
#include "Utilities/MPUtils.h"
#include "Utilities/XMLNode.h"

#include <algorithm>


/*------------------------------ Construction --------------------------------*/

ICreateController::
ICreateController(Robot* const _r, const double _gain, const double _max)
  : ControllerMethod(_r) { }


ICreateController::
ICreateController(Robot* const _r, XMLNode& _node) : ControllerMethod(_r, _node) {
}


ICreateController::
ICreateController(Robot* const _r, const ICreateController& _c)
  : ControllerMethod(_r, _c)
{ }


std::unique_ptr<ControllerMethod>
ICreateController::
Clone(Robot* const _r) const {
  return std::unique_ptr<ICreateController>(new ICreateController(_r, *this));
}


ICreateController::
~ICreateController() = default;

/*----------------------------- Control Selection ----------------------------*/

std::vector<double>
ICreateController::
ComputeDesiredForce(const Cfg& _current, const Cfg& _target, const double _dt) {
  const double x = _target[0] - _current[0],
               y = _target[1] - _current[1],
               a = Normalize(_target[2] - _current[2]),
               translation = std::sqrt(x*x + y*y),
               preRotation = Normalize(std::atan2(y, x)/PI - _current[2]);

  static constexpr double threshold = 1e-2;

  // Ideally, we would like the robot to cover the entire translation or
  // rotation in one step. The computed distances will thus be divided by _dt to
  // give the desired velocity.

  // If we're still here, we are at the target. Line up with the end Cfg.
  if(translation <= threshold)
  {
    if(m_debug)
      std::cout << "End rotation" << std::endl;
    return {0, 0, a / _dt};
  }
  // Line up to move towards target.
  else if(std::abs(preRotation) > threshold)
  {
    if(m_debug)
      std::cout << "Pre rotation" << std::endl;
    return {0, 0, preRotation / _dt};
  }
  // If we're still here, we are lined up. Go to the target.
  else
  {
    if(m_debug)
      std::cout << "Translation" << std::endl;
    return {translation / _dt, 0, 0};
  }
}

/*----------------------------------------------------------------------------*/
