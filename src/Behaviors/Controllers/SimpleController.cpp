#include "SimpleController.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/XMLNode.h"

#include <algorithm>

#include "nonstd/container_ops.h"


/*------------------------------ Construction --------------------------------*/

SimpleController::
SimpleController(Robot* const _r, const double _gain, const double _max)
  : ControllerMethod(_r), m_gain(_gain), m_max(_max) { }


SimpleController::
SimpleController(Robot* const _r, XMLNode& _node)
  : ControllerMethod(_r, _node)
{
  m_gain = _node.Read("gain", true, 0.,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "The proportional gain");

  m_max = _node.Read("maxMagnitude", false,
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "The maximum force/velocity magnitude to request");
}


SimpleController::
SimpleController(Robot* const _r, const SimpleController& _c)
  : ControllerMethod(_r, _c),
    m_gain(_c.m_gain),
    m_max(_c.m_max)
{ }


std::unique_ptr<ControllerMethod>
SimpleController::
Clone(Robot* const _r) const {
  return std::unique_ptr<SimpleController>(new SimpleController(_r, *this));
}


SimpleController::
~SimpleController() = default;

/*----------------------------- Control Selection ----------------------------*/

std::vector<double>
SimpleController::
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

/*----------------------------------------------------------------------------*/
