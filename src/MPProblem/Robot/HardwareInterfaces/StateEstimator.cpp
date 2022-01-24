#include "StateEstimator.h"

#include "AverageEstimator.h"
#include "MPProblem/Robot/Robot.h"
#include "Simulator/MicroSimulator.h"
#include "Utilities/XMLNode.h"
#include "Utilities/PMPLExceptions.h"


/*------------------------------- Construction -------------------------------*/

StateEstimator::
StateEstimator(Robot* const _robot)
  : m_robot(_robot), m_estimatedState(m_robot)
{ }


std::unique_ptr<StateEstimator>
StateEstimator::
Factory(Robot* const _robot, XMLNode& _node) {
  std::string type = _node.Read("type", true, "",
      "The StateEstimator class name.");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  std::unique_ptr<StateEstimator> output;

  if(type == "average")
    output = std::unique_ptr<AverageEstimator>(new AverageEstimator(_robot));
  else
    throw ParseException(_node.Where()) << "Unknown StateEstimator type '"
                                        << type << "'.";

  output->m_debug = _node.Read("debug", false, false, "Show debug messages.");

  return output;
}


StateEstimator::
~StateEstimator() = default;

/*-------------------------------- Interface ---------------------------------*/

void
StateEstimator::
SetState(const Cfg& _cfg, const std::vector<double>& _uncertainty) {
  // Require that the DOF and uncertainty size match.
  if(_cfg.DOF() != _uncertainty.size())
    throw RunTimeException(WHERE) << "Cannot set a state of size "
                                  << _cfg.DOF()
                                  << " with uncertainty of non-equal size "
                                  << _uncertainty.size() << "."
                                  << std::endl;

  m_estimatedState = _cfg;
  m_uncertainty = _uncertainty;
}


void
StateEstimator::
ApplyControls(const ControlSet& _controls, const double _dt) {
  // Use the robot's dynamics model to estimate the effect of the controls.
  m_estimatedState = m_robot->GetMicroSimulator()->Test(m_estimatedState,
      _controls, _dt);

  /// @todo Update uncertainty based on actuator's process noise. Ideally the
  ///       actuators should provide a function for estimating the uncertainty
  ///       in each DOF based on _controls and _dt.
}


const Cfg&
StateEstimator::
GetEstimatedState() const {
  return m_estimatedState;
}


const std::vector<double>&
StateEstimator::
GetUncertainty() const {
  return m_uncertainty;
}

/*----------------------------------------------------------------------------*/
