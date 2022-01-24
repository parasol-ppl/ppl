#include "MicroSimulator.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Robot.h"
#include "Simulator/BulletEngine.h"
#include "Simulator/BulletModel.h"
#include "Utilities/PMPLExceptions.h"


/*------------------------------ Construction --------------------------------*/

MicroSimulator::
MicroSimulator(Robot* const _robot) :
    m_robot(_robot),
    m_engine(new BulletEngine(m_robot->GetMPProblem())),
    m_model(m_engine->AddRobot(m_robot))
{
  if(m_robot->GetSimulationModel() == m_model)
    throw RunTimeException(WHERE) << "Microsimulator requires its own simulation "
                                  << "model of the robot.";
}


MicroSimulator::
~MicroSimulator() {
  delete m_engine;
}

/*-------------------------------- Interface ---------------------------------*/

Cfg
MicroSimulator::
Test(const Cfg& _start, const Control& _control, const double _dt) {
  // Sanity check to make sure we are testing on the right robot.
  if(_start.GetRobot() != m_robot)
    throw RunTimeException(WHERE) << "Can't test dynamics model on a "
                                  << "configuration for a different robot.";

  if(m_robot->GetSimulationModel() == m_model)
    throw RunTimeException(WHERE) << "Microsimulator requires its own simulation "
                                  << "model of the robot.";

  // Set up the internal model at _start.
  m_model->SetState(_start);

  // Set the force/velocity.
  m_model->Execute(_control);

  // Apply for _dt units of time.
  m_engine->Step(_dt);

  // Return the resulting state.
  return m_model->GetState();
}


Cfg
MicroSimulator::
Test(const Cfg& _start, const ControlSet& _controlSet, const double _dt) {
  // Sanity check to make sure we are testing on the right robot.
  if(_start.GetRobot() != m_robot)
    throw RunTimeException(WHERE) << "Can't test dynamics model on a "
                                  << "configuration for a different robot.";

  if(m_robot->GetSimulationModel() == m_model)
    throw RunTimeException(WHERE) << "Microsimulator requires its own simulation "
                                  << "model of the robot.";

  // Set up the internal model at _start.
  m_model->SetState(_start);

  // Set the force/velocity.
  m_model->Execute(_controlSet);

  // Apply for _dt units of time.
  m_engine->Step(_dt);

  // Return the resulting state.
  return m_model->GetState();
}

/*----------------------------------------------------------------------------*/
