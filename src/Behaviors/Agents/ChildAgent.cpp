#include "ChildAgent.h"

#include "Coordinator.h"

#include "MPProblem/Robot/HardwareInterfaces/HardwareInterface.h"
#include "MPProblem/Robot/HardwareInterfaces/RobotCommandQueue.h"
#include "MPProblem/Robot/HardwareInterfaces/SensorInterface.h"

#include "Behaviors/Controllers/ControllerMethod.h"

#include "Simulator/Simulation.h"
/*---------------------------------- Construction ----------------------------------*/

ChildAgent::
ChildAgent(Robot* const _r) : PathFollowingAgent(_r) {

}

ChildAgent::
ChildAgent(Robot* const _r, XMLNode& _node) : PathFollowingAgent(_r,_node) {
  m_controlChannel = _node.Read("controlChannel",false,"","Channel to publish controls to.");
}

ChildAgent::
ChildAgent(Robot* const _r, const ChildAgent& _a) : PathFollowingAgent(_r,_a)
{ }

ChildAgent::
~ChildAgent() {
  Uninitialize();
}

std::unique_ptr<Agent>
ChildAgent::
Clone(Robot* const _r) const {
  return std::unique_ptr<ChildAgent>(new ChildAgent(_r, *this));
}

/*----------------------------- Simulation Interface -------------------------------*/

void
ChildAgent::
Initialize() {
  // Set up control publishing
  if(m_controlChannel.size() > 0) {
   /* if(!m_communicator->GetPublisher(m_controlChannel))
      throw RunTimeException(WHERE) << m_robot->GetLabel()
                                    << " control channel not connected."
                                    << std::endl;
*/
    //m_running = true;
    //auto publish = [this](){this->PublishControls();};
    //m_thread = std::thread(publish);
  }

  PathFollowingAgent::Initialize();
}

void
ChildAgent::
Uninitialize() {
  /*if(m_controlChannel.size() > 0)
    std::lock_gaurd<std::mutex> guard(m_lock);

    m_running = false;
    if(m_thread.joinable())
      m_thread.join();
  }*/
  PathFollowingAgent::Uninitialize();
}

void
ChildAgent::
Step(const double _dt) {
  if(m_debug and m_graphVisualID == (size_t(-1)) and m_solution.get() and Simulation::Get()) {
    m_graphVisualID = Simulation::Get()->AddRoadmap(m_solution->GetRoadmap(),
      glutils::color(0., 1., 0., 0.2));
  }
  if(m_debug and m_pathVisualID == (size_t(-1)) and !m_path.empty() and Simulation::Get()) {
    m_pathVisualID = Simulation::Get()->AddPath(m_path, glutils::color::red);
  }

  PathFollowingAgent::Step(_dt);
  if(!HasPlan()) {
    const Cfg current = m_robot->GetSimulationModel()->GetState();
    auto control = m_robot->GetController()->operator()(current,
      current, 1);
    ExecuteControls({control},1);
  }
}

/*------------------------------- Child Interface ---------------------------------*/

Coordinator*
ChildAgent::
GetCoordinator() {
  return m_coordinator;
}

void
ChildAgent::
SetCoordinator(Coordinator* const _coordinator) {
  m_coordinator = _coordinator;
}

/*---------------------------------- Accessors ------------------------------------*/

MPSolution*
ChildAgent::
GetMPSolution() {
  return m_solution.get();
}

/*-------------------------------- Task Helpers -----------------------------------*/

bool
ChildAgent::
SelectTask() {
  return true;
}

bool
ChildAgent::
EvaluateTask() {
  return PathFollowingAgent::EvaluateTask();
}

void
ChildAgent::
ExecuteTask(const double _dt) {
  PathFollowingAgent::ExecuteTask(_dt);
}

void
ChildAgent::
GeneratePlan() {}

void
ChildAgent::
ClearPlan() {
  PathFollowingAgent::ClearPlan();
}

/*------------------------------- Controller Helpers ---------------------------------*/

void
ChildAgent::
ExecuteControls(const ControlSet& _c, const size_t _steps) {
  this->Agent::ExecuteControls(_c, _steps);
  if(_c.size() > 1)
    throw RunTimeException(WHERE,
        "We are assuming that only one control will be passed in at a time.");
}

void
ChildAgent::
ExecuteControlsSimulation(const ControlSet& _c, const size_t _steps) {
  Agent::ExecuteControlsSimulation(_c,_steps);
}

void
ChildAgent::
ExecuteControlsHardware(const ControlSet& _c, const size_t _steps) {
  Agent::ExecuteControlsHardware(_c,_steps);
}

/*-----------------------------------------------------------------------------------*/
