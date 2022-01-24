#include "PlanningAgent.h"

#include "MPProblem/Robot/HardwareInterfaces/RobotCommandQueue.h"
#include "MPProblem/Robot/HardwareInterfaces/StateEstimator.h"
#include "Simulator/BulletModel.h"
#include "Simulator/Simulation.h"
#include "Utilities/MetricUtils.h"

#include <chrono>
#include <iostream>

#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"

/*------------------------------ Construction --------------------------------*/

PlanningAgent::
PlanningAgent(Robot* const _r) : Agent(_r) { }

PlanningAgent::
PlanningAgent(Robot* const _r, const PlanningAgent& _a) : Agent(_r, _a)
{ }

PlanningAgent::
PlanningAgent(Robot* const _r, XMLNode& _node) : Agent(_r, _node) {
  // Currently there are no parameters. Parse XML options here.
}

std::unique_ptr<Agent>
PlanningAgent::
Clone(Robot* const _r) const {
  return std::unique_ptr<PlanningAgent>(new PlanningAgent(_r, *this));
}

PlanningAgent::
~PlanningAgent() = default;

/*----------------------------- Agent Interface ------------------------------*/

void
PlanningAgent::
Initialize() {
  // Guard against re-init.
  if(m_initialized)
    return;
  m_initialized = true;

  // Get problem info.
  auto problem = m_robot->GetMPProblem();
  const std::string& xmlFile = problem->GetXMLFilename();

  // Initialize the agent's planning library.
  m_library = std::unique_ptr<MPLibrary>(new MPLibrary(xmlFile));
  m_solution = std::unique_ptr<MPSolution>(new MPSolution(m_robot));

  // Initialize a clock to track this agent's total planning time. This is done
  // to ensure that the stat's clock map isn't adding elements across threads.
  const std::string clockName = "Planning::" + m_robot->GetLabel();

  if(Simulation::Get())
    Simulation::GetStatClass()->ClearClock(clockName);
  //TODO::Put other stat class accessors here or better yet - create a different stat holder
}


void
PlanningAgent::
Step(const double _dt) {
  Initialize();

  // If the agent is planning or localizing, skip this step.
  if(IsPlanning() or IsLocalizing())
    return;

  // If the agent localized before this step, update the simulated state and
  // replan if necessary.
  if(!IsLocalizing() and m_localizeCount == 0)
    UpdateSimulatedState();

  // If the simulation has passed a set number of timesteps, localize.
  ++m_localizeCount;
  if(m_localizeCount > m_localizePeriod) {
    Localize();
    m_localizeCount = 0;
  }

  // Wait for the previous controls to finish if they still have time remaining.
  if(ContinueLastControls())
    return;

  // If we have no task, select the next one.
  if(!GetTask() and !SelectTask()) {
    // If no incomplete tasks remain, we are done.
    if(m_debug)
      std::cout << "Completed all tasks, halting robot." << std::endl;
    PauseAgent(1);
    return;
  }

  // If we have no plan, generate a plan.
  if(!HasPlan()) {
    PauseAgent(1);
    GeneratePlan();
    return;
  }

  // Evaluate task progress. If task is still valid, continue execution.
  if(EvaluateTask())
    ExecuteTask(_dt);
  else
    PauseAgent(1);
}

void
PlanningAgent::
Uninitialize() {
  if(!m_initialized)
    return;
  m_initialized = false;

  // Wait for the planning thread to stop.
  StopPlanning();
  while(m_planning)
    std::this_thread::sleep_for(std::chrono::microseconds(1));

  SetTask(nullptr);
  m_library.reset();
  m_solution.reset();

  if(m_roadmapVisualID != size_t(-1) and Simulation::Get()) {
    Simulation::Get()->RemoveRoadmap(m_roadmapVisualID);
    m_roadmapVisualID = size_t(-1);
  }
}

void
PlanningAgent::
SetTask(std::shared_ptr<MPTask> const _task) {
  ClearPlan();
  Agent::SetTask(_task);
}

/*------------------------------- Planning -----------------------------------*/

bool
PlanningAgent::
HasPlan() const {
  // We have a plan if the solution has cfgs.
  return m_solution->GetPath()->Size();
}

void
PlanningAgent::
ClearPlan() {
  if(HasPlan())
    //++m_planVersion;

  // Remove the path visual if needed.
  if(m_pathVisualID != size_t(-1) and Simulation::Get()) {
    Simulation::Get()->RemovePath(m_pathVisualID);
    m_pathVisualID = size_t(-1);
  }

  // Clear the path.
  if(m_solution)
    m_solution->GetPath()->Clear();
}

bool
PlanningAgent::
IsPlanning() const {
  return m_planning;
}

size_t
PlanningAgent::
GetPlanVersion() const {
  return m_planVersion;
}

MPSolution*
PlanningAgent::
GetMPSolution() {
  return m_solution.get();
}

/*---------------------------- Planning Helpers ------------------------------*/

void
PlanningAgent::
GeneratePlan() {
  m_planning = true;

  // Sets tasks' start constraint to the robot's current position
  auto start = std::unique_ptr<CSpaceConstraint>(
      new CSpaceConstraint(m_robot, m_robot->GetSimulationModel()->GetState()));

  GetTask()->SetStartConstraint(std::move(start));
  // Create a copy of the problem so that we can use the data objects in planning
  // without affecting the rest of the simulation.
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*m_robot->GetMPProblem()));

  // Start running the work function in another thread.
  m_thread = std::thread([this, problemCopy]() {
    // Track planning time by robot label.
    MethodTimer mt(Simulation::GetStatClass(),
        "Planning::" + this->m_robot->GetLabel());

    this->WorkFunction(problemCopy);

    // Retarget the library's problem back on the global copy so that later uses
    // of m_library which depend on the problem will not crash.
    this->m_library->SetMPProblem(this->m_robot->GetMPProblem());

    // Update plan version and flag.
    ++m_planVersion;
    m_planning = false;
  });

  // Detach thread so that it automatically joins on completion.
  m_thread.detach();
}

void
PlanningAgent::
WorkFunction(std::shared_ptr<MPProblem> _problem) {
#if 0
  // How to hard-code display of a goal boundary.
  const Boundary* tb = GetTask()->GetGoalConstraints().front()->GetBoundary();
  std::unique_ptr<WorkspaceBoundingSphere> b(new WorkspaceBoundingSphere(
      tb->GetCenter(), 4));
  Simulation::Get()->AddBoundary(b.get(), glutils::color::orange);
#endif

  // Initialize the solution.
  m_solution = std::unique_ptr<MPSolution>(new MPSolution(m_robot));

  // add DrawableRoadmap to be drawn
  if(m_debug) {
    m_roadmapVisualID = Simulation::Get()->AddRoadmap(
        m_solution->GetRoadmap(m_robot), glutils::color::green);
  }
  // Create a plan with PMPL.
  m_library->Solve(_problem.get(), GetTask().get(), m_solution.get());

  // If a path was created, show a visual.
  const auto& path = m_solution->GetPath()->Cfgs();
  if(!path.empty())
    m_pathVisualID = Simulation::Get()->AddPath(path, glutils::color::red);
}

void
PlanningAgent::
StopPlanning() {
  m_library->Halt();
}

/*------------------------------ Task Helpers --------------------------------*/

bool
PlanningAgent::
SelectTask() {
  auto tasks = m_robot->GetMPProblem()->GetTasks(m_robot);

  // Return false if there are no unfinished tasks.
  if(tasks.empty()) {
    SetTask(nullptr);
    return false;
  }

  // Otherwise, choose the next one.
  SetTask(tasks.front());
  GetTask()->GetStatus().start();
  return true;
}

bool
PlanningAgent::
EvaluateTask() {
  // This agent only plans and cannot complete tasks.
  return false;
}

void
PlanningAgent::
ExecuteTask(const double) {
  // This agent only plans, do nothing.
}

/*--------------------------- Localization Helpers ---------------------------*/

void
PlanningAgent::
UpdateSimulatedState() {
  auto hardware = m_robot->GetHardwareQueue();
  if(!hardware)
    return;

  auto sensor = hardware->GetSensor();
  if(!sensor)
    return;

  auto stateEstimator = m_robot->GetStateEstimator();
  stateEstimator->ApplyObservations(sensor);

  auto updatedPos = stateEstimator->GetEstimatedState();

  // Ensure that the updated position is valid
  if(!m_library->GetValidityChecker("pqp_solid")->IsValid(updatedPos, "cfg")) {
    if(m_debug)
      std::cout << "Ignoring Invalid Localization at: " << updatedPos
                << std::endl;
    return;
  }

  auto previousPos = m_robot->GetSimulationModel()->GetState();
  m_robot->GetSimulationModel()->SetState(updatedPos);

  auto dm = m_library->GetDistanceMetric("euclidean");
  const double distance = dm->Distance(previousPos, updatedPos);

  // If the error is too large, tell the agent to replan.
  if(distance > m_localizeErrorThreshold) {
    ClearPlan();
    if(m_debug)
      std::cout << "Large Error Found in Localization"
                << "\nError Distance: "
                << distance
                << "\nClearing Plan for Robot: "
                << m_robot->GetLabel()
                << std::endl;
  }

  if(m_debug)
    std::cout << "\nOld Simulated State: "
              << previousPos
              << "\nNew Simulated State: "
              << updatedPos
              << std::endl;
}

/*----------------------------------------------------------------------------*/
