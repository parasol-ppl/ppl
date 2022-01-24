#include "Coordinator.h"

#include <limits>
#include <queue>

#include "nonstd/numerics.h"
#include "nonstd/timer.h"
#include "nonstd/io.h"

#include "Behaviors/Agents/StepFunctions/StepFunction.h"
#include "Behaviors/Controllers/ControllerMethod.h"

#include "MPProblem/Robot/Robot.h"

#include "Simulator/Simulation.h"
#include "Simulator/BulletModel.h"

#ifndef PPL_TEST_TRAITS_H_
  #include "Traits/CfgTraits.h"
#else 
  #include "Traits/TestTraits.h"
#endif

#include "TMPLibrary/Solution/TaskSolution.h"

#include "sandbox/gui/main_window.h"

/*------------------------------ Construction --------------------------------*/

Coordinator::
Coordinator(Robot* const _r) : Agent(_r) {
}

Coordinator::
Coordinator(Robot* const _r, XMLNode& _node) : Agent(_r, _node) {

  // Parse the labels of the group members.
  for(auto& child : _node) {
    if(child.Name() == "Member") {
      // Parse the robot label.
      const std::string memberLabel = child.Read("label", true, "",
          "The label of the member robot.");
      m_memberLabels.push_back(memberLabel);
    }
  }

  m_dmLabel = _node.Read("dmLabel", true, "", "Distance metric for checking "
      "nearest agents and charging locations.");

  m_numRandTasks = _node.Read("numRandTasks", false, 0, 0, MAX_INT, "number of "
      "random tasks to generate");

  // This is a coordinator agent, which does not make sense without some group
  // members to coordinate. Throw an exception if it has no members.
  if(m_memberLabels.empty())
    throw ParseException(_node.Where(), "Coordinator requires at "
        "least one member robot.");
}

Coordinator::
~Coordinator() {
  Uninitialize();
}

std::unique_ptr<Agent>
Coordinator::
Clone(Robot* const _r) const {
  throw RunTimeException(WHERE, "Not yet implemented.");
  return {nullptr};
}

/*------------------------------ Agent Interface -----------------------------*/

void
Coordinator::
Initialize() {
  if(m_initialized)
    return;
  m_initialized = true;

  InitializePlanningComponents();

  // Set up the group members.
  auto problem = m_robot->GetMPProblem();
  for(const auto& memberLabel : m_memberLabels) {
    Robot* member = problem->GetRobot(memberLabel);

    // We are assuming that all member robots have a compatible agent type.
    // Throw an exception if not.
    Agent* memberAgent = member->GetAgent();

    ChildAgent* c = static_cast<ChildAgent*>(memberAgent);
    if(c) {
      m_childAgents.push_back(c);
    }
    else {
      throw RunTimeException(WHERE) << "Incompatible agent type specified for "
                   "group member '" << memberLabel << "'." << std::endl;
    }
  }

  if(m_debug) {
    std::cout << "Child Agents" << std::endl;
    for(auto agent : m_childAgents) {
      std::cout << "\t" << agent->GetRobot()->GetLabel() << std::endl;
    }
  }

  InitializeAgents();

  // Generate random tasks if requested
  if(m_numRandTasks > 0) {
    if(problem->GetTasks(m_robot).size() == 1)
      problem->ReassignTask(problem->GetTasks(m_robot)[0].get(),
                            m_childAgents[0]->GetRobot());
    GenerateRandomTasks();
  }

  for(auto agent : m_childAgents) {
    agent->GetRobot()->SetVirtual(false);
  }

  Simulation::Get()->PrintStatFile();
}

void
Coordinator::
InitializePlanningComponents() {
  // Get problem info.
  auto problem = m_robot->GetMPProblem();
  const std::string& xmlFile = problem->GetXMLFilename();

  // Initialize the agent's planning library.
  m_tmpLibrary = new TMPLibrary(xmlFile);
  m_library = m_tmpLibrary->GetMPLibrary();
  m_solution = new MPSolution(m_robot);
  m_library->SetMPSolution(m_solution);
  m_library->SetMPProblem(problem);

  MPTask* task = new MPTask(m_robot);
  m_library->SetTask(task);
}

void
Coordinator::
Step(const double _dt) {
  Initialize();

/*
  if(this->m_debug)
    std::cout << "___________________________________________________________"
              << std::endl;
  for(auto agent : m_memberAgents)  {
    if(this->m_debug and !m_runDummies)
      std::cout << agent->GetRobot()->GetLabel()
                << " has plan: "
                << agent->HasPlan()
                << std::endl
                << "Has task: "
                << agent->GetTask().get()
                << std::endl;
  }
*/
/*
  for(auto agent : m_memberAgents){
    if(m_runDummies) {
      auto dummy = static_cast<HandoffAgent*>(agent);
      dummy->HandoffAgent::Step(_dt);
    }
    else {
      agent->Step(_dt);
    }
  }
  for(auto agent : m_childAgents){
    if(m_runDummies) {
      //auto dummy = static_cast<HandoffAgent*>(agent);
      //dummy->HandoffAgent::Step(_dt);
      auto dummy = static_cast<ChildAgent*>(agent);
      dummy->ChildAgent::Step(_dt);
    }
    //else {
      agent->Step(_dt);
    //}
  }
*/
  //TODO::Undo this comment
  //CheckFinished();

  if(this->m_stepFunction.get())
    this->m_stepFunction->StepAgent(_dt);

  m_currentTime += m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();
}

void
Coordinator::
Uninitialize() {
  if(!m_initialized)
    return;
  m_initialized = false;

  delete m_solution;
  delete m_library;
  delete m_tmpLibrary;

  m_solution = nullptr;
  m_library  = nullptr;

  for(auto id : m_simulatorGraphIDs) {
    Simulation::Get()->RemoveRoadmap(id);
  }
}

/*-------------------------- Coordinator Interface ---------------------------*/


/*--------------------------- Member Management ------------------------------*/

void
Coordinator::
DispatchTo(Agent* const _member, std::unique_ptr<Boundary>&& _where) {
  // Create a task to send the member to the desired location. Use the
  // coordinator robot because this is shared-roadmap planning.
  std::shared_ptr<MPTask> task(new MPTask(m_robot));
  std::unique_ptr<BoundaryConstraint> destination(
      new BoundaryConstraint(m_robot, std::move(_where))
  );

  task->AddGoalConstraint(std::move(destination));
  if(this->m_debug) {
    std::cout << "SENDING " << _member->GetRobot()->GetLabel()
              << " TO:" << std::endl;
    for(const auto& constraint : task->GetGoalConstraints())
      std::cout << "\t" << *(constraint->GetBoundary()) << std::endl;
  }

  // Set the member's current task.
  task->GetStatus().start();
  _member->SetTask(task);
}

double
Coordinator::
GetCurrentTime() {
  return m_currentTime;
}

/*--------------------------- Initialize Functions ------------------------------*/

void
Coordinator::
InitializeAgents() {
  if(m_debug){
    std::cout << "Initializing Agents" << std::endl;
  }
  for(auto agent : m_childAgents) {
    agent->Initialize();
    agent->SetCoordinator(this);
  }
}

/*--------------------------- Helpers ------------------------------*/

TMPLibrary*
Coordinator::
GetTMPLibrary() {
  return m_tmpLibrary;
}

void
Coordinator::
SetRoadmapGraph(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _graph) {
  *m_solution->GetRoadmap(m_robot) = *_graph;
}

void
Coordinator::
GenerateRandomTasks() {
  auto problem = m_robot->GetMPProblem();
  auto env = problem->GetEnvironment();

  m_library->SetSeed();

  auto sampler = m_library->GetSampler("UniformRandomFree");
  auto numAttempts = 1000000;

  auto numNodes = 2;

  size_t numTasks = 0;

  std::set<Cfg> samples;

  while(numTasks < m_numRandTasks) {
    std::vector<Cfg> samplePoints;

    // sample random start and goal
    sampler->Sample(numNodes, numAttempts, env->GetBoundary(),
        std::back_inserter(samplePoints));

    if(samplePoints.size() < 2)
      continue;

    //Temporary for icra discrete stuff
    auto startCfg = samplePoints[0];
    int x = int(startCfg[0]+.5);
    int y = int(startCfg[1]+.5);
    startCfg.SetData({double(x),double(y),0});
    auto goalCfg = samplePoints[1];
    x = int(goalCfg[0]+.5);
    y = int(goalCfg[1]+.5);
    goalCfg.SetData({double(x),double(y),0});

    if(startCfg == goalCfg)
      continue;

    if(samples.count(startCfg) or samples.count(goalCfg))
      continue;

    if(!env->GetBoundary()->InBoundary(startCfg)
    or !env->GetBoundary()->InBoundary(goalCfg))
      continue;
    std::cout << "Sampled task" << std::endl
              << "Start: " << startCfg.PrettyPrint() << std::endl
              << "Goal: " << goalCfg.PrettyPrint() << std::endl;

    // create tasks from sample start and goal
    std::unique_ptr<CSpaceConstraint> start =
      std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(m_robot, startCfg));
    std::unique_ptr<CSpaceConstraint> goal =
      std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(m_robot, goalCfg));

    std::unique_ptr<MPTask> task =
      std::unique_ptr<MPTask>(new MPTask(m_robot));

    task->SetStartConstraint(std::move(start));
    task->AddGoalConstraint(std::move(goal));

    problem->AddTask(std::move(task));

    numTasks++;
    samples.insert(startCfg);
    samples.insert(goalCfg);
  }
}

void
Coordinator::
DistributePlan(Plan* _plan) {
  //for(auto agent : m_memberAgents) {
  for(auto agent : m_childAgents) {
    auto allocs = _plan->GetAllocations(agent->GetRobot());
    std::vector<TaskSolution*> solutions;

    //Temporary dummy task
    std::shared_ptr<MPTask> temp = std::shared_ptr<MPTask>(new MPTask(agent->GetRobot()));
    agent->SetTask(temp);

    for(auto iter = allocs.begin(); iter != allocs.end(); iter++) {
      auto sol = _plan->GetTaskSolution(*iter);
      solutions.push_back(sol);
      auto mpSol = sol->GetMotionSolution();

      if(iter == allocs.begin()) {
        agent->GetMPSolution()->SetRoadmap(agent->GetRobot(),mpSol->GetRoadmap());
        agent->GetMPSolution()->SetPath(agent->GetRobot(),mpSol->GetPath(agent->GetRobot()));
      }
      else {
        *(agent->GetMPSolution()->GetPath(agent->GetRobot())) += *(mpSol->GetPath());
      }
    }
    auto& cfgs = agent->GetMPSolution()->GetPath()->Cfgs();
    agent->SetPlan(cfgs);
    if(cfgs.empty())
      continue;
    auto goalCfg = cfgs.back();
    std::unique_ptr<CSpaceConstraint> goal =
      std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(agent->GetRobot(), goalCfg));
    temp->AddGoalConstraint(std::move(goal));
  }
}

std::vector<std::string>
Coordinator::
GetMemberLabels() {
  return m_memberLabels;
}

std::vector<ChildAgent*>
Coordinator::
GetChildAgents() {
  return m_childAgents;
}
