#include "TMPLibrary.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "Traits/TMPTraits.h"

#include "Simulator/Simulation.h"
#include "TMPLibrary/PoIPlacementMethods/PoIPlacementMethod.h"
#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"
#include "TMPLibrary/StateGraphs/StateGraph.h"
#include "TMPLibrary/TaskAllocators/TaskAllocatorMethod.h"
#include "TMPLibrary/TaskDecomposers/TaskDecomposerMethod.h"
#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"
#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"
#include "TMPLibrary/TMPTools/TMPTools.h"

/*---------------------------- Construction ----------------------------------*/

TMPLibrary::
TMPLibrary() {
  m_tmpStrategies = new TMPStrategyMethodSet(this,
                  typename TMPTraits::TMPStrategyMethodList(), "TMPStrategies");
  m_poiPlacementMethods = new PoIPlacementMethodSet(this,
                  typename TMPTraits::PoIPlacementMethodList(), "PoIPlacementMethods");
  m_taskAllocators = new TaskAllocatorMethodSet(this,
                  typename TMPTraits::TaskAllocatorMethodList(), "TaskAllocators");
  m_taskDecomposers = new TaskDecomposerMethodSet(this,
                  typename TMPTraits::TaskDecomposerMethodList(), "TaskDecomposers");
  m_taskEvaluators = new TaskEvaluatorMethodSet(this,
                  typename TMPTraits::TaskEvaluatorMethodList(), "TaskEvaluators");
  m_stateGraphs = new StateGraphSet(this,
                  typename TMPTraits::StateGraphList(), "StateGraphs");
  m_tmpTools = new TMPTools(this);
}


TMPLibrary::
TMPLibrary(const std::string& _filename) : TMPLibrary() {
  m_library = new MPLibrary(_filename);
  m_library->SetSeed();
  ReadXMLFile(_filename);
}


TMPLibrary::
~TMPLibrary() {
  delete m_tmpStrategies;
  delete m_poiPlacementMethods;
  delete m_taskEvaluators;
  delete m_taskDecomposers;
  delete m_taskAllocators;
  delete m_stateGraphs;
}

void
TMPLibrary::
Initialize() {
  m_tmpStrategies->Initialize();
  m_poiPlacementMethods->Initialize();
  m_taskEvaluators->Initialize();
  m_taskDecomposers->Initialize();
  m_taskAllocators->Initialize();
  this->GetPlan()->GetStatClass()->StartClock("StateGraphConstruction");
  m_stateGraphs->Initialize();
  this->GetPlan()->GetStatClass()->StopClock("StateGraphConstruction");
}

void
TMPLibrary::
Uninitialize() {}

/*---------------------------- XML Helpers -----------------------------------*/

void
TMPLibrary::
ReadXMLFile(const std::string& _filename) {
  // Open the XML and get the root node.
  XMLNode tmpNode(_filename, "MotionPlanning");

  // Find the 'TMPLibrary' node.
  XMLNode* tmpLibrary = nullptr;
  for(auto& child : tmpNode)
    if(child.Name() == "TMPLibrary")
      tmpLibrary = &child;

  // Throw exception if we can't find it.
  if(!tmpLibrary)
    throw ParseException(WHERE) << "Cannot find TMPLibrary node in XML file '"
            << _filename << "'.";

  // Parse the library node to set algorithms and parameters.
  for(auto& child : *tmpLibrary)
    ParseChild(child);

  // Ensure we have at least one solver.
  if(m_solvers.empty())
    throw ParseException(WHERE) << "Cannot find Solver node in XML file '"
            << _filename << "'.";

  // Print XML details if requested.
  bool print = tmpNode.Read("print", false, false, "Print all XML input");
  if(print)
    Print(cout);

  // Handle XML warnings/errors.
  bool warnings = tmpNode.Read("warnings", false, false, "Report warnings");
  if(warnings) {
    bool warningsAsErrors = tmpNode.Read("warningsAsErrors", false, false,
                    "XML warnings considered errors");
    tmpLibrary->WarnAll(warningsAsErrors);
  }
}

bool
TMPLibrary::
ParseChild(XMLNode& _node) {
  if(_node.Name() == "TMPStrategies") {
    m_tmpStrategies->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "PoIPlacementMethods") {
    m_poiPlacementMethods->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "TaskEvaluators") {
    m_taskEvaluators->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "TaskDecomposers") {
    m_taskDecomposers->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "TaskAllocators") {
    m_taskAllocators->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "StateGraphs") {
    m_stateGraphs->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "TMPTools") {
    m_tmpTools->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Solver") {
    const std::string label = _node.Read("tmpStrategyLabel", true, "",
                    "The strategy to use.");

    const std::string baseFilename = _node.Read("baseFilename", true, "",
                    "BaseFilename for the solver.");// + "." + std::to_string(seed);

    //const bool vdOutput = _node.Read("vizmoDebug", false, false,
    //              "True yields VizmoDebug output for the solver.");

    m_solvers.emplace_back(Solver{label, baseFilename});
    return true;
  }
  else
    return false;
}

/*---------------------------- TMPStrategy Accessors -----------------------------------*/

TMPLibrary::TMPStrategyMethodPointer
TMPLibrary::
GetTMPStrategy(const std::string& _l) {
  return m_tmpStrategies->GetMethod(_l);
}

void
TMPLibrary::
AddTMPStrategy(TMPStrategyMethodPointer _sm, const std::string& _l) {
  m_tmpStrategies->AddMethod(_sm,_l);
}

TMPLibrary::PoIPlacementMethodPointer
TMPLibrary::
GetPoIPlacementMethod(const std::string& _l) {
  return m_poiPlacementMethods->GetMethod(_l);
}

void
TMPLibrary::
TMPLibrary::AddPoIPlacementMethod(PoIPlacementMethodPointer _pm, const std::string& _l) {
  m_poiPlacementMethods->AddMethod(_pm,_l);
}

TMPLibrary::TaskEvaluatorMethodPointer
TMPLibrary::
GetTaskEvaluator(const std::string& _l) {
  return m_taskEvaluators->GetMethod(_l);
}

void
TMPLibrary::
AddTaskEvaluator(TaskEvaluatorMethodPointer _te, const std::string& _l) {
  m_taskEvaluators->AddMethod(_te,_l);
}

TMPLibrary::TaskDecomposerMethodPointer
TMPLibrary::
GetTaskDecomposer(const std::string& _l) {
  return m_taskDecomposers->GetMethod(_l);
}

void
TMPLibrary::
AddTaskDecomposer(TaskDecomposerMethodPointer _td, const std::string& _l) {
  m_taskDecomposers->AddMethod(_td,_l);
}

TMPLibrary::TaskAllocatorMethodPointer
TMPLibrary::
GetTaskAllocator(const std::string& _l) {
  return m_taskAllocators->GetMethod(_l);
}

void
TMPLibrary::
AddTaskAllocator(TaskAllocatorMethodPointer _ta, const std::string& _l) {
  m_taskAllocators->AddMethod(_ta,_l);
}

/*---------------------------- Solution Accessors -----------------------------------*/

Plan*
TMPLibrary::
GetPlan() {
  return m_plan;
}

void
TMPLibrary::
SetPlan(Plan* _plan) {
  m_plan = _plan;
}

TMPLibrary::StateGraphPointer
TMPLibrary::
GetStateGraph(const std::string& _l) {
  return m_stateGraphs->GetMethod(_l);
}

void
TMPLibrary::
AddStateGraph(StateGraphPointer _sg, const std::string& _l) {
  m_stateGraphs->AddMethod(_sg,_l);
}

/*-------------------------------- Debugging ---------------------------------*/

void
TMPLibrary::
Print(ostream& _os) const {
  _os << "TMPLibrary" << std::endl;
  m_tmpStrategies->Print(_os);
  m_poiPlacementMethods->Print(_os);
  m_taskEvaluators->Print(_os);
  m_taskDecomposers->Print(_os);
  m_taskAllocators->Print(_os);
  m_stateGraphs->Print(_os);
}

/*----------------------------- Input Accessors ------------------------------*/

MPLibrary*
TMPLibrary::
GetMPLibrary() const noexcept {
  return m_library;
}

void
TMPLibrary::
SetMPLibrary(MPLibrary* _l) noexcept {
  m_library = _l;
}

MPProblem*
TMPLibrary::
GetMPProblem() const noexcept {
  return m_problem;
}

void
TMPLibrary::
SetMPProblem(MPProblem* const _problem) noexcept {
  m_problem = _problem;
}

std::vector<std::shared_ptr<MPTask>>&
TMPLibrary::
GetTasks() noexcept {
  return m_tasks;
}

void
TMPLibrary::
AddTask(MPTask* const _task) noexcept {
  m_tasks.emplace_back(std::shared_ptr<MPTask>(_task));
}

void
TMPLibrary::
AddTask(std::shared_ptr<MPTask> const _task) noexcept {
  m_tasks.push_back(_task);
}

void
TMPLibrary::
ClearTasks() {
  m_tasks.clear();
}

std::vector<std::shared_ptr<GroupTask>>
TMPLibrary::
GetGroupTasks() const noexcept {
  return m_groupTasks;
}

void
TMPLibrary::
AddGroupTask(GroupTask* const _task) noexcept {
  m_groupTasks.emplace_back(std::shared_ptr<GroupTask>(_task));
}

void
TMPLibrary::
AddGroupTask(std::shared_ptr<GroupTask> const _task) noexcept {
  m_groupTasks.push_back(_task);
}

inline
void
TMPLibrary::
ClearGroupTasks() {
  m_groupTasks.clear();
}

const std::string&
TMPLibrary::
GetBaseFilename() const noexcept {
  return m_problem->GetBaseFilename();
}

void
TMPLibrary::
SetBaseFilename(const std::string& _s) noexcept {
  m_problem->SetBaseFilename(_s);
}

/*--------------------------- Execution Interface ----------------------------*/

void
TMPLibrary::
Solve(MPProblem* _problem,
      Decomposition* _decomp,
      Plan* _plan,
      Coordinator* _coordinator,
      std::vector<Robot*> _team) {

  m_problem = _problem;
  m_plan = _plan;
  m_plan->SetMPProblem(_problem);
  m_plan->SetCoordinator(_coordinator);
  m_plan->SetTeam(_team);
  //m_taskPlan->CreateWholeTasks(_decomp);

  for(auto task : _decomp->GetTaskMap()) {
    auto sol = std::shared_ptr<TaskSolution>(new TaskSolution(task.second.get()));
    m_plan->SetTaskSolution(task.second.get(),sol);
  }

  auto solution = new MPSolution(_coordinator->GetRobot());
    m_library->SetMPSolution(solution);

  for(auto task : _decomp->GetMotionTasks())
    m_tasks.push_back(task->GetMotionTask());

  for(auto& solver : m_solvers)
    RunSolver(solver);
}

void
TMPLibrary::
InitializeMPProblem(MPProblem* _problem) {
  SetMPProblem(_problem);
  Initialize();
}

void
TMPLibrary::
RunSolver(const Solver& _solver) {
  std::string originalBaseFilename = m_problem->GetBaseFilename();
  Initialize();

  // Call solver
  std::cout << "\n\nTMPLibrary is solving with TMPStrategyMethod labeled "
                  << _solver.label << std::endl;

  // If this task has a label, append it to the solver's output file name.
  std::string baseFilename = _solver.baseFilename;
  // Remove spaces to keep file names nice.
  {
    auto newEnd = std::remove_if(baseFilename.begin(), baseFilename.end(),
                    ::isspace);
    baseFilename.erase(newEnd, baseFilename.end());
  }

  SetBaseFilename(GetMPProblem()->GetPath(baseFilename));
  //m_library->GetStatClass()->SetAuxDest(GetBaseFilename());
  //If you are getting a seg fault here, it is bc you are still using a TaskPlan
  //based tmpLibrary approach. This is slowly being phased out and this is where
  //you get caught using the old way. Look at the options above and pick one that
  //initializes a Plan object (m_plan) or fix the one you're currently using.
  m_plan->GetStatClass()->SetAuxDest(GetBaseFilename());

  // Initialize vizmo debug if there is a valid filename
  //if(_solver.vizmoDebug)
  //  VDInit(GetBaseFilename() + ".vd");

  GetTMPStrategy(_solver.label)->operator()();

  // Close vizmo debug if necessary
  //if(_solver.vizmoDebug)
  //  VDClose();

  Uninitialize();
  m_problem->SetBaseFilename(originalBaseFilename);
}
