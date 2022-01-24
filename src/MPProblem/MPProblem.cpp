#include "MPProblem.h"

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/GroupTask.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/DynamicObstacle.h"
#include "MPProblem/InteractionInformation.h"
#include "Utilities/MPUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

using namespace std;


/*---------------------------- Construction ----------------------------------*/

MPProblem::
MPProblem() = default;


MPProblem::
MPProblem(const string& _filename) {
  ReadXMLFile(_filename);
}


MPProblem::
MPProblem(const MPProblem& _other) {
  *this = _other;
}


MPProblem::
~MPProblem() = default;

/*-------------------------------- Assignment --------------------------------*/

MPProblem&
MPProblem::
operator=(const MPProblem& _other) {
  // Guard against self-assignment.
  if(this == &_other)
    return *this;

  if(!_other.m_groupTaskMap.empty())
    throw RunTimeException(WHERE, "The other MPProblem has group tasks present,"
                                  " which cannot yet be copied.");

  // Copy environment.
  m_environment = std::unique_ptr<Environment>(
      new Environment(*_other.m_environment)
  );

  // Copy robots.
  m_robots.clear();
  for(const auto& robot : _other.m_robots) {
    m_robots.emplace_back(new Robot(this, *robot));
    m_robotCapabilityMap[m_robots.back()->
           GetCapability()].push_back(m_robots.back().get());
  }

  // Copy tasks.
  m_taskMap.clear();
  for(const auto& robotTasks : _other.m_taskMap) {
    Robot* const robot = GetRobot(robotTasks.first->GetLabel());
    const auto& tasks = robotTasks.second;

    for(const auto& task : tasks) {
      m_taskMap[robot].emplace_back(new MPTask(*task));
      m_taskMap[robot].back()->SetRobot(robot);
    }
  }

//  // Copy Group Tasks.
//  m_groupTaskMap.clear();
//  for(const auto& groupTasks : _other.m_groupTaskMap) {
//    RobotGroup* const group = groupTasks.first;
//    const auto& tasks = groupTasks.second;
//
//    for(const auto& groupTask : tasks) {
//      m_groupTaskMap[group].emplace_back(new GroupTask(*groupTask));
//      m_groupTaskMap[group].back()->SetRobotGroup(group);
//    }
//  }

  m_pointRobot.reset(new Robot(this, *_other.m_pointRobot));

  m_xmlFilename = _other.m_xmlFilename;
  m_baseFilename = _other.m_baseFilename;
  m_filePath = _other.m_filePath;

  return *this;
}

/*---------------------------- XML Helpers -----------------------------------*/

const std::string&
MPProblem::
GetXMLFilename() const {
  return m_xmlFilename;
}


void
MPProblem::
ReadXMLFile(const string& _filename) {
  const bool envIsSet = m_environment.get();

  size_t sl = _filename.rfind("/");
  m_filePath = _filename.substr(0, sl == string::npos ? 0 : sl + 1);
  m_xmlFilename = _filename;

  // Open the XML and get the root and input nodes.
  XMLNode mpNode(_filename, "MotionPlanning");
  XMLNode input(_filename, "Problem");

  // Check the input node for a base filename. This will only be used by the
  // simulator at present (will be overwritten for individual planning runs).
  m_baseFilename = input.Read("baseFilename", false, "", "The output base name "
      "for simulator stats.");

  // Parse the input node to set the environment, robot(s), and query.
  if(!envIsSet)
    for(auto& child : input)
      ParseChild(child);

  // Print XML details if requested.
  const bool print = mpNode.Read("print", false, false, "Print all XML input");
  if(print)
    Print(cout);

  // Handle XML warnings/errors.
  const bool warnings = mpNode.Read("warnings", false, false, "Report warnings");
  if(warnings) {
    const bool warningsAsErrors = mpNode.Read("warningsAsErrors", false, false,
        "XML warnings considered errors");
    if(!envIsSet)
      input.WarnAll(warningsAsErrors);
  }

  // Make sure there is an environment and a robot.
  if(!m_environment)
    throw ParseException(input.Where(), "No environment specified in the "
        "problem node.");
  if(m_robots.empty())
    throw ParseException(input.Where(), "No robots specified in the problem "
        "node.");

  // If no tasks were specified, assume we want an unconstrained plan for the
  // first robot.
  if(m_taskMap.empty() and m_groupTaskMap.empty()) {
    if(m_robots.size() > 1)
      throw ParseException(input.Where(), "No task was specified in the problem "
          "node, but multiple robots are specified. Taskless execution only "
          "supports single robot problems.");

    auto robot = m_robots.front().get();
    std::cout << "No task specified, assuming we want an unconstrained plan for "
              << "the first robot, labeled \'" << robot->GetLabel() << "\'."
              << std::endl;

    m_taskMap[robot].emplace_back(new MPTask(robot));
  }

  // Compute the environment resolution.
  GetEnvironment()->ComputeResolution(GetRobots());
}

/*----------------------------- Environment Accessors ------------------------*/

Environment*
MPProblem::
GetEnvironment() {
  return m_environment.get();
}


void
MPProblem::
SetEnvironment(std::unique_ptr<Environment>&& _e) {
  m_environment = std::move(_e);

  // Reset point robot in the case of changing the environment.
  m_pointRobot.reset();
  MakePointRobot();

  // Reset the robot DOF limits based on the new environment.
  for(auto& robot : m_robots)
    robot->InitializePlanningSpaces();
}

/*------------------------------ Robot Accessors -----------------------------*/

size_t
MPProblem::
NumRobots() const noexcept {
  return m_robots.size();
}


Robot*
MPProblem::
GetRobot(const size_t _index) const noexcept {
  if(_index >= m_robots.size())
    throw RunTimeException(WHERE) << "Requested Robot " << _index
                                  << ", but only " << m_robots.size()
                                  << " robots are available.";
  return m_robots[_index].get();
}


Robot*
MPProblem::
GetRobot(const std::string& _label) const noexcept {
  if(_label == "point")
    return m_pointRobot.get();

  for(auto& robot : m_robots)
    if(robot->GetLabel() == _label)
      return robot.get();

  throw RunTimeException(WHERE) << "No robot found with label '" << _label
                                << "'.";
}


const std::vector<std::unique_ptr<Robot>>&
MPProblem::
GetRobots() const noexcept {
  return m_robots;
}

const std::vector<Robot*> 
MPProblem::
GetRobotsOfType(std::string _type) const noexcept {
  return m_robotCapabilityMap.at(_type);
}

size_t
MPProblem::
NumRobotGroups() const noexcept {
  return m_robotGroups.size();
}


RobotGroup*
MPProblem::
GetRobotGroup(const size_t _index) const noexcept {
  if(_index >= m_robotGroups.size())
    throw RunTimeException(WHERE) << "Requested robot group " << _index
                                  << ", but only " << m_robotGroups.size()
                                  << " groups are available.";
  return m_robotGroups[_index].get();
}


RobotGroup*
MPProblem::
GetRobotGroup(const std::string& _label) const noexcept {
  for(auto& group : m_robotGroups)
    if(group->GetLabel() == _label)
      return group.get();

  throw RunTimeException(WHERE) << "No robot group found with label '" << _label
                                << "'.";
}


const std::vector<std::unique_ptr<RobotGroup>>&
MPProblem::
GetRobotGroups() const noexcept {
  return m_robotGroups;
}

Cfg 
MPProblem::
GetInitialCfg(Robot* _r) {
	return m_initialCfgs[_r];
}
		
void 
MPProblem::
SetInitialCfg(Robot* _r, Cfg _cfg) {
	if(_r != _cfg.GetRobot())
		throw RunTimeException(WHERE) << "Initial Cfg robot does not match." << std::endl;
	m_initialCfgs[_r] = _cfg;
}

/*----------------------------- Task Accessors -------------------------------*/

MPTask*
MPProblem::
GetTask(std::string _label) {
  return m_taskLabelMap[_label];
}

std::vector<std::shared_ptr<MPTask>>
MPProblem::
GetTasks(Robot* const _robot) const noexcept {
  if(m_taskMap.empty())
    return std::vector<std::shared_ptr<MPTask>>();

  const auto& tasks = m_taskMap.at(_robot);

  std::vector<std::shared_ptr<MPTask>> output;

  for(const auto& task : tasks)
    if(!task->GetStatus().is_complete())
      output.push_back(task);

  return output;
}


std::vector<std::shared_ptr<GroupTask>>
MPProblem::
GetTasks(RobotGroup* const _group) const noexcept {
  const auto& tasks = m_groupTaskMap.at(_group);

  std::vector<std::shared_ptr<GroupTask>> output;

  for(const auto& task : tasks)
    if(!task->GetStatus().is_complete())
      output.push_back(task);

  return output;
}

void
MPProblem::
AddTask(std::unique_ptr<MPTask>&& _task) {
  auto robot = _task->GetRobot();
  m_taskMap[robot].push_back(std::move(_task));
}


void
MPProblem::
ReassignTask(MPTask* const _task, Robot* const _newOwner) {
  // Find an iterator to the existing entry in the task map.
  auto oldOwner = _task->GetRobot();
  auto& oldTasks = m_taskMap.at(oldOwner);
  auto iter = oldTasks.begin();
  for(; iter != oldTasks.end(); ++iter)
    if(iter->get() == _task)
      break;

  // If we hit the end, the task was not found.
  if(iter == oldTasks.end())
    throw RunTimeException(WHERE, "Requested task was not found.");

  // Set the task's robot.
  (*iter)->SetRobot(_newOwner);

  // Move the task to the new owner's map.
  auto& newTasks = m_taskMap.at(_newOwner);
  newTasks.emplace_back(std::move(*iter));
  oldTasks.erase(iter);
}

void
MPProblem::
AddDecomposition(Robot* _coordinator, std::unique_ptr<Decomposition>&& _decomp) {
	m_taskDecompositions[_coordinator].push_back(std::move(_decomp));
}
		
const std::vector<std::unique_ptr<Decomposition>>& 
MPProblem::
GetDecompositions(Robot* _coordinator) {
	return m_taskDecompositions[_coordinator];
}

const std::unordered_map<Robot*,std::vector<std::unique_ptr<Decomposition>>>& 
MPProblem::
GetDecompositions() {
	return m_taskDecompositions;
}

/*---------------------------- Dynamic Obstacles -----------------------------*/

const std::vector<DynamicObstacle>&
MPProblem::
GetDynamicObstacles() const noexcept {
  return m_dynamicObstacles;
}


void
MPProblem::
AddDynamicObstacle(DynamicObstacle&& _obstacle) {
  m_dynamicObstacles.emplace_back(std::move(_obstacle));
}


void
MPProblem::
ClearDynamicObstacles() {
  m_dynamicObstacles.clear();
}

/*-------------------------------- Debugging ---------------------------------*/

void
MPProblem::
Print(ostream& _os) const {
  _os << "MPProblem"
      << std::endl;
  m_environment->Print(_os);
  /// @todo Print robot and task information.
}

/*-------------------------- File Path Accessors -----------------------------*/

const std::string&
MPProblem::
GetBaseFilename() const {
  return m_baseFilename;
}


void
MPProblem::
SetBaseFilename(const std::string& _s) {
  m_baseFilename = _s;
}


string
MPProblem::
GetPath(const string& _filename) {
  if(!_filename.empty() and _filename[0] != '/')
    return m_filePath + _filename;
  else
    return _filename;
}


void
MPProblem::
SetPath(const string& _filename) {
  m_filePath = _filename;
}

/*---------------------------- Handoff Template Accessors -------------------*/

std::vector<std::unique_ptr<InteractionInformation>>&
MPProblem::
GetInteractionInformations() {
  return m_interactionInformations;
}

/*---------------------------- Construction Helpers --------------------------*/

void
MPProblem::
ParseChild(XMLNode& _node) {
  /// @todo We currently assume that the environment is parsed first. Need to
  ///       make sure this always happens regardless of the XML file ordering.
  if(_node.Name() == "Environment") {
    // Ignore this node if we already have an environment.
    if(!m_environment)
      m_environment = std::unique_ptr<Environment>(new Environment(_node));
    MakePointRobot();
  }
  else if(_node.Name() == "Robot") {
    m_robots.emplace_back(new Robot(this, _node));
    m_robotCapabilityMap[m_robots.back()->
           GetCapability()].push_back(m_robots.back().get());
  }
  else if(_node.Name() == "RobotGroup") {
    m_robotGroups.emplace_back(new RobotGroup(this, _node));
  }
  else if(_node.Name() == "DynamicObstacle") {
    m_dynamicObstacles.emplace_back(_node, this);
  }
  else if(_node.Name() == "Task") {
    const std::string label = _node.Read("robot", true, "",
        "Label for the robot assigned to this task.");
    auto robot = this->GetRobot(label);
    auto task = std::shared_ptr<MPTask>(new MPTask(this, _node));
    m_taskMap[robot].push_back(task);
    /// @todo Add check that tasks have unique labels
    m_taskLabelMap[task->GetLabel()] = task.get();
  }
  else if(_node.Name() == "GroupTask") {
    auto task = GroupTask::Factory(this, _node);
    auto group = this->GetRobotGroup(task->GetRobotGroup()->GetLabel());
    m_groupTaskMap[group].emplace_back(std::move(task));
  }
  else if(_node.Name() == "HandoffTemplate") {
    m_interactionInformations.emplace_back(std::unique_ptr<InteractionInformation>(
                                           new InteractionInformation(this, _node)));
  }
	else if(_node.Name() == "Decomposition") {
		auto decomp = std::unique_ptr<Decomposition>(new Decomposition(_node,this));
		m_taskDecompositions[decomp->GetCoordinator()].push_back(std::move(decomp));
	}
}


void
MPProblem::
MakePointRobot() {
  // Make robot's multibody.
  std::unique_ptr<MultiBody> point(new MultiBody(MultiBody::Type::Active));

  const bool is2d = GetEnvironment()->GetBoundary()->GetDimension() == 2;

  Body body(point.get());
  body.SetBodyType(is2d ? Body::Type::Planar : Body::Type::Volumetric);
  body.SetMovementType(Body::MovementType::Translational);

  // Create body geometry. Use a single, open triangle. Make sure it isn't
  // co-planar with any axis-pair-plane to get a non-degenerate bounding box.
  GMSPolyhedron poly;
  poly.GetVertexList() = std::vector<Vector3d>{{1e-8,    0,    0},
                                               {   0, 1e-8,    0},
                                               {   0,    0, 1e-8}};
  poly.GetPolygonList() = vector<GMSPolygon>{GMSPolygon(0, 1, 2,
      poly.GetVertexList())};
  // Translate the triangle so that its center is on the origin.
  const Vector3d center = poly.GetPolygonList()[0].FindCenter();
  for(auto& vertex : poly.GetVertexList())
    vertex -= center;
  body.SetPolyhedron(std::move(poly));

  // Add body geometry to multibody.
  const size_t index = point->AddBody(std::move(body));
  point->SetBaseBody(index);

  // Make sure we didn't accidentally call another robot 'point'.
  if(GetRobot("point"))
    throw RunTimeException(WHERE, "A robot in the problem is named 'point'. "
        "This name is reserved for the internal point robot.");

  // Create the robot object.
  m_pointRobot = std::unique_ptr<Robot>(
      new Robot(this, std::move(point), "point")
  );
  m_pointRobot->SetVirtual(true);
}

/*----------------------------------------------------------------------------*/
