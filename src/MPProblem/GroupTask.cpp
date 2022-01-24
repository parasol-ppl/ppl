#include "GroupTask.h"

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "Geometry/Boundaries/Boundary.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"


/*------------------------------ Construction --------------------------------*/


GroupTask::
GroupTask(RobotGroup* const _robotGroup)
  : m_group(_robotGroup)
{ }


GroupTask::
GroupTask(MPProblem* const _problem, XMLNode& _node) {
  // Parse task and robot labels.
  m_label = _node.Read("label", true, "", "Unique label for this task");

  // Get the robot group by label.
  const std::string groupLabel = _node.Read("group", true, "",
      "Label for the robot group assigned to this task.");
  m_group = _problem->GetRobotGroup(groupLabel);

  // Parse individual robot tasks.
  for(auto& child : _node) {
    if(child.Name() == "Task") {
      // Parse the individual task.
      MPTask robotTask(_problem, child);

      // Ensure that the task belongs to a robot in this group.
      Robot* const robot = robotTask.GetRobot();
      if(!m_group->VerifyRobotInGroup(robot))
        throw ParseException(child.Where()) << "Robot '" << robot->GetLabel()
                                            << "' is not in group '"
                                            << m_group->GetLabel() << "'.";

      m_individualTasks.emplace_back(std::move(robotTask));
    }
  }

  // Disassembly items, to be moved to a specialized subclass.

  // Look for an end-effector group.
  const std::string effectorGroupLabel = _node.Read("endEffectorGroupLabel",
           false, "",
           "Label for the robot group with manipulator assigned to this task.");
  if(!effectorGroupLabel.empty())
    m_endEffectorGroup = _problem->GetRobotGroup(effectorGroupLabel);

  // Look for a manipulator group.
  const std::string manipGroupLabel = _node.Read("manipulatorGroupLabel", false,
      "", "Label for the robot group with manipulator assigned to this task.");
  if(!manipGroupLabel.empty())
    m_manipulatorGroup = _problem->GetRobotGroup(manipGroupLabel);
}


GroupTask::
~GroupTask() = default;


std::unique_ptr<GroupTask>
GroupTask::
Factory(MPProblem* const _problem, XMLNode& _node) {
  // Read the task type.
  std::string type = _node.Read("type", false, "",
      "The task type (currently none are supported).");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  if(type == "")
    return std::unique_ptr<GroupTask>(new GroupTask(_problem, _node));
  else if(type == "disassembly")
    throw NotImplementedException(_node.Where());
  else
    throw ParseException(_node.Where()) << "Unrecognized task type '" << type
                                        << "'.";
}

/*--------------------------- Property Accessors -----------------------------*/

RobotGroup*
GroupTask::
GetRobotGroup() const noexcept {
  return m_group;
}


void
GroupTask::
SetRobotGroup(RobotGroup* const _r) {
  if(m_group == _r)
    return;

  // Clear the current robot assigned to each task.
  for(auto& task : *this)
    task.SetRobot(nullptr);

  m_group = _r;
}


const std::string&
GroupTask::
GetLabel() const noexcept {
  return m_label;
}


void
GroupTask::
SetLabel(const std::string& _label) noexcept {
  m_label = _label;
}


size_t
GroupTask::
Size() const noexcept {
  return m_individualTasks.size();
}


bool
GroupTask::
Empty() const noexcept {
  if(m_individualTasks.empty())
    return true;

  for(const auto& task : m_individualTasks)
    if(!task.Empty())
      return false;

  return true;
}


nonstd::status&
GroupTask::
GetStatus() noexcept {
  return m_status;
}


const nonstd::status&
GroupTask::
GetStatus() const noexcept {
  return m_status;
}

/*----------------------------- Individual Tasks -----------------------------*/

size_t
GroupTask::
GetNumGoals() const noexcept {
  size_t max = 0;
  for(const auto& task : *this)
    max = std::max(max, task.GetNumGoals());

  return max;
}


void
GroupTask::
AddTask(const MPTask& _t) {
  m_individualTasks.push_back(_t);
}


GroupTask::iterator
GroupTask::
RemoveTask(iterator _iter) {
  return m_individualTasks.erase(_iter);
}


GroupTask::iterator
GroupTask::
begin() noexcept {
  return m_individualTasks.begin();
}


GroupTask::iterator
GroupTask::
end() noexcept {
  return m_individualTasks.end();
}


GroupTask::const_iterator
GroupTask::
begin() const noexcept {
  return m_individualTasks.begin();
}


GroupTask::const_iterator
GroupTask::
end() const noexcept {
  return m_individualTasks.end();
}

/*---------------------------- Disassembly Items -----------------------------*/

RobotGroup*
GroupTask::
GetEndEffectorGroup() const noexcept {
  return m_endEffectorGroup;
}


RobotGroup*
GroupTask::
GetManipulatorGroup() const noexcept {
  return m_manipulatorGroup;
}

Robot*
GroupTask::
GetManipulatorRobot() {
  if(!m_manipulatorGroup)
    return nullptr;

  // If the manipulator group is present, return the manipulator.
  /// @todo This uses magic strings in the XML files, let's find a better way.
  return m_manipulatorGroup->GetRobot("manipulator");
}


Robot*
GroupTask::
GetEndEffectorRobot() {
  if(!m_manipulatorGroup)
    return nullptr;

  // If the manipulator group is present, return the manipulator.
  /// @todo This uses magic strings in the XML files, let's find a better way.
  return m_endEffectorGroup->GetRobot("effector");
}


/*-------------------------- Constraint Accessors ----------------------------*/

void
GroupTask::
GetStartConstraintCenter(GroupCfg& _center) const noexcept {
  for(size_t i = 0; i < _center.GetNumRobots(); ++i) {
    Cfg robotCfg(_center.GetRobot(i));
    robotCfg.SetData(m_individualTasks.at(i).GetStartConstraint()->
        GetBoundary()->GetCenter());
    _center.SetRobotCfg(i, std::move(robotCfg));
  }
}

/*---------------------------- Constraint Evaluation -------------------------*/

bool
GroupTask::
EvaluateStartConstraints(const GroupCfg& _cfg) const {
  for(const auto& task : m_individualTasks) {
    // For now we will require all individual tasks to be assigned to a robot.
    auto robot = task.GetRobot();
    if(!robot)
      throw NotImplementedException(WHERE) << "Currently each individual task "
                                           << "must be assigned to a robot( "
                                           << "task '" << task.GetLabel()
                                           << "' is not assigned).";
    const auto& cfg = _cfg.GetRobotCfg(robot);
    const bool satisfied = task.EvaluateStartConstraints(cfg);
    if(!satisfied)
      return false;
  }

  return true;
}


bool
GroupTask::
EvaluatePathConstraints(const GroupCfg& _cfg) const {
  for(const auto& task : m_individualTasks) {
    // For now we will require all individual tasks to be assigned to a robot.
    auto robot = task.GetRobot();
    if(!robot)
      throw NotImplementedException(WHERE) << "Currently each individual task "
                                           << "must be assigned to a robot ("
                                           << "task '" << task.GetLabel()
                                           << "' is not assigned).";
    const auto& cfg = _cfg.GetRobotCfg(robot);
    const bool satisfied = task.EvaluatePathConstraints(cfg);
    if(!satisfied)
      return false;
  }

  return true;
}


bool
GroupTask::
EvaluateGoalConstraints(const GroupCfg& _cfg) const {
  for(const auto& task : m_individualTasks) {
    // For now we will require all individual tasks to be assigned to a robot.
    auto robot = task.GetRobot();
    if(!robot)
      throw NotImplementedException(WHERE) << "Currently each individual task "
                                           << "must be assigned to a robot( "
                                           << "task '" << task.GetLabel()
                                           << "' is not assigned).";
    const auto& cfg = _cfg.GetRobotCfg(robot);
    const bool satisfied = task.EvaluateGoalConstraints(cfg);
    if(!satisfied)
      return false;
  }

  return true;
}


bool
GroupTask::
EvaluateGoalConstraints(const GroupCfg& _cfg, const size_t _index) const {
  const size_t maxGoals = GetNumGoals();

  for(const auto& task : m_individualTasks) {
    // For now we will require all individual tasks to be assigned to a robot.
    auto robot = task.GetRobot();
    if(!robot)
      throw NotImplementedException(WHERE) << "Currently each individual task "
                                           << "must be assigned to a robot( "
                                           << "task '" << task.GetLabel()
                                           << "' is not assigned).";
    const auto& cfg = _cfg.GetRobotCfg(robot);

    // Determine the index offset so that all goals are right-aligned. If the
    // index is too early then there is no goal constraint.
    const size_t offset = maxGoals - task.GetNumGoals();
    const bool noConstraint = _index < offset;
    if(noConstraint)
      continue;

    const bool satisfied = task.EvaluateGoalConstraints(cfg, _index - offset);
    if(!satisfied)
      return false;
  }

  return true;
}

/*----------------------------------------------------------------------------*/
