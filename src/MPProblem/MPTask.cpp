#include "MPTask.h"

#include "Geometry/Boundaries/Boundary.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"


/*------------------------------ Construction --------------------------------*/

MPTask::
MPTask(Robot* const _robot) : m_robot(_robot) {
	m_releaseWindow = std::make_pair(0,MAX_DBL);
	m_deadlineWindow = std::make_pair(0,MAX_DBL);
}


MPTask::
MPTask(MPProblem* const _problem, XMLNode& _node) {
  // Parse task and robot labels.
  m_label = _node.Read("label", true, "", "Unique label for this task");

  // Get the robot by label.
  const std::string robotLabel = _node.Read("robot", true, "", "Label for the "
      "robot assigned to this task.");
  m_robot = _problem->GetRobot(robotLabel);

  // Read the required capability, if any.
  m_capability = _node.Read("capability", false, "",
      "Indicates the capability of the robot performing the task.");
  std::transform(m_capability.begin(), m_capability.end(), m_capability.begin(),
      ::tolower);

	m_releaseWindow.first = _node.Read("releaseStart",false,0.0,0.0,MAX_DBL,
			"Indicates the time the task is released and can be initiated.");
	m_releaseWindow.second = _node.Read("releaseEnd",false,MAX_DBL,0.0,MAX_DBL,
			"Indicates the final time the task can be initiated.");

	m_deadlineWindow.first = _node.Read("deadlineStart",false,0.0,0.0,MAX_DBL,
			"Indicates the minimum time the task can be completed.");
	m_deadlineWindow.second = _node.Read("deadlineEnd",false,MAX_DBL,0.0,MAX_DBL,
			"Indicates the maximum time the task can be completed.");

  // Parse constraints.
  for(auto& child : _node) {
    if(child.Name() == "StartConstraints") {
      for(auto& grandChild : child)
        m_startConstraint = Constraint::Factory(m_robot, grandChild);
    }
    else if(child.Name() == "PathConstraints") {
      for(auto& grandChild : child)
        m_pathConstraints.push_back(Constraint::Factory(m_robot, grandChild));
    }
    else if(child.Name() == "GoalConstraints") {
      for(auto& grandChild : child)
        m_goalConstraints.push_back(Constraint::Factory(m_robot, grandChild));
    }
  }
}


MPTask::
MPTask(const MPTask& _other) {
  *this = _other;
}


MPTask::
MPTask(MPTask&& _other) = default;


MPTask::
~MPTask() = default;

/*-------------------------------- Assignment --------------------------------*/

MPTask&
MPTask::
operator=(const MPTask& _other) {
  if(this != &_other) {
    m_label = _other.m_label;
    m_robot = _other.m_robot;

    if(_other.m_startConstraint.get())
      m_startConstraint = _other.m_startConstraint->Clone();
    else
      m_startConstraint.reset();

    m_pathConstraints.clear();
    for(const auto& c : _other.m_pathConstraints)
      m_pathConstraints.push_back(c->Clone());

    m_goalConstraints.clear();
    for(const auto& c : _other.m_goalConstraints)
      m_goalConstraints.push_back(c->Clone());
  }

  return *this;
}


MPTask&
MPTask::
operator=(MPTask&& _other) = default;

/*--------------------------- Property Accessors -----------------------------*/

Robot*
MPTask::
GetRobot() const noexcept {
  return m_robot;
}


void
MPTask::
SetRobot(Robot* const _r) {
  m_robot = _r;

  if(m_startConstraint.get())
    m_startConstraint->SetRobot(_r);
  for(auto& c : m_pathConstraints)
    c->SetRobot(_r);
  for(auto& c : m_goalConstraints)
    c->SetRobot(_r);
}


const std::string&
MPTask::
GetLabel() const noexcept {
  return m_label;
}


void
MPTask::
SetLabel(const std::string& _label) noexcept {
  m_label = _label;
}


nonstd::status&
MPTask::
GetStatus() noexcept {
  return m_status;
}


const nonstd::status&
MPTask::
GetStatus() const noexcept {
  return m_status;
}


bool
MPTask::
Empty() const noexcept {
  return !m_startConstraint.get() and m_goalConstraints.empty();
}


size_t
MPTask::
GetNumGoals() const noexcept {
  return m_goalConstraints.size();
}

/*-------------------------- Constraint Accessors ----------------------------*/

void
MPTask::
SetStartConstraint(std::unique_ptr<Constraint>&& _c) {
  m_startConstraint = std::move(_c);
}


void
MPTask::
AddPathConstraint(std::unique_ptr<Constraint>&& _c) {
  m_pathConstraints.push_back(std::move(_c));
}


void
MPTask::
AddGoalConstraint(std::unique_ptr<Constraint>&& _c) {
  m_goalConstraints.push_back(std::move(_c));
}


const Constraint*
MPTask::
GetStartConstraint() const noexcept {
  return m_startConstraint.get();
}


const MPTask::ConstraintSet&
MPTask::
GetPathConstraints() const noexcept {
  return m_pathConstraints;
}


const MPTask::ConstraintSet&
MPTask::
GetGoalConstraints() const noexcept {
  return m_goalConstraints;
}


void
MPTask::
ClearGoalConstraints(){
  m_goalConstraints.clear();
}

void
MPTask::
SetCapability(const std::string& _capability) {
  m_capability = _capability;
}


const std::string&
MPTask::
GetCapability() const noexcept {
  return m_capability;
}

/*------------------------------ Time Accessors ------------------------------*/

void
MPTask::
SetEstimatedStartTime(const double _time) noexcept {
  m_startTime = _time;
}


void
MPTask::
SetEstimatedCompletionTime(const double _time) noexcept {
  m_finishTime = _time;
}


double
MPTask::
GetEstimatedCompletionTime() const noexcept {
  return m_finishTime;
}


double
MPTask::
GetEstimatedStartTime() const noexcept {
  return m_startTime;
}

void 
MPTask::
SetReleaseWindow(const std::pair<double,double> _release) noexcept {
	m_releaseWindow = _release;
}
		
void
MPTask::SetDeadlineWindow(const std::pair<double,double> _deadline) noexcept {
	m_deadlineWindow = _deadline;
}
		
std::pair<double,double>
MPTask::
GetReleaseWindow() const noexcept {
	return m_releaseWindow;
}
		
std::pair<double,double>
MPTask::
GetDeadlineWindow() const noexcept {
	return m_deadlineWindow;
}

/*---------------------------- Constraint Evaluation -------------------------*/

bool
MPTask::
EvaluateCapability(const Robot* const _r) const {
  return m_capability.empty()
      or m_capability == _r->GetCapability();
}


bool
MPTask::
EvaluateStartConstraints(const Cfg& _cfg) const {
  return !m_startConstraint.get()
      or m_startConstraint->Satisfied(_cfg);
}


bool
MPTask::
EvaluatePathConstraints(const Cfg& _cfg) const {
  for(const auto& constraint : m_pathConstraints)
    if(!constraint->Satisfied(_cfg))
      return false;
  return true;
}


bool
MPTask::
EvaluateGoalConstraints(const Cfg& _cfg) const {
  if(m_goalConstraints.empty())
    return true;
  return EvaluateGoalConstraints(_cfg, m_goalConstraints.size() - 1);
}


bool
MPTask::
EvaluateGoalConstraints(const Cfg& _cfg, const size_t _index) const {
  try {
    const auto& constraint = m_goalConstraints.at(_index);
    return constraint->Satisfied(_cfg);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Request for goal constraint " << _index
                                  << " in a task with only "
                                  << m_goalConstraints.size() << " goals.";
  }
}

/*----------------------------------------------------------------------------*/
