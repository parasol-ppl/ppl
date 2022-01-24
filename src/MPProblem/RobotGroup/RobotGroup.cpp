#include "RobotGroup.h"

#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include <algorithm>
#include <sstream>


/*------------------------------- Construction -------------------------------*/

RobotGroup::
RobotGroup(MPProblem* const _problem, const std::string& _label,
    const std::vector<Robot*>& _robots)
  : m_problem(_problem), m_label(_label), m_robots(_robots) {
  for(size_t i = 0; i < m_robots.size(); ++i)
    m_indexes[m_robots[i]] = i;
}


RobotGroup::
RobotGroup(MPProblem* const _problem, XMLNode& _node) : m_problem(_problem) {
  // A robot group node should have a label and a set of child robots.
  m_label = _node.Read("label", true, "", "The group label.");

  const std::string robotLabelList = _node.Read("robotLabels", false, "",
                        "A space-delimited list of all robot labels in group.");

  // If there's not a single list, then check if there are child nodes:
  if(robotLabelList.empty()) {
    // Parse each child node.
    for(auto& child : _node) {
      std::string name = child.Name();
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);

      // Currently only "member" children are supported.
      if(name != "member")
        continue;

      const std::string label = child.Read("label", true, "", "The robot group "
                                                              "label.");
      Robot* const memberRobot = m_problem->GetRobot(label);
      m_robots.push_back(memberRobot);
    }
  }
  else {
    // Parse the space-delimited robot label list.
    std::istringstream labelStream(robotLabelList);
    std::string robotLabel;
    while(labelStream >> robotLabel) {
      Robot* const memberRobot = m_problem->GetRobot(robotLabel);
      m_robots.push_back(memberRobot);
    }
  }

  if(m_robots.empty())
    throw ParseException(_node.Where()) << "The robot group is empty.";

  for(size_t i = 0; i < m_robots.size(); ++i)
    m_indexes[m_robots[i]] = i;
}

/*----------------------------------- Accessors ------------------------------*/

std::string
RobotGroup::
GetLabel() const noexcept {
  return m_label;
}


Robot*
RobotGroup::
GetRobot(const size_t _index) const noexcept {
  try {
    return m_robots.at(_index);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Request for robot " << _index
                                  << " in a group with " << m_robots.size()
                                  << " robots.";
  }
}


Robot*
RobotGroup::
GetRobot(const std::string& _label) const noexcept {
  for(auto robot : m_robots)
    if(robot->GetLabel() == _label)
      return robot;
  return nullptr;
}


const std::vector<Robot*>&
RobotGroup::
GetRobots() const noexcept {
  return m_robots;
}


size_t
RobotGroup::
GetGroupIndex(Robot* const _robot) const noexcept {
  try {
    return m_indexes.at(_robot);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Request for index of robot " << _robot
                                  << " which is not in the group.";
  }
}


bool
RobotGroup::
VerifyRobotInGroup(Robot* const _robot) const noexcept {
  return m_indexes.count(_robot);
}


size_t
RobotGroup::
Size() const noexcept {
  return m_robots.size();
}


size_t
RobotGroup::
TotalDofs() const noexcept {
  size_t dofs = 0;
  for(Robot* const r : m_robots)
    dofs += r->GetMultiBody()->DOF();

  return dofs;
}

/*-------------------------------- Iteration ---------------------------------*/

RobotGroup::iterator
RobotGroup::
begin() noexcept {
  return m_robots.begin();
}


RobotGroup::iterator
RobotGroup::
end() noexcept {
  return m_robots.end();
}


RobotGroup::const_iterator
RobotGroup::
begin() const noexcept {
  return m_robots.begin();
}


RobotGroup::const_iterator
RobotGroup::
end() const noexcept {
  return m_robots.end();
}

/*----------------------------------- Debug ----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const RobotGroup& _g) {
  _os << "Group " << _g.GetLabel() << " has " << _g.Size() << " robots.";
  for(const auto robot : _g)
    _os << "\n\t" << robot->GetLabel();
  return _os << std::endl;
}

/*----------------------------------------------------------------------------*/
