#include "TaskSolution.h"

#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"
/*----------------------------------- Construction -----------------------------*/

TaskSolution::
TaskSolution(SemanticTask* _task) : m_task(_task) {}

TaskSolution::
~TaskSolution() {}

SemanticTask*
TaskSolution::
GetTask() {
  return m_task;
}

/*------------------------------------ Accessors ------------------------------*/

void
TaskSolution::
SetRobot(Robot* _robot) {
  m_robot = _robot;
}

Robot*
TaskSolution::
GetRobot() {
  return m_robot;
}

void
TaskSolution::
SetRobotGroup(RobotGroup* _group) {
  m_robotGroup = _group;
}

RobotGroup*
TaskSolution::
GetRobotGroup() {
  return m_robotGroup;
}

void
TaskSolution::
SetMotionSolution(MPSolution* _solution) {
  m_motionSolution = _solution;
}

TaskSolution::MPSolution*
TaskSolution::
GetMotionSolution() {
  return m_motionSolution;
}

void
TaskSolution::
SetStartTime(double _startTime) {
  m_startTime = _startTime;
}

double
TaskSolution::
GetStartTime() {
  return m_startTime;
}

/*--------------------------------------- Print ----------------------------*/
void
TaskSolution::
Print() {
  std::cout << "Solution for " << m_task->GetLabel() << std::endl;
  std::cout << "Allocated to ";
  if(m_robot) {
    std::cout << m_robot->GetLabel();
  }
  else if(m_robotGroup) {
    std::cout << m_robotGroup->GetLabel() << " (";
    for(auto r : m_robotGroup->GetRobots())
      std::cout << r->GetLabel() << ", ";
  }

  std::cout << std::endl << "Start time: " << m_startTime << std::endl;

  std::cout << "Path: ";
  if(m_robot) {
    for(auto vid : m_motionSolution->GetPath(m_robot)->VIDs()) {
      std::cout << vid << ", ";
    }
  }
  else if(m_robotGroup) {
    std::cout << "Robot Group Path Unsupported.";
  }
  std::cout << std::endl << std::endl;
}
