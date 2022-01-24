#include "DynamicObstacle.h"

#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/IOUtils.h"
#include "Utilities/MPUtils.h"


/*------------------------------- Construction -------------------------------*/

DynamicObstacle::
DynamicObstacle(Robot* const _robot, std::vector<Cfg> _path)
  : m_robot(_robot), m_path(_path) {
}


DynamicObstacle::
DynamicObstacle(XMLNode& _node, MPProblem* const _problem) {
  const std::string robotLabel = _node.Read("robot", true, "",
      "The robot which is acting as a dynamic obstacle.");
  m_robot = _problem->GetRobot(robotLabel);

  const std::string filePath = GetPathName(_node.Filename());
  const std::string obstacleFile = filePath + _node.Read("pathfile", true, "",
      "DynamicObstacle path file name");

  m_path = LoadPath(obstacleFile, m_robot);
}


DynamicObstacle::
~DynamicObstacle() = default;

/*-------------------------------- Accessors ---------------------------------*/

Robot*
DynamicObstacle::
GetRobot() const noexcept {
  return m_robot;
}


const std::vector<Cfg>
DynamicObstacle::
GetPath() const noexcept {
  return m_path;
}

const size_t 
DynamicObstacle::
GetStartTime() const noexcept {
  return m_startTime;
}
    
void 
DynamicObstacle::
SetStartTime(size_t _start) {
  m_startTime = _start;
}

/*----------------------------------------------------------------------------*/