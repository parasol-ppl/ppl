#include "RobotCommandQueue.h"

#include "ActuatorInterface.h"
#include "SensorInterface.h"

#include "Utilities/XMLNode.h"


/*------------------------------- Construction -------------------------------*/

RobotCommandQueue::
RobotCommandQueue(XMLNode& _node) : m_period(0) {
  // Ignore this node when we aren't building the simulator.
  _node.Ignore();
}

/*----------------------------------------------------------------------------*/
