#include "RobotCommandQueue.h"

#include "ActuatorInterface.h"
#include "SensorInterface.h"

#include "Utilities/XMLNode.h"


/*------------------------------- Construction -------------------------------*/

RobotCommandQueue::
RobotCommandQueue(XMLNode& _node) {
  // Read the child nodes to setup the hardware components.
  for(auto& child : _node) {
    if(child.Name() == "HardwareInterface") {
      auto hardware = HardwareInterface::Factory(child);

      switch(hardware->GetHardwareType()) {
        case HardwareInterface::Actuator:
          m_base = std::unique_ptr<ActuatorInterface>(
              static_cast<ActuatorInterface*>(hardware.release()));
          break;
        case HardwareInterface::Sensor:
          m_sensor = std::unique_ptr<SensorInterface>(
              static_cast<SensorInterface*>(hardware.release()));
          break;
      }
    }
  }

  // Set the queue period to the maximum communication time..
  m_period = GetCommunicationTime();

  StartQueue();
}

/*----------------------------------------------------------------------------*/
