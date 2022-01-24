#include "ActuatorInterface.h"


/*------------------------------- Construction -------------------------------*/

ActuatorInterface::
~ActuatorInterface() = default;

/*--------------------------- Hardware Interface -----------------------------*/

HardwareInterface::HardwareType
ActuatorInterface::
GetHardwareType() const noexcept {
  return HardwareInterface::HardwareType::Actuator;
}

/*----------------------------------------------------------------------------*/
