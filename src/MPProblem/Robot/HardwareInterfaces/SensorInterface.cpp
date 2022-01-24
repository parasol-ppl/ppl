#include "SensorInterface.h"

#include "Utilities/PMPLExceptions.h"


/*------------------------------- Construction -------------------------------*/

SensorInterface::
~SensorInterface() = default;

/*--------------------------- Hardware Interface -----------------------------*/

HardwareInterface::HardwareType
SensorInterface::
GetHardwareType() const noexcept {
  return HardwareInterface::HardwareType::Sensor;
}

/*----------------------------- Sensor Interface -----------------------------*/

bool
SensorInterface::
IsReady() const noexcept {
  return m_ready;
}


size_t
SensorInterface::
GetLastTimestamp() const noexcept {
  return m_timestamp;
}


std::vector<mathtool::Transformation>
SensorInterface::
GetLastTransformations() {
  throw RunTimeException(WHERE) << "Not implemented.";
}


std::vector<std::vector<double>>
SensorInterface::
GetLastJointAngles() {
  throw RunTimeException(WHERE) << "Not implemented.";
}

/*----------------------------------------------------------------------------*/
