#include "Commands.h"

#include "Utilities/PMPLExceptions.h"

#include "nonstd/numerics.h"


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Command ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

Command::
Command(const Command::Type _t, const double _seconds)
  : type(_t), seconds(_seconds)
{ }


Command::
~Command() = default;

/*-------------------------------- Downcasts ---------------------------------*/

MotionCommand&
Command::
ToMotionCommand() {
  const auto& self = *this;
  return const_cast<MotionCommand&>(self.ToMotionCommand());
}


const MotionCommand&
Command::
ToMotionCommand() const {
  if(type != Command::Type::Motion)
    throw RunTimeException(WHERE) << "Requested MotionCommand from a Command "
                                  << "with different type '" << type << "'.";
  return static_cast<const MotionCommand&>(*this);
}


SensorCommand&
Command::
ToSensorCommand() {
  const auto& self = *this;
  return const_cast<SensorCommand&>(self.ToSensorCommand());
}


const SensorCommand&
Command::
ToSensorCommand() const {
  if(type != Command::Type::Sensor)
    throw RunTimeException(WHERE) << "Requested SensorCommand from a Command "
                                  << "with different type '" << type << "'.";
  return static_cast<const SensorCommand&>(*this);
}

/*----------------------------------------------------------------------------*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MotionCommand ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

MotionCommand::
MotionCommand() : Command(Command::Type::Motion, 0)
{ }


MotionCommand::
MotionCommand(const ControlSet& _c, const double _seconds)
  : Command(Command::Type::Motion, _seconds), controls(_c)
{ }


std::unique_ptr<Command>
MotionCommand::
Clone() const {
  return std::unique_ptr<Command>(new MotionCommand(*this));
}


MotionCommand::
~MotionCommand() = default;

/*--------------------------------- Equality ---------------------------------*/

bool
MotionCommand::
operator==(const MotionCommand& _other) const noexcept {
  return controls == _other.controls
     and nonstd::approx(seconds, _other.seconds);
}


bool
MotionCommand::
operator!=(const MotionCommand& _other) const noexcept {
  return !(*this == _other);
}

/*----------------------------------------------------------------------------*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SensorCommand ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

SensorCommand::
SensorCommand(const double _seconds)
  : Command(Command::Type::Sensor, _seconds)
{ }


std::unique_ptr<Command>
SensorCommand::
Clone() const {
  return std::unique_ptr<Command>(new SensorCommand(*this));
}


SensorCommand::
~SensorCommand() = default;

/*----------------------------------------------------------------------------*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Debugging ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

std::ostream&
operator<<(std::ostream& _os, const Command::Type& _t) {
  switch(_t) {
    case Command::Type::Motion:
      return _os << "Motion";
    case Command::Type::Sensor:
      return _os << "Sensor";
    default:
      return _os << "Unknown";
  }
}

/*----------------------------------------------------------------------------*/
