#ifndef PMPL_COMMANDS_H
#define PMPL_COMMANDS_H

#include "MPProblem/Robot/Control.h"

#include <memory>

struct MotionCommand;
struct SensorCommand;


////////////////////////////////////////////////////////////////////////////////
/// A command of unspecified type. This serves as an interface class for the
/// RobotCommandQueue.
////////////////////////////////////////////////////////////////////////////////
class Command {

  public:

    ///@name Local Types
    ///@{

    enum class Type {Motion, Sensor};

    ///@}
    ///@name Internal State
    ///@{

    const Type type; ///< The command type.
    double seconds;  ///< The command duration.

    ///@}
    ///@name Construction
    ///@{

    /// Construct a command.
    /// @param _t The command type.
    /// @param _seconds The number of seconds required to execute this.
    Command(const Type _t, const double _seconds);

    /// Dynamic-copy a derived command type.
    virtual std::unique_ptr<Command> Clone() const = 0;

    virtual ~Command() = 0;

    ///@}
    ///@name Downcasts
    ///@{
    /// Downcast this generic command to the appropriate object type.

    /// Downcast to a MotionCommand. Throws if this has any other command type.
    MotionCommand& ToMotionCommand();
    const MotionCommand& ToMotionCommand() const;

    /// Downcast to a SensorCommand. Throws if this has any other command type.
    SensorCommand& ToSensorCommand();
    const SensorCommand& ToSensorCommand() const;

    ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// A motion command, consisting of a control and length of time to execute it.
////////////////////////////////////////////////////////////////////////////////
struct MotionCommand : public Command
{

  ///@name Internal State
  ///@{

  ControlSet controls; ///< The controls.

  ///@}
  ///@name Construction
  ///@{

  MotionCommand();

  /// @param _c Set of controls for the motion command
  /// @param _seconds The control duration
  MotionCommand(const ControlSet& _c, const double _seconds);

  virtual std::unique_ptr<Command> Clone() const override;

  virtual ~MotionCommand();

  ///@}
  ///@name Equality
  ///@{

  /// Define an equivalence relation.
  /// @return True if both commands have the same control sets and
  ///         near-equal durations (within a small tolerance).
  bool operator==(const MotionCommand& _other) const noexcept;

  bool operator!=(const MotionCommand& _other) const noexcept;

  ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// A sensor command, which is basically a one-time signal to take a
/// measurement and send back the results.
///
/// @note For now we have assumed a very simple sensor model, which will likely
///       be refined as we need support for more advanced uses and sensors.
///       I.e., it will eventually be desirable to have separate functions for
///       taking measurements and receiving the results.
////////////////////////////////////////////////////////////////////////////////
struct SensorCommand : public Command {

  ///@name Construction
  ///@{

  /// @param _seconds The command duration
  SensorCommand(const double _seconds = 0);

  virtual std::unique_ptr<Command> Clone() const override;

  virtual ~SensorCommand();

  ///@}

};


// Debugging.
std::ostream& operator<<(std::ostream&, const Command::Type&);

#endif
