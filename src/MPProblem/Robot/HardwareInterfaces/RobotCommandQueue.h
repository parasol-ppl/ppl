#ifndef PMPL_ROBOT_COMMAND_QUEUE_H
#define PMPL_ROBOT_COMMAND_QUEUE_H

#include "Commands.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

class ActuatorInterface;
class SensorInterface;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// A command queue for controlling a hardware robot through one or more
/// hardware interfaces.
///
/// We expect all hardware robots to be able to set their actuators according to
/// each possible control in the software robot's dynamics model. If this is not
/// the case, then the actuator settings need to be adjusted to reflect the
/// physical robot's capabilities.
///
/// This object supports a queue of 'commands', where a command is (currently) a
/// set of controls executed for some length of time. The primary means of
/// controlling the hardware robot is thus manipulation of the command queue,
/// which allows us to set and execute commands asynchronously.
///
/// Currently this object supports up to one robot base and one sensor.
///
/// @note For now we are assuming the entire queue will use the longest
///       communication time for each component. We may wish to refine this to
///       check/use the specific times for individual components.
////////////////////////////////////////////////////////////////////////////////
class RobotCommandQueue final {

  public:

    ///@name Construction
    ///@{

    /// Create a robot command queue from an XML specification of the hardware
    /// settings.
    /// @param _node The XML node.
    RobotCommandQueue(XMLNode& _node);

    ~RobotCommandQueue();

    ///@}
    ///@name Hardware Properties
    ///@{

    /// Get the longest communication time for any piece of hardware.
    double GetCommunicationTime() const;

    /// Get the robot's base/actuator.
    ActuatorInterface* GetActuator() const noexcept;

    /// Get the robot's sensor.
    SensorInterface* GetSensor() const noexcept;

    ///@}
    ///@name Command Queue Manipulation
    ///@{

    /// Get the current command that the robot is executing.
    std::shared_ptr<Command> GetCurrentCommand() const;

    /// Add a command to the end of the robot's command queue.
    /// @param _command The command to execute.
    void EnqueueCommand(Command&& _command);

    /// Clear out the command queue.
    void ClearCommandQueue();

    /// Check if the robot is currently idle and has no queued commands. This
    /// will return false if you have ordered the robot to wait.
    bool IsIdle() const;

    /// Check if the robot has a localizing command currently on the queue.
    bool IsLocalizing() const;

    ///@}
    ///@name Robot Control
    ///@{

    /// Halt the hardware robot immediately. This should NOT touch the command
    /// queue in order to avoid deadlock conditions (do that separately).
    /// @return True if the hardware acknowledged the command. THIS DOES NOT
    ///         GUARANTEE THAT IT WORKED!
    bool FullStop();

    ///@}

  protected:

    ///@}
    ///@name Helpers
    ///@{

    /// Initialize the command queue and start it in a separate thread.
    void StartQueue();

    /// Stop the command queue and release its resources. Commands already in
    /// the queue will be cleared.
    void StopQueue();

    /// The queue function to run in a separate thread.
    void QueueFunction();

    /// Send a command to the robot. Must check for and recognize empty controls
    /// as 'wait' commands.
    /// @param _command Command to send to robot
    void SendToRobot(const Command& _command);

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::queue<std::shared_ptr<Command>> m_queue; ///< The command queue.

    double m_period{.01};        ///< The polling period.
    volatile bool m_idle{true};  ///< Is the robot presently idle?

    mutable std::atomic<bool> m_running; ///< Keep running the queue?
    std::thread m_thread;        ///< A thread for running the command queue.
    mutable std::mutex m_lock;   ///< Lock for changing the queue.

    std::unique_ptr<ActuatorInterface> m_base;  ///< Interface for the base.
    std::unique_ptr<SensorInterface> m_sensor;  ///< Interface for the sensor.

    std::atomic<bool> m_localizing{false}; ///< Is there a localization command in the queue?

    ///@}

};

#endif
