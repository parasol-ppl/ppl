#include "RobotCommandQueue.h"

#include "ActuatorInterface.h"
#include "SensorInterface.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include "nonstd/io.h"
#include "nonstd/numerics.h"
#include "nonstd/timer.h"

#include <algorithm>
#include <unistd.h>


/*------------------------------- Construction -------------------------------*/

RobotCommandQueue::
~RobotCommandQueue() {
  // Stop the command queue and empty it.
  StopQueue();
  ClearCommandQueue();

  // Halt the robot.
  FullStop();

  // Individual hardware components will be taken down by their destructors.
}

/*-------------------------- Hardware Properties -----------------------------*/

double
RobotCommandQueue::
GetCommunicationTime() const {
  return std::max(m_base.get()   ? m_base->GetCommunicationTime()   : 0,
                  m_sensor.get() ? m_sensor->GetCommunicationTime() : 0);
}


ActuatorInterface*
RobotCommandQueue::
GetActuator() const noexcept {
  return m_base.get();
}


SensorInterface*
RobotCommandQueue::
GetSensor() const noexcept {
  return m_sensor.get();
}

/*------------------------------ Command Queue -------------------------------*/

std::shared_ptr<Command>
RobotCommandQueue::
GetCurrentCommand() const {
  std::lock_guard<std::mutex> guard(m_lock);
  return m_queue.front();
}


void
RobotCommandQueue::
EnqueueCommand(Command&& _command) {
  // Make sure the command duration is at least equal to the polling period.
  _command.seconds = std::max(m_period, _command.seconds);

  if(_command.type == Command::Type::Sensor)
    m_localizing = true;
  std::lock_guard<std::mutex> guard(m_lock);
  m_queue.emplace(_command.Clone());
}


void
RobotCommandQueue::
ClearCommandQueue() {
  std::lock_guard<std::mutex> guard(m_lock);
  while(!m_queue.empty())
    m_queue.pop();
}


bool
RobotCommandQueue::
IsIdle() const {
  std::lock_guard<std::mutex> guard(m_lock);
  return m_idle;
}

bool
RobotCommandQueue::
IsLocalizing() const {
  return m_localizing;
}

/*----------------------------- Robot Control --------------------------------*/

bool
RobotCommandQueue::
FullStop() {
  std::cout << "Calling full stop." << std::endl;
  return m_base->FullStop();
}

/*--------------------------------- Helpers ----------------------------------*/

void
RobotCommandQueue::
StartQueue() {
  // Create a functor to execute queued commands. While the queue is empty, this
  // will wait for 100 ms before checking again.

  // Start the queue thread.
  m_running = true;
  auto workFunction = [this](){this->QueueFunction();};
  m_thread = std::thread(workFunction);
}


void
RobotCommandQueue::
StopQueue() {
  std::lock_guard<std::mutex> guard(m_lock);

  // Stop the queue thread and wait for it to join.
  m_running = false;
  if(m_thread.joinable())
    m_thread.join();

  // We must wait for the thread to stop before calling full stop or we might
  // send the command too early (could be ignored by robot if it is busy
  // receiving the previous command).
  FullStop();
}


void
RobotCommandQueue::
QueueFunction() {
  // Define the smallest change in time that we will consider. This serves as
  // both an equality threshold and an amount of time for sleep cycles.
  constexpr double deltaT = 1e-4;

  // Create clocks to track how long we've been executing the current command
  // and how long the current polling cycle is taking.
  nonstd::timer executeClock, cycleClock;

  // We only need to send commands once; keep track of whether we have already
  // sent the current command.
  bool currentCommandSent = false;

  while(m_running) {
    // Lock the queue while we check it and pull the next command.
    m_lock.lock();
    cycleClock.restart();

    // If there is no command to execute, create a 'wait' command for the
    // next period.
    if(m_queue.empty()) {
      m_idle = true;
      m_lock.unlock();

      SendToRobot(MotionCommand(ControlSet(), m_period));

      while(cycleClock.elapsed() > m_period)
        std::this_thread::sleep_for(std::chrono::duration<double>(deltaT));
      continue;
    }

    // If we're still here, there is a command on the queue. Determine how much
    // longer it should be executed.
    auto& currentCommand = *m_queue.front().get();
    const double executeTime = currentCommand.seconds - executeClock.elapsed();

    // If there is no more time on the current command, remove it and move on.
    if(executeTime <= deltaT) {
      executeClock.reset();
      m_queue.pop();
      m_lock.unlock();
      currentCommandSent = false;
      continue;
    }

    // Determine how long we will sleep while partially executing the current
    // command.
    double sleepTime = m_period;

    // If the current command is shorter than the polling period, we will try
    // to execute it anyway but there may be some issues.
    if(executeTime < m_period and !currentCommandSent) {
      std::cerr << "\nRobotCommandQueue:: WARNING! Current command has "
                << "duration " << executeTime << " remaining, which is less than "
                << "the polling period " << m_period << "! "
                << "Executing anyway, but weird behavior may result."
                << std::endl;
      sleepTime = executeTime;
    }
    // If there are less than two whole periods left on the command, execute
    // them together to avoid the above problem.
    else if(executeTime < 2 * m_period) {
      sleepTime = executeTime;
    }

    // Send the command to the robot if we have not already done so.
    if(!currentCommandSent) {
      currentCommandSent = true;
      m_idle = false;
      SendToRobot(currentCommand);
      executeClock.start();
      cycleClock.restart();
    }

    m_lock.unlock();

    // Sleep until at least one polling cycle has elasped.
    while(cycleClock.elapsed() > sleepTime)
      std::this_thread::sleep_for(std::chrono::duration<double>(deltaT));
  }
}


void
RobotCommandQueue::
SendToRobot(const Command& _command) {
  switch(_command.type) {
    case Command::Type::Motion:
      if(m_base.get())
        m_base->SendCommand(_command.ToMotionCommand());
      return;
    case Command::Type::Sensor:
      if(m_sensor.get())
        m_sensor->SendCommand(_command.ToSensorCommand());
      m_localizing = false;
      return;
  }
}

/*----------------------------------------------------------------------------*/
