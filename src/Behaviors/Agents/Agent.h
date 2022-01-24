#ifndef AGENT_H_
#define AGENT_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "MPProblem/Robot/Control.h"

class Cfg;
class MPTask;
class Robot;
class StepFunction;
class XMLNode;

////////////////////////////////////////////////////////////////////////////////
/// The decision-making faculties of a robot.
///
/// @details Agents are used with simulated robots. On each time step of the
///          simulation, the agent decides what the robot will do and sends an
///          appropriate command to the controller.
////////////////////////////////////////////////////////////////////////////////
class Agent {

  protected:

    ///@name Internal State
    ///@{

    Robot* const m_robot;              ///< The robot that this agent controls.

    mutable bool m_initialized{false}; ///< Is the agent initialized?

    std::shared_ptr<MPTask> m_task;    ///< The task this agent is working on.

    ControlSet m_currentControls;      ///< The current control set.
    size_t m_stepsRemaining{0};        ///< Steps remaining on current controls.

    bool m_debug{true};                ///< Toggle debug messages.

    /// Specifies the type of agent for heterogenous multiagent teams.
    std::string m_capability;

    ///< Step function to define agent behaviors.
    std::unique_ptr<StepFunction> m_stepFunction;

    ///@}

  public:

    ///@name Construction
    ///@{

    /// Create an agent for a robot.
    /// @param _r The robot which this agent will reason for.
    Agent(Robot* const _r);

    /// Create an agent for a robot.
    /// @param _r The robot which this agent will reason for.
    /// @param _node The XML node to parse.
    Agent(Robot* const _r, XMLNode& _node);

    /// Copy an agent for another robot.
    /// @param _r The destination robot.
    /// @param _a The agent to copy.
    Agent(Robot* const _r, const Agent& _a);

    /// Create a dynamically-allocated agent from an XML node.
    /// @param _r The robot which this agent will reason for.
    /// @param _node The XML node to parse.
    /// @return An agent of the type specified by _node.
    static std::unique_ptr<Agent> Factory(Robot* const _r, XMLNode& _node);

    /// Create a copy of this agent for another robot. This is provided so that
    /// we can copy an agent without knowing its type.
    /// @param _r The robot which this cloned agent will reason for.
    /// @return A copy of the agent.
    virtual std::unique_ptr<Agent> Clone(Robot* const _r) const = 0;

    virtual ~Agent();

    ///@}
    ///@name Agent Properties
    ///@{

    /// Get the robot object to which this agent belongs.
    Robot* GetRobot() const noexcept;

    /// Is this agent a child of some group/aggregate?
    virtual bool IsChild() const noexcept;

    ///@}
    ///@name Task Management
    ///@{

    /// Set the task for this agent.
    /// @param _task The new task for this agent. Should be owned by the
    ///              MPProblem.
    virtual void SetTask(const std::shared_ptr<MPTask> _task);

    /// Get the task that the agent is currently working on.
    std::shared_ptr<MPTask> GetTask() const noexcept;

    /// Resets the start constraint of the current task to the robot's current
    /// position.
    virtual void ResetStartConstraint();

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Set up the agent before running. Anything that needs to be done only once
    /// on first starting should go here.
    virtual void Initialize() = 0;

    /// Decide what to do on each time step in the simulation. The agent should
    /// implement its decision by sending commands to the robot's controller.
    /// @param _dt The timestep length.
    virtual void Step(const double _dt) = 0;

    /// Tear down the agent. Release any resources and reset the object to its
    /// pre-initialize state.
    virtual void Uninitialize() = 0;

    /// Find the smallest time interval which is an integer multiple of the
    /// problem time resolution and larger than the hardware time (if any).
    size_t MinimumSteps() const;

    /// Check for proximity to other robots and return those that lie within
    /// some threshold.
    /// @WARNING This checks the distance between the robots' reference points;
    ///          it does not indicate the minimum distance between their hulls.
    /// @param _distance The distance threshold.
    /// @return The vector of Robots within the threshold.
    std::vector<Agent*> ProximityCheck(const double _distance) const;

    ///@}
    ///@name Agent Control
    ///@{

    /// Stop the robot in simulation (places 0s in all 6 velocity dofs).
    /// @WARNING Arbitrarily setting the velocity does not respect the robot's
    ///          dynamics. It is suitable for debugging and freezing a scenario upon
    ///          completion, but it is not physically realistic.
    void Halt();

    /// Orders the agent to stop itself at its current position. It will ask the
    /// controller to choose actions which stay as close as possible to the
    /// current position.
    /// @param _steps The number of steps to stop for.
    void PauseAgent(const size_t _steps);

    ///@}
    ///@name Accessors
    ///@{

    /// Get the type of agent
    const std::string& GetCapability() const noexcept;

    ///@}
  protected:

    /// Instruct the agent to enqueue a command for gathering sensor readings.
    void Localize();

    /// Is the agent waiting on sensor data?
    bool IsLocalizing() const noexcept;

    /// Estimate the state of the robot from the last localization data.
    Cfg EstimateState();

    /// Continue executing the last controls if time remains.
    /// @return True if time is left on the last controls, false if
    ///         the agent is finished and needs new controls.
    bool ContinueLastControls();

    /// Execute a set of controls on the simulated robot, and additionally on the hardware if
    /// present.
    /// @param _c The controls to execute.
    /// @param _steps The number of time steps to execute the control.
    virtual void ExecuteControls(const ControlSet& _c, const size_t _steps);

    /// Execute a set of controls on the simulated robot.
    /// @param _c The controls to execute.
    virtual void ExecuteControlsSimulation(const ControlSet& _c, const size_t _steps);

    /// Execute a set of controls on the hardware if present.
    /// @param _c The controls to execute.
    /// @param _steps The number of time steps to execute the control.
    virtual void ExecuteControlsHardware(const ControlSet& _c, const size_t _steps);

    ///@}
  private:

    ///@name Disabled Functions
    ///@{
    /// Regular copy/move is disabled because each agent must be created for a
    /// specific robot object.

    Agent(const Agent&) = delete;
    Agent(Agent&&)      = delete;

    Agent& operator=(const Agent&) = delete;
    Agent& operator=(Agent&&)      = delete;

    ///@}

};

#endif
