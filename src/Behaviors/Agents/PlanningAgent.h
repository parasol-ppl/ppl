#ifndef PMPL_PLANNING_AGENT_H_
#define PMPL_PLANNING_AGENT_H_

#include "Agent.h"

#include "MPLibrary/PMPL.h"

#include <memory>
#include <thread>

////////////////////////////////////////////////////////////////////////////////
/// Base class for agents which will plan paths using PPL. This agent only
/// plans; it does not attempt to execute the plan in the simulation.
///
/// The core behavior is:
/// 1. If no task, select one (or pass if none is available).
/// 2. If no plan, create one (in a separate thread, robot waits during planning).
/// 3. Continue executing current plan.
///
/// There are two primary mechanisms for directing agents of this type:
/// 1. The task: describes the agent's planning task/high level goal. Without a
///              task, the agent will not attempt to do anything.
/// 2. The plan: describes the agent's motion plan for achieving the task.
///              Clearing this without clearing the task will cause the agent to
///              replan.
///
/// Whenever the agent produces a new plan or clears an existing one, its 'plan
/// version' will be incremented by one. This allows coordinated agents to
/// detect changes in each others' plans and possibly react/replan in response.
/// @ingroup Agents
////////////////////////////////////////////////////////////////////////////////
class PlanningAgent : public Agent {

  public:

    ///@name Construction
    ///@{

    /// Create an agent for a robot.
    /// @param _r The robot which this agent will reason for.
    PlanningAgent(Robot* const _r);

    /// Copy an agent for another robot.
    /// @param _r The destination robot.
    /// @param _a The agent to copy.
    PlanningAgent(Robot* const _r, const PlanningAgent& _a);

    /// Create an agent for a robot.
    /// @param _r The robot which this agent will reason for.
    /// @param _node The XML node to parse.
    PlanningAgent(Robot* const _r, XMLNode& _node);

    virtual std::unique_ptr<Agent> Clone(Robot* const _r) const override;

    virtual ~PlanningAgent();

    ///@}
    ///@name Agent Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Step(const double _dt) override;

    virtual void Uninitialize() override;

    virtual void SetTask(std::shared_ptr<MPTask> const _task) override;

    ///@}
    ///@name Planning
    ///@{

    /// Does the agent have a plan for its current task?
    /// @return True if the agent has a plan.
    virtual bool HasPlan() const;

    /// Clear the agent's current plan. If a plan was cleared, the plan version
    /// will be updated. Overrides should always call this base method to update
    /// the plan version.
    virtual void ClearPlan();

    /// Is the agent currently generating a plan?
    /// @return True if the agent is generating a plan.
    bool IsPlanning() const;

    /// Get the current version number for the agent's plan.
    /// @return The plan version number.
    size_t GetPlanVersion() const;

    /// Get the pointer to this agents solution object.
    /// @return The MPSolution pointer.
    MPSolution* GetMPSolution();

    ///@}

  protected:

    ///@name Planning Helpers
    ///@{

    /// Generate a plan for the agent's current task. Calls the WorkFunction in
    /// a separate thread.
    virtual void GeneratePlan();

    /// Function call for PPL.
    virtual void WorkFunction(std::shared_ptr<MPProblem> _problem);

    /// Stop the current plan (only works for MPStrategies which are implemented
    /// using the base Run() method).
    void StopPlanning();

    ///@}
    ///@name Task Helpers
    ///@{

    /// Select the next task for this agent. The base implementation selects the
    /// first available task.
    /// @return True if a new task was selected, or false if none remain.
    virtual bool SelectTask();

    /// Evaluate the agent's progress on its current task.
    /// @return True if we should continue the current task, false otherwise.
    virtual bool EvaluateTask();

    /// Continue executing the agent's current task.
    /// @param _dt The step length.
    virtual void ExecuteTask(const double _dt);

    ///@}
    ///@name Localization Helpers
    ///@{

    /// Evaluates whether the robot's simulated state is incorrect (beyond some
    /// error threshold) and sets the simulated state as necessary.
    void UpdateSimulatedState();

    ///@}
    ///@name Internal State
    ///@{

    std::unique_ptr<MPLibrary> m_library;   ///< This agent's planning library.
    std::unique_ptr<MPSolution> m_solution; ///< The current solution.
    std::thread m_thread;                   ///< Thread for agent to run PPL.
    std::atomic<bool> m_planning{false};    ///< Is the agent currently planning.
    std::atomic<size_t> m_planVersion{1};   ///< The current plan version.

    size_t m_roadmapVisualID{size_t(-1)};   ///< The ID of the roadmap drawing.
    size_t m_pathVisualID{size_t(-1)};      ///< The ID of the path drawing.

    size_t m_localizePeriod{200};         ///< The number of timesteps between localize calls.
    size_t m_localizeCount{1};            ///< The number of timesteps since the last localize call.
    double m_localizeErrorThreshold{0.4}; ///< The largest allowed error before requiring a replan.

    ///@}

};

#endif
