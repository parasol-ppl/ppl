#ifndef _PMPL_CHILD_AGENT_H_
#define _PMPL_CHILD_AGENT_H_

#include "PathFollowingAgent.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <queue>
#include <vector>

class Coordinator;

class ChildAgent : public PathFollowingAgent {
  public :
    ///@name Construction
    ///@{

    /// Create a child agent for a robot.
    /// @param _r The robot which this agent will reason for.
    ChildAgent(Robot* const _r);

    /// Create a child agent for a robot.
    /// @param _r The robot which this agent will reason for.
    /// @param _node The XML node to parse.
    ChildAgent(Robot* const _r, XMLNode& _node);

    /// Copy a child agent for another robot.
    /// @param _r The destination robot.
    /// @param _a The agent to copy.
    ChildAgent(Robot* const _r, const ChildAgent& _a);

    virtual ~ChildAgent();

    std::unique_ptr<Agent> Clone(Robot* const _r) const override;

    ///@}
    ///@name Simulation Interface
    ///@{

    virtual void Initialize() override;

    virtual void Step(const double _dt) override;

    virtual void Uninitialize() override;

    ///@}
    ///@name Child Interface
    ///@{

    /// Get a pointer to the coordinator of the current agent.
    /// @return A pointer to the agent's coordinator.
    Coordinator* GetCoordinator();

    /// Set m_parentAgent to an agent of the same capability.
    /// @param _parent The parent agent.
    void SetCoordinator(Coordinator* const _parent);

    ///@}
    ///@name Accessors
    ///@{

    /// Returns this agent's m_solution pointer
    MPSolution* GetMPSolution();

    ///@}
    ///@name Task Helpers
    ///@{

    virtual bool SelectTask() override;

    /// Evaluate the agent's progress on its current task.
    /// @return True if the current task should be continued, false otherwise.
    virtual bool EvaluateTask();

    /// Continue executing the agent's current task.
    /// @param _dt The step length.
    virtual void ExecuteTask(const double _dt) override;

    virtual void GeneratePlan() override;

    virtual void ClearPlan() override;
    ///@}
  private:

    ///@name Controller Helpers
    ///@{

    virtual void ExecuteControls(const ControlSet& _c, const size_t _steps) override;

    virtual void ExecuteControlsSimulation(const ControlSet& _c, const size_t _steps) override;

    virtual void ExecuteControlsHardware(const ControlSet& _c, const size_t _steps) override;

    ///@}
    ///@name Internal State
    ///@{

    /// The parent group to which this agent belongs.
    Coordinator* m_coordinator{nullptr};

    /*std::queue<std::pair<size_t,ControlSet>> m_controlQueue;

    mutable std::atomic<bool> m_running;

    std::thread m_thread;

    mutable std::mutex m_lock;   */

    std::string m_controlChannel;

    ControlSet m_queuedControlSet;
    size_t m_queuedSteps;
    bool m_locked{true};

    Cfg m_lastCfg;
    ///@}

};

#endif
