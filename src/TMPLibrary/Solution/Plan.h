#ifndef _PMPL_PLAN_H_
#define _PMPL_PLAN_H_

#include <list>
#include <memory>
#include <vector>
#include <unordered_map>

class Coordinator;
class Decomposition;
class MPProblem;
class Robot;
class RobotGroup;
class SemanticTask;
class StatClass;
class TaskSolution;

class Plan {
  public:
    ///@name Construction
    ///@{

    Plan();

    ~Plan();

    ///@}
    ///@name Accessors
    ///@{

    /// Coordinator
    void SetCoordinator(Coordinator* _coordinator);

    /// Get Coordinator
    Coordinator* GetCoordinator() const;

    /// Robot Team
    void SetTeam(std::vector<Robot*> _team);

    /// Get Robot Team
    const std::vector<Robot*>& GetTeam() const;

    /// Decomposition
    /// @param _decomp Decomposition.
    void SetDecomposition(Decomposition* _decomp);

    /// Get Decomposition
    Decomposition* GetDecomposition() const;

    /// Task Allocations
    /// @param _robot Robot.
    void ClearAllocations(Robot* _robot);

    /// Task Allocations
    /// @param _group Group.
    void ClearAllocations(RobotGroup* _group);

    /// Add Task Allocations
    /// @param _robot Robot.
    /// @param _task Semantic task.
    void AddAllocation(Robot* _robot, SemanticTask* _task);

    /// Add Task Allocations
    /// @param _robot Robot.
    /// @param _task Semantic task.
    void AddAllocation(RobotGroup* _group, SemanticTask* _task);

    /// Get Allocations
    /// @param _robot Robot.
    std::list<SemanticTask*> GetAllocations(Robot* _robot);

    /// Get Allocations
    /// @param _group Robot group.
    std::list<SemanticTask*> GetAllocations(RobotGroup* _group);

    /// Get Task Solutions
    const std::unordered_map<SemanticTask*,std::shared_ptr<TaskSolution>>& GetTaskSolutions();

    /// Task Plans
    /// @param _task Task.
    /// @param _solution Solution.
    void SetTaskSolution(SemanticTask* _task, std::shared_ptr<TaskSolution> _solution);

    /// Get Task Solutions
    /// @param _task Task.
    TaskSolution* GetTaskSolution(SemanticTask* _task);

    /// Get Task Solutions of a robot
    /// @param _robot Robot.
    std::vector<TaskSolution*> GetRobotTaskSolutions(Robot* _robot);

    /// MPProblem
    /// @param _problem MPProblem.
    void SetMPProblem(MPProblem* _problem);

    /// Get MPProblem
    MPProblem* GetMPProblem();

    /// StatClass
    StatClass* GetStatClass();

    ///@}
    ///@name Print
    ///@{

    void Print();

    ///@}

  private:
    ///@name Internal State
    ///@{

    Coordinator* m_coordinator{nullptr};

    std::vector<Robot*> m_team;

    Decomposition* m_decomposition{nullptr};

    std::unordered_map<Robot*,std::list<SemanticTask*>> m_allocations;

    std::unordered_map<RobotGroup*,std::list<SemanticTask*>> m_groupAllocations;

    std::unordered_map<SemanticTask*,std::shared_ptr<TaskSolution>> m_taskSolutions;

    std::unique_ptr<StatClass> m_statClass;

    MPProblem* m_problem{nullptr};

    ///@}

};

#endif
