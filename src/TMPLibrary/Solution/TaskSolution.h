#ifndef _PMPL_TASK_SOLUTION_H_
#define _PMPL_TASK_SOLUTION_H_

#include "MPLibrary/MPSolution.h"

#ifndef PPL_TEST_TRAITS_H_
  #include "Traits/CfgTraits.h"
#else
  #include "Traits/TestTraits.h"
#endif

class Robot;
class RobotGroup;
class SemanticTask;

class TaskSolution {
  public:
    ///@name Local Types
    ///@{

    typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>> MPSolution;

    ///@}
    ///@name Construction
    ///@{

    TaskSolution(SemanticTask* _task);

    ~TaskSolution();

    ///@}
    ///@name Accessors
    ///@{

    /// Get Task
    /// @return Semantic task.
    SemanticTask* GetTask();

    /// Set Robot
    /// @param  _robot Robot.
    void SetRobot(Robot* _robot);

    /// Get Robot
    /// @return Robot.
    Robot* GetRobot();

    /// Set Robot Group
    /// @param _group.
    void SetRobotGroup(RobotGroup* _group);

    /// Get Robot Group
    /// @return Robot group.
    RobotGroup* GetRobotGroup();

    /// Set Motions Solution
    /// @param _solution.
    void SetMotionSolution(MPSolution* _solution);

    /// Get Motions Solution
    /// @return Solution.
    MPSolution* GetMotionSolution();

    /// Set Start Time
    /// @param _startTime.
    void SetStartTime(double _startTime);

    /// Get Start Time
    /// @return Start time.
    double GetStartTime();

    ///@}
    ///@name Print
    ///@{

    void Print();

    ///@}

  private:
    ///@name Internal State
    ///@{

    SemanticTask* m_task;

    Robot* m_robot{nullptr};

    RobotGroup* m_robotGroup{nullptr};

    MPSolution* m_motionSolution{nullptr};

    double m_startTime{0};

    ///@}
};

#endif
