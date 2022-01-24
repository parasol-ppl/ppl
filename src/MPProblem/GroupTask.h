#ifndef PMPL_GROUP_TASK_H_
#define PMPL_GROUP_TASK_H_

#include "MPProblem/MPTask.h"

#include "nonstd/status.h"

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class Boundary;
class GroupCfg;
class Constraint;
class MPProblem;
class Robot;
class RobotGroup;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Describes a motion task for a group of robots as a set of individual tasks.
///
/// The individual tasks may be assigned to specific robots in the group, or
/// they may be completable by any robot in the group.
///
/// @note The goals are 'right-aligned' in that we assume that all robots must
///       reach their last goal together.
///
/// @note Assigning a group task to another group will clear any assignments
///       of robots to individual tasks because there is no universal way to map
///       the new robots to the same set of tasks. The new group may be
///       smaller/larger, have different labels/capabilities, etc.
///
/// @todo Move disassembly-specific items to a derived class specifically
///       for disassembly tasks.
////////////////////////////////////////////////////////////////////////////////
class GroupTask {

  public:

    ///@name Local Types
    ///@{

    /// A set of individual tasks.
    typedef std::vector<MPTask>     TaskSet;

    typedef TaskSet::iterator       iterator;
    typedef TaskSet::const_iterator const_iterator;

    ///@}
    ///@name Construction
    ///@{

    /// Create an empty task for a given robot.
    /// @param _robotGroup The robot group assigned to this task.
    explicit GroupTask(RobotGroup* const _robotGroup);

    /// Parse the set of task constraints described in an XML node
    /// and verify that tasks are assigned to robots in group.
    /// @param _problem The MPProblem for this task.
    /// @param _node The XML node to parse.
    explicit GroupTask(MPProblem* const _problem, XMLNode& _node);

    virtual ~GroupTask();

    /// Generate a group task of the appropriate type.
    /// @param _problem The MPProblem for this task.
    /// @param _node The XML node to parse.
    static std::unique_ptr<GroupTask> Factory(MPProblem* const _problem,
        XMLNode& _node);

    ///@}
    ///@name Property Accessors
    ///@{

    /// Get the robot associated with this task.
    RobotGroup* GetRobotGroup() const noexcept;

    /// Assign this task to another robot group. If the new group is different,
    /// all individual tasks will have their robot pointers cleared (since there
    /// is no universal means of mapping the individual tasks to the new robots).
    /// @param _r The destination robot which will receive this assignment.
    void SetRobotGroup(RobotGroup* const _r);

    /// Get the semantic label for this task.
    const std::string& GetLabel() const noexcept;

    /// Set the semantic label for this task.
    void SetLabel(const std::string& _label) noexcept;

    /// Get the number of individual tasks within this group task.
    size_t Size() const noexcept;

    /// Check if there are no individual tasks.
    /// @return False if no tasks or all tasks empty
    bool Empty() const noexcept;

    /// Get the status object for this task.
    nonstd::status& GetStatus() noexcept;
    const nonstd::status& GetStatus() const noexcept;

    ///@}
    ///@name Individual Tasks
    ///@{
    /// Access, modify, and iterate over the individual tasks.

    /// Get the number of goals in the longest (most goals) task.
    size_t GetNumGoals() const noexcept;

    /// Add an individual task.
    /// @param _t The individual task to add.
    void AddTask(const MPTask& _t);

    /// Remove an individual task.
    /// @param _iter An iterator to the task to remove.
    /// @return An iterator to the following element.
    iterator RemoveTask(iterator _iter);

    iterator begin() noexcept;
    iterator end() noexcept;

    /// Get iterator to beginning of tasks
    const_iterator begin() const noexcept;
    /// Get iterator to end of tasks
    const_iterator end() const noexcept;

    ///@}
    ///@name Constraint Accessors
    ///@{

    /// Uses the robot group in _center to populate all of the individual cfgs
    /// in that group cfg.
    /// @todo This is a hack that was needed to move the disassembly work
    ///       forward before we had flushed out the tasks/groups. To be removed
    ///       at the earliest opportunity.
    /// @param _center Robot group to populate from
    void GetStartConstraintCenter(GroupCfg& _center) const noexcept;

    ///@}
    ///@name Constraint Evaluation
    ///@{

    /// Evaluate whether a configuration satisfies the start constraints.
    /// @param _cfg The configuration to check.
    /// @return True if each robot satsifies its individual start constraints at
    ///         _cfg.
    bool EvaluateStartConstraints(const GroupCfg& _cfg) const;

    /// Evaluate whether a configuration satisfies the path constraints.
    /// @param _cfg The configuration to check.
    /// @return True if each robot satsifies its individual path constraints at
    ///         _cfg.
    bool EvaluatePathConstraints(const GroupCfg& _cfg) const;

    /// Evaluate whether a configuration satisfies the final goal constraints.
    /// @param _cfg The configuration to check.
    /// @return True if each robot satsifies its last individual goal constraints
    ///         at _cfg.
    bool EvaluateGoalConstraints(const GroupCfg& _cfg) const;

    /// Evaluate whether a configuration satisfies the constraints for a
    /// designated goal.
    /// @param _cfg The configuration to check.
    /// @param _index The goal index to check.
    /// @return True if each robot satsifies its individual goal constraints at
    ///         _cfg for goal _index.
    bool EvaluateGoalConstraints(const GroupCfg& _cfg, const size_t _index) const;

    ///@}
    ///@name Disassembly Items
    ///@{
    /// To be moved to a specialized derived class.

    /// Get the optional manipulator robot group associated with this task.
    RobotGroup* GetEndEffectorGroup() const noexcept;

    /// Get the optional manipulator robot group associated with this task.
    RobotGroup* GetManipulatorGroup() const noexcept;

    /// Get the robot pointer for the (optional) end effector for the group task
    Robot* GetEndEffectorRobot();

    /// Get the robot pointer for the (optional) manipulator for this group task
    Robot* GetManipulatorRobot();

    ///@}

  private:

    ///@name Internal State
    ///@{

    RobotGroup* m_group{nullptr}; ///< The robot group assigned to this task.

    std::string m_label;          ///< The task's semantic label.
    nonstd::status m_status;      ///< The status of the group task.

    TaskSet m_individualTasks;    ///< The individual tasks.

    ///@}
    ///@name Disassembly Items
    ///@{
    /// To be moved to a specialized derived class.

    // Used in disassembly planning, this group should contain the assembly
    // along with the manipulator's end effector. Duplicates a little bit of
    // data, but allows us to most extensibly use group path/cfg output.
    RobotGroup* m_endEffectorGroup{nullptr}; ///< The optional effector group

    // Used in disassembly planning, this group should contain the group along
    // with the manipulator. Duplicates a little bit of data, but allows us to
    // most extensibly use group path/cfg output.
    RobotGroup* m_manipulatorGroup{nullptr}; ///< The optional manipulator group

    ///@}

};

#endif
