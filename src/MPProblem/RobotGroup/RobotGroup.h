#ifndef PMPL_ROBOT_GROUP_H_
#define PMPL_ROBOT_GROUP_H_

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>

class MPProblem;
class Robot;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// A group of one or more robots.
////////////////////////////////////////////////////////////////////////////////
class RobotGroup final {

  public:

    ///@name Local Types
    ///@{

    typedef std::vector<Robot*>::iterator       iterator;
    typedef std::vector<Robot*>::const_iterator const_iterator;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a group from a set of robots.
    /// @param _problem The owning MPProblem.
    /// @param _label The group label.
    /// @param _robot The set of robots in this group.
    RobotGroup(MPProblem* const _problem, const std::string& _label,
        const std::vector<Robot*>& _robots);

    /// Construct a group from an XML node. The referenced robots must be
    /// specified before the group in the XML file.
    /// @param _problem The owning MPProblem.
    /// @param _node The XML node.
    RobotGroup(MPProblem* const _problem, XMLNode& _node);

    ///@}
    ///@name Accessors
    ///@{

    /// Get the group label.
    std::string GetLabel() const noexcept;

    /// Get a robot by index.
    Robot* GetRobot(const size_t _index) const noexcept;

    /// Get a robot by label.
    Robot* GetRobot(const std::string& _label) const noexcept;

    /// Get all robots
    const std::vector<Robot*>& GetRobots() const noexcept;

    /// Get the group index for a robot.
    /// @note This is NOT the index in the MPProblem.
    size_t GetGroupIndex(Robot* const _robot) const noexcept;

    /// Check whether the robot is in the group or not.
    bool VerifyRobotInGroup(Robot* const _robot) const noexcept;

    /// Get the number of robots in the group.
    size_t Size() const noexcept;

    /// Get the number of DOFs accumulated over all robots.
    size_t TotalDofs() const noexcept;

    ///@}
    ///@name Iteration
    ///@{
    /// Iterate over the robots in the group.

    iterator begin() noexcept;
    iterator end() noexcept;

    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;

    ///@}

  private:

    ///@name Internal State
    ///@{

    MPProblem* const m_problem;   ///< The owning problem.
    std::string m_label;          ///< The group label.

    std::vector<Robot*> m_robots; ///< The robots in this group.
    std::unordered_map<Robot*, size_t> m_indexes; ///< The group index map.

    ///@}

};

/*----------------------------------- Debug ----------------------------------*/

std::ostream& operator<<(std::ostream&, const RobotGroup&);

/*----------------------------------------------------------------------------*/

#endif
