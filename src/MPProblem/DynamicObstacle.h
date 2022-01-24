#ifndef PMPL_DYNAMIC_OBSTACLE_H_
#define PMPL_DYNAMIC_OBSTACLE_H_

#include <memory>
#include <vector>

#include "ConfigurationSpace/Cfg.h"
#include "Utilities/XMLNode.h"

class MPProblem;
class Robot;


////////////////////////////////////////////////////////////////////////////////
/// A model of a dynamic obstacle with a known trajectory.
///
/// The obstacle body and motion properties are modeled as a robot. The
/// trajectory is interpreted as a sequence of configurations which the obstacle
/// will assume starting from a specified start time
/// and continuing at a rate of one Cfg per time step.
////////////////////////////////////////////////////////////////////////////////
class DynamicObstacle {

  public:

    ///@name Construction
    ///@{

    /// Construct a dynamic obstacle.
    /// @param _robot The robot model for the obstacle.
    /// @param _path The obstacle's known trajectory.
    DynamicObstacle(Robot* const _robot, std::vector<Cfg> _path);

    /// Construct a dynamic obstacle from an XML node.
    /// @param _node The XML node to parse.
    /// @param _problem The owning problem.
    DynamicObstacle(XMLNode& _node, MPProblem* const _problem);

    ~DynamicObstacle();

    ///@}
    ///@name Accessors
    ///@{

    /// Get the obstacle's robot model.
    /// @return The robot model
    Robot* GetRobot() const noexcept;

    /// Get the obstacle's path.
    /// @return The obstacle's path
    const std::vector<Cfg> GetPath() const noexcept;

    /// Get the obstacle start time.
    /// @return The obstacle's start time
    const size_t GetStartTime() const noexcept;

    /// Set the obstacle start time.
    void SetStartTime(size_t _start);

    ///@}

  private:

    ///@name Internal State
    ///@{

    Robot* m_robot{nullptr}; ///< The obstacle robot.
    std::vector<Cfg> m_path; ///< Assuming 1 cfg per time resolution.
    size_t m_startTime{0}; ///< Time that the path starts at.

    ///@}

};

#endif