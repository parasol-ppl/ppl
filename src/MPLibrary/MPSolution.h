#ifndef PMPL_MP_SOLUTION_TYPE_H_
#define PMPL_MP_SOLUTION_TYPE_H_

#include "ConfigurationSpace/LocalObstacleMap.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "Utilities/MetricUtils.h"

#include <map>
#include <memory>
#include <unordered_map>

class Robot;


////////////////////////////////////////////////////////////////////////////////
/// Container for the output of a planning algorithm. Includes free and blocked
/// roadmaps, a path, and a local obstacle map for each robot. Also includes a
/// stat class for performance tracking.
///
/// @todo Currently this object can represent a solution for each single robot
///       and several robot group. It can almost support multiple of each - it
///       just needs an interface for adding more robots/groups to the container.
///
/// @note This object makes only one stat class, which is shared across all
///       uses.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPSolutionType final {

  public:

    ///@name Solution Object Types
    ///@{

    typedef typename MPTraits::Path             Path;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename MPTraits::LocalObstacleMap LocalObstacleMap;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::GroupPathType    GroupPathType;

    /// The outputs for an individual robot.
    struct RobotSolution {
      std::unique_ptr<RoadmapType>  freeMap; ///< The free-space roadmap.
      std::unique_ptr<RoadmapType>  obstMap; ///< The obstacle-space roadmap.
      std::unique_ptr<Path>            path; ///< The current solution path.
      std::unique_ptr<LocalObstacleMap> lom; ///< The local obstacle map.

      RobotSolution() = default;

      /// Initialize a solution for a single robot.
      /// @param _robot The robot.
      /// @param _stats The stats object for timing.
      RobotSolution(Robot* const _robot, StatClass* const _stats);
    };

    /// The outputs for a robot group.
    struct GroupSolution {
      std::unique_ptr<GroupRoadmapType>  freeMap; ///< The free-space roadmap.
      std::unique_ptr<GroupRoadmapType>  obstMap; ///< The obstacle-space roadmap.
      std::unique_ptr<GroupPathType>        path; ///< The current solution path.

      GroupSolution() = default;

      /// Initialize a solution for a robot group.
      /// @param _group The robot group.
      GroupSolution(RobotGroup* const _group, MPSolutionType* const _solution);
    };

    ///@}
    ///@name Construction
    ///@{

    MPSolutionType(Robot* const _r);

    MPSolutionType(RobotGroup* const _g);

    ///@}
    ///@name Modifiers
    ///@{

    void AddRobot(Robot* const _r) noexcept;

    void SetRobot(Robot* const _r) noexcept;

    void SetRoadmap(Robot* const _r, RoadmapType* _roadmap) noexcept;

    void SetPath(Robot* const _r, Path* _path) noexcept;

    void AddRobotGroup(RobotGroup* const _g) noexcept;

    void SetGroupRoadmap(RobotGroup* const _g, GroupRoadmapType* _roadmap) noexcept;

    void SetGroupPath(RobotGroup* const _r, GroupPathType* _path) noexcept;

    ///@}
    ///@name Accessors
    ///@{

    RoadmapType* GetRoadmap(Robot* const _r = nullptr) const noexcept;

    RoadmapType* GetBlockRoadmap(Robot* const _r = nullptr) const noexcept;

    Path* GetPath(Robot* const _r = nullptr) const noexcept;

    LocalObstacleMap* GetLocalObstacleMap(Robot* const _r = nullptr) const
        noexcept;

    GroupRoadmapType* GetGroupRoadmap(RobotGroup* const _g = nullptr) const
        noexcept;

    GroupPathType* GetGroupPath(RobotGroup* const _g = nullptr) const noexcept;

    StatClass* GetStatClass() const noexcept;

    Robot* GetRobot() const noexcept;

    RobotGroup* GetRobotGroup() const noexcept;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Get the solution for a robot.
    /// @param _r The robot.
    /// @return The solution object for _r.
    const RobotSolution* GetRobotSolution(Robot* _r) const noexcept;

    /// Get the solution for a robot.
    /// @param _r The robot.
    /// @return The solution object for _r.
    RobotSolution* GetRobotSolution(Robot* _r) noexcept;

    /// Get the solution for a robot group.
    /// @param _g The robot group.
    /// @return The solution object for _g.
    const GroupSolution* GetGroupSolution(RobotGroup* _g) const noexcept;

    /// Get the solution for a robot group.
    /// @param _g The robot group.
    /// @return The solution object for _g.
    GroupSolution* GetGroupSolution(RobotGroup* _g) noexcept;
    ///@}
    ///@name Internal State
    ///@{

    Robot* m_robot{nullptr};            ///< The robot executing this task.
    RobotGroup* m_group{nullptr};       ///< The robot group.

    std::unique_ptr<StatClass> m_stats; ///< Performance tracking.

    /// The solution object for each robot and group.
    std::unordered_map<Robot*, RobotSolution> m_individualSolutions;
    std::unordered_map<RobotGroup*, GroupSolution> m_groupSolutions;

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MPSolutionType<MPTraits>::
RobotSolution::
RobotSolution(Robot* const _robot, StatClass* const _stats)
  : freeMap(new RoadmapType(_robot)),
    obstMap(new RoadmapType(_robot)),
    path   (new Path(freeMap.get())),
    lom    (new LocalObstacleMap(_stats)) {
  freeMap->SetCCTracker(_stats);
  obstMap->SetCCTracker(_stats);
}


template <typename MPTraits>
MPSolutionType<MPTraits>::
GroupSolution::
GroupSolution(RobotGroup* const _group, MPSolutionType* const _solution)
  : freeMap(new GroupRoadmapType(_group, _solution)),
    obstMap(new GroupRoadmapType(_group, _solution)),
    path   (new GroupPathType(freeMap.get())) { }


template <typename MPTraits>
MPSolutionType<MPTraits>::
MPSolutionType(Robot* const _r)
  : m_robot(_r), m_stats(new StatClass()) {
  m_individualSolutions[m_robot] = RobotSolution(m_robot, m_stats.get());
}


template <typename MPTraits>
MPSolutionType<MPTraits>::
MPSolutionType(RobotGroup* const _g)
  : m_group(_g), m_stats(new StatClass()) {
  // Initialize a solution for each robot in the group.
  for(auto robot : *m_group)
    m_individualSolutions[robot] = RobotSolution(robot, m_stats.get());

  // Initialize a group solution. Must happen after solutions are populated so
  // that the group map can access the individual robot maps.
  m_groupSolutions[m_group] = GroupSolution(_g, this);
}

/*-------------------------------- Modifiers ---------------------------------*/

template <typename MPTraits>
void
MPSolutionType<MPTraits>::
AddRobot(Robot* const _r) noexcept {
  m_robot = _r;

  auto iter = m_individualSolutions.find(_r);
  if(iter != m_individualSolutions.end()) {
    std::cout << "Robot "
              << _r->GetLabel()
              << " is already in solution."
              << std::endl;
    return;
  }
    
  m_individualSolutions[_r] = RobotSolution(_r,m_stats.get());
}

template <typename MPTraits>
void
MPSolutionType<MPTraits>::
SetRobot(Robot* const _r) noexcept {
  if(m_robot == _r)
    return;

  // Move m_robot's solution to match the new pointer.
  auto iter = m_individualSolutions.find(m_robot);
  m_individualSolutions[_r] = std::move(iter->second);
  m_individualSolutions.erase(iter);

  m_robot = _r;

  m_individualSolutions[_r].freeMap->SetRobot(_r);
  m_individualSolutions[_r].obstMap->SetRobot(_r);
  m_individualSolutions[_r].path->FlushCache();
}

template <typename MPTraits>
void
MPSolutionType<MPTraits>::
SetRoadmap(Robot* const _r, RoadmapType* _roadmap) noexcept {
  auto robotSolution = GetRobotSolution(_r);
  if(!robotSolution)
    throw RunTimeException(WHERE) << "Cannot set roadmap for robot that does not "
                                  << "have a solution.";

  robotSolution->freeMap.reset(_roadmap);
  robotSolution->path.reset(new Path(_roadmap));
}

template <typename MPTraits>
void
MPSolutionType<MPTraits>::
SetPath(Robot* const _r, Path* _path) noexcept {
  auto robotSolution = GetRobotSolution(_r);
  if(!robotSolution)
    throw RunTimeException(WHERE) << "Cannot set path for robot that does not "
                                  << "have a solution.";

  robotSolution->path.release();
  robotSolution->path.reset(_path);
}

template <typename MPTraits>
void
MPSolutionType<MPTraits>::
AddRobotGroup(RobotGroup* const _g) noexcept {
  m_group = _g;

  auto iter = m_groupSolutions.find(_g);
  if(iter != m_groupSolutions.end()) {
    std::cout << "Group :"
              << _g->GetLabel()
              << " already in solution." 
              << std::endl;
    return;
  }

  m_groupSolutions[_g] = GroupSolution(_g,this);
}

template <typename MPTraits>
void
MPSolutionType<MPTraits>::
SetGroupRoadmap(RobotGroup* const _g, GroupRoadmapType* _roadmap) noexcept {
  auto groupSolution = GetGroupSolution(_g);
  if(!groupSolution)
    throw RunTimeException(WHERE) << "Cannot set roadmap for group that does not "
                                  << "have a solution.";

  groupSolution->freeMap.reset(_roadmap);
  groupSolution->path.reset(new GroupPathType(_roadmap));
}

template <typename MPTraits>
void
MPSolutionType<MPTraits>::
SetGroupPath(RobotGroup* const _g, GroupPathType* _path) noexcept {
  auto groupSolution = GetGroupSolution(_g);
  if(!groupSolution)
    throw RunTimeException(WHERE) << "Cannot set path for group that does not "
                                  << "have a solution.";

  groupSolution->path.release();
  groupSolution->path.reset(_path);
}

/*---------------------------- Roadmap Accessors -----------------------------*/

template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPSolutionType<MPTraits>::
GetRoadmap(Robot* const _r) const noexcept {
  auto s = GetRobotSolution(_r);
  return s ? s->freeMap.get() : nullptr;
}


template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPSolutionType<MPTraits>::
GetBlockRoadmap(Robot* const _r) const noexcept {
  auto s = GetRobotSolution(_r);
  return s ? s->obstMap.get() : nullptr;
}


template <typename MPTraits>
inline
typename MPTraits::Path*
MPSolutionType<MPTraits>::
GetPath(Robot* const _r) const noexcept {
  auto s = GetRobotSolution(_r);
  return s ? s->path.get() : nullptr;
}


template <typename MPTraits>
inline
typename MPTraits::LocalObstacleMap*
MPSolutionType<MPTraits>::
GetLocalObstacleMap(Robot* const _r) const noexcept {
  auto s = GetRobotSolution(_r);
  return s ? s->lom.get() : nullptr;
}


template <typename MPTraits>
inline
typename MPTraits::GroupRoadmapType*
MPSolutionType<MPTraits>::
GetGroupRoadmap(RobotGroup* const _g) const noexcept {
  auto s = GetGroupSolution(_g);
  return s ? s->freeMap.get() : nullptr;
}


template <typename MPTraits>
typename MPSolutionType<MPTraits>::GroupPathType*
MPSolutionType<MPTraits>::
GetGroupPath(RobotGroup* const _g) const noexcept {
  auto s = GetGroupSolution(_g);
  return s ? s->path.get() : nullptr;
}


template <typename MPTraits>
inline
StatClass*
MPSolutionType<MPTraits>::
GetStatClass() const noexcept {
  return m_stats.get();
}


template <typename MPTraits>
Robot*
MPSolutionType<MPTraits>::
GetRobot() const noexcept {
  return m_robot;
}


template <typename MPTraits>
RobotGroup*
MPSolutionType<MPTraits>::
GetRobotGroup() const noexcept {
  return m_group;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
inline
const typename MPSolutionType<MPTraits>::RobotSolution*
MPSolutionType<MPTraits>::
GetRobotSolution(Robot* _r) const noexcept {
  if(!_r)
    _r = m_robot;

  auto iter = m_individualSolutions.find(_r);
  const bool found = iter != m_individualSolutions.end();
  return found ? &iter->second : nullptr;
}


template <typename MPTraits>
inline
typename MPSolutionType<MPTraits>::RobotSolution*
MPSolutionType<MPTraits>::
GetRobotSolution(Robot* _r) noexcept {
  if(!_r)
    _r = m_robot;

  auto iter = m_individualSolutions.find(_r);
  const bool found = iter != m_individualSolutions.end();
  return found ? &iter->second : nullptr;
}


template <typename MPTraits>
inline
const typename MPSolutionType<MPTraits>::GroupSolution*
MPSolutionType<MPTraits>::
GetGroupSolution(RobotGroup* _g) const noexcept {
  if(!_g)
    _g = m_group;

  auto iter = m_groupSolutions.find(_g);
  const bool found = iter != m_groupSolutions.end();
  return found ? &iter->second : nullptr;
}

template <typename MPTraits>
inline
typename MPSolutionType<MPTraits>::GroupSolution*
MPSolutionType<MPTraits>::
GetGroupSolution(RobotGroup* _g) noexcept {
  if(!_g)
    _g = m_group;

  auto iter = m_groupSolutions.find(_g);
  const bool found = iter != m_groupSolutions.end();
  return found ? &iter->second : nullptr;
}
/*----------------------------------------------------------------------------*/

#endif
