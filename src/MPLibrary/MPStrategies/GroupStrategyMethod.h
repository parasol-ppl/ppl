#ifndef PMPL_GROUP_STRATEGY_METHOD_H_
#define PMPL_GROUP_STRATEGY_METHOD_H_

#include "MPStrategyMethod.h"

#include "ConfigurationSpace/RoadmapGraph.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/MPTask.h"
#include "MPLibrary/Samplers/SamplerMethod.h"
#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for MPStrategies that plan for robot groups.
///
/// @todo Incorporate path constraints when generating the start and goal.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupStrategyMethod : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{
    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename MPTraits::GroupPathType     GroupPath;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename GroupRoadmapType::VID       VID;

    typedef typename SamplerMethod<MPTraits>::BoundaryMap BoundaryMap;

    ///@}
    ///@name Construction
    ///@{

    GroupStrategyMethod() = default;

    GroupStrategyMethod(XMLNode& _node);

    virtual ~GroupStrategyMethod() = default;

    ///@}

  protected:

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Finalize() override;

    virtual size_t GenerateStart(const std::string& _samplerLabel) override;

    virtual std::vector<size_t> GenerateGoals(const std::string& _samplerLabel)
        override;

    ///@}
    ///@name Helpers
    ///@{

    /// Get a map from robot -> boundary for the current group task's start
    /// constraints.
    BoundaryMap GetStartBoundaryMap() const noexcept;

    /// Get a map from robot -> boundary for each of the current group task's
    /// goal constraints.
    std::vector<BoundaryMap> GetGoalBoundaryMaps() const noexcept;

    ///@}

};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
GroupStrategyMethod<MPTraits>::
GroupStrategyMethod(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

template <typename MPTraits>
void
GroupStrategyMethod<MPTraits>::
Finalize() {
  if(!this->m_writeOutput)
    return;

  // auto groupTask = this->GetGroupTask();
  //auto group = groupTask->GetRobotGroup();
  //auto groupRoadmap = this->GetGroupRoadmap();

  // Collect the full cfg paths for each robot.
  //size_t longestPath = 0;
  //std::vector<std::vector<CfgType>> paths;
  //std::vector<std::vector<VID>> paths;
  //paths.reserve(groupTask->Size());
  // size_t i = 0;
  // std::cout << "Printing individual paths" << std::endl;
  // for(auto& task : *groupTask) {
  //   auto robot = task.GetRobot();
  //   auto path = this->GetPath(robot);

  //   // If the path is empty, we didn't solve the problem.
  //   if(path->Empty())
  //     return;

  //   // Collect the path.
  //   //paths.push_back(path->VIDs());
  //   //longestPath = std::max(longestPath, paths.back().size());

  //   std::cout << "VID Path for robot " << robot->GetLabel() << ": " << path->VIDs()
  //             << std::endl;

  //   if(path and path->Size()) {
  //     const std::string base = this->GetBaseFilename();
  //     ::WritePath(base +"robot"+ std::to_string(i) + ".rdmp.path", path->Cfgs());
  //     vector<CfgType> dummy = path->Cfgs();
  //     auto roadmap = this->GetRoadmap(dummy[0].GetRobot());
  //     roadmap->Write(base +"robot"+ std::to_string(i) + ".map", this->GetEnvironment());
  //   }
  //   ++i;

  // }


  const std::string base = this->GetBaseFilename();

#ifdef GROUP_MAP
  // Output final map.
  auto roadmap = this->GetGroupRoadmap();
  roadmap->Write(base + ".map", this->GetEnvironment());

  /// @todo Add a group blocked map and output it here if populated. There is
  ///       storage for it in the MPSolution but no accessor yet.
  //// Output the blocked map if it is populated.
  //auto blockmap = this->GetBlockRoadmap();
  //if(blockmap->Size())
  //  blockmap->Write(base + ".block.map", this->GetEnvironment());

  // Output path vertices. Write both full and roadmap paths for now.
  auto path = this->GetGroupPath();
  if(path and path->Size()) {
    ::WritePath(base + ".rdmp.path", path->Cfgs());
    ::WritePath(base + ".path", path->FullCfgs(this->GetMPLibrary()));
  }

  // Output stats.
  std::ofstream osStat(base + ".stat");
  this->GetStatClass()->PrintAllStats(osStat, roadmap);
#else
  // Output stats.
  std::ofstream osStat(base + ".stat");
  this->GetStatClass()->PrintAllStats(osStat);
#endif
}


template <typename MPTraits>
size_t
GroupStrategyMethod<MPTraits>::
GenerateStart(const std::string& _samplerLabel) {
  /// @todo For now we will only support one task per robot. Generalize this to
  ///       support more task arrangements.
  MethodTimer mt(this->GetStatClass(), this->GetName() + "::GenerateStart");

  // Determine which sampler to use.
  const auto& samplerLabel = this->m_querySampler.empty() ? _samplerLabel
                                                          : this->m_querySampler;
  auto sampler = this->GetSampler(samplerLabel);

  if(this->m_debug)
    std::cout << "Generating start configuration with sampler '"
              << samplerLabel << "'."
              << std::endl;

  // Collect boundaries for individual robots.
  const auto boundaryMap = this->GetStartBoundaryMap();

  // Sample start configurations which place all robots at covalid
  // configurations and mapped robots within their start constraints.
  constexpr size_t attempts = 100;
  std::vector<GroupCfgType> cfgs;
  // TODO Write this API
  sampler->Sample(1, attempts, boundaryMap, std::back_inserter(cfgs));

  // Throw an error if we failed to generate a single configuration.
  if(cfgs.empty())
    throw RunTimeException(WHERE) << "Could not generate valid start "
                                  << "configuration.";

  // Add the configurations to the roadmap.
  const auto& start = cfgs.front();
  auto g = this->GetGroupRoadmap();
  const VID vid = g->AddVertex(start);

  if(this->m_debug)
    std::cout << "\tVID " << vid << " at " << start.PrettyPrint()
              << std::endl;

  return vid;
}


template <typename MPTraits>
std::vector<size_t>
GroupStrategyMethod<MPTraits>::
GenerateGoals(const std::string& _samplerLabel) {
  /// @todo For now we will only support one task per robot. Generalize this to
  ///       support more task arrangements.
  MethodTimer mt(this->GetStatClass(), this->GetName() + "::GenerateGoals");

  // Determine which sampler to use.
  const auto& samplerLabel = this->m_querySampler.empty() ? _samplerLabel
                                                          : this->m_querySampler;
  auto sampler = this->GetSampler(samplerLabel);

  if(this->m_debug)
    std::cout << "Generating goal configurations with sampler '"
              << samplerLabel << "'."
              << std::endl;

  // Collect boundaries for individual robots for each goal.
  const auto boundaryMaps = this->GetGoalBoundaryMaps();

  // Generate configurations for each boundary map.
  std::vector<VID> vids;
  for(size_t i = 0; i < boundaryMaps.size(); ++i) {
    if(this->m_debug)
      std::cout << "\n\tGoal " << i << ":"
                << std::endl;
    const auto& boundaryMap = boundaryMaps[i];

    // Sample goal configurations which place all robots at covalid
    // configurations and mapped robots within their goal constraints.
    constexpr size_t attempts = 100;
    std::vector<GroupCfgType> cfgs;
    // TODO Write this API
    sampler->Sample(1, attempts, boundaryMap, std::back_inserter(cfgs));

    // Throw an error if we failed to generate a single configuration.
    if(cfgs.empty())
      throw RunTimeException(WHERE) << "Could not generate valid goal "
                                    << "configuration.";

    // Add the configurations to the roadmap.
    const auto& goal = cfgs.front();
    auto g = this->GetGroupRoadmap();
    const VID vid = g->AddVertex(goal);

    if(this->m_debug)
      std::cout << "\t\tVID " << vid << " at " << goal.PrettyPrint()
                << std::endl;

    vids.push_back(vid);
  }

  return vids;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
typename GroupStrategyMethod<MPTraits>::BoundaryMap
GroupStrategyMethod<MPTraits>::
GetStartBoundaryMap() const noexcept {
  /// @todo For now we will only support one task per robot. Generalize this to
  ///       support more task arrangements.
  BoundaryMap map;

  auto groupTask = this->GetGroupTask();
  auto group = groupTask->GetRobotGroup();

  if(this->m_debug)
    std::cout << "Generating start boundary map for robot group '"
              << group->GetLabel() << "'."
              << std::endl;

  for(auto& task : *groupTask) {
    // For now we will require that every task is mapped to a boundary.
    auto robot = task.GetRobot();
    if(!robot)
      throw NotImplementedException(WHERE) << "Task '" << task.GetLabel()
                                           << "' is not assigned to a robot. "
                                           << "Unmapped individual tasks are "
                                           << "not yet supported.";

    // Skip individual tasks with no start constraint.
    auto startConstraint = task.GetStartConstraint();
    if(!startConstraint)
      continue;

    // Skip individual tasks with no constraint boundary.
    auto boundary = startConstraint->GetBoundary();
    if(!boundary)
      continue;

    // If we already have a boundary for this robot, we would need to
    // intersect them to sample in both. Throw an exception for now until we
    // devise a general mechanism for doing this.
    if(map.count(robot))
      throw NotImplementedException(WHERE) << "Robot '" << robot->GetLabel()
                                           << "' already mapped to boundary '"
                                           << *map[robot]
                                           << "'. Mapping to second boundary '"
                                           << *boundary << "' from task '"
                                           << task.GetLabel()
                                           << "' is not yet supported ("
                                           << "requires boundary "
                                           << "intersections).";

    // Map this robot to this boundary.
    map[robot] = boundary;
    if(this->m_debug)
      std::cout << "\tMapped robot '" << robot->GetLabel()
                << "' to task '" << task.GetLabel()
                << "' with boundary " << *boundary
                << "."
                << std::endl;
  }

  if(this->m_debug) {
    // Find the robots which are not mapped.
    for(Robot* const r : *group)
      if(!map.count(r))
        std::cout << "\n\tRobot '" << r->GetLabel() << "' is not mapped.";

    std::cout << std::endl;
  }

  return map;
}


template <typename MPTraits>
std::vector<typename GroupStrategyMethod<MPTraits>::BoundaryMap>
GroupStrategyMethod<MPTraits>::
GetGoalBoundaryMaps() const noexcept {
  /// @todo For now we will only support one task per robot. Generalize this to
  ///       support more task arrangements.
  std::vector<BoundaryMap> maps;

  auto groupTask = this->GetGroupTask();
  auto group = groupTask->GetRobotGroup();

  // Determine how many goals are in the longest task.
  size_t maxGoals = groupTask->GetNumGoals();

  if(this->m_debug)
    std::cout << "Generating goal boundary map for robot group '"
              << group->GetLabel() << "'."
              << "\nLongest task has " << maxGoals << " goals."
              << std::endl;

  // Assuming that all robots must end satsifying their final task, we will use
  // maxGoals maps and populate them from back to front (so that all robots will
  // have a last boundary). This will leave robots with smaller goal sets as
  // unconstrained while the robots with larger goal sets are reaching their
  // intermediate goals.
  maps.resize(maxGoals);
  for(size_t i = maxGoals; i > 0; --i) {
    // We are collecting the (i-1)th constraints.
    const size_t goalIndex     = i - 1,
                 boundaryCount = maxGoals - goalIndex;
    auto& map = maps[goalIndex];

    if(this->m_debug)
      std::cout << "\tMapping goal " << goalIndex << "."
                << std::endl;

    for(auto& task : *groupTask) {
      // For now we will require that every task is mapped to a boundary.
      auto robot = task.GetRobot();
      if(!robot)
        throw NotImplementedException(WHERE) << "Task '" << task.GetLabel()
                                             << "' is not assigned to a robot. "
                                             << "Unmapped individual tasks are "
                                             << "not yet supported.";

      // Skip individual tasks with too few goal constraints.
      const auto& goalConstraints = task.GetGoalConstraints();
      if(goalConstraints.size() < boundaryCount)
        continue;

      // Get the constraint.
      const size_t constraintIndex = goalConstraints.size() - boundaryCount;
      const auto& constraint = goalConstraints[constraintIndex];

      // Skip individual tasks with no constraint boundary.
      auto boundary = constraint->GetBoundary();
      if(!boundary)
        continue;

      // If we already have a boundary for this robot, we would need to
      // intersect them to sample in both. Throw an exception for now until we
      // devise a general mechanism for doing this.
      if(map.count(robot))
        throw NotImplementedException(WHERE) << "Robot '" << robot->GetLabel()
                                             << "' already mapped to boundary '"
                                             << *map[robot]
                                             << "'. Mapping to second boundary '"
                                             << *boundary << "' from task '"
                                             << task.GetLabel()
                                             << "' is not yet supported ("
                                             << "requires boundary "
                                             << "intersections).";

      // Map this robot to this boundary.
      map[robot] = boundary;
      if(this->m_debug)
        std::cout << "\t\tMapped robot '" << robot->GetLabel()
                  << "' to task '" << task.GetLabel()
                  << "' with boundary " << *boundary
                  << "."
                  << std::endl;
    }

    if(this->m_debug) {
      // Find the robots which are not mapped.
      for(Robot* const r : *group)
        if(!map.count(r))
          std::cout << "\n\t\tRobot '" << r->GetLabel() << "' is not mapped.";

      std::cout << std::endl;
    }
  }

  return maps;
}

/*----------------------------------------------------------------------------*/

#endif
