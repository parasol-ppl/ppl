#include "RegionKit.h"

#include <random>

#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"
#include "Utilities/MPUtils.h"
#include "Utilities/XMLNode.h"

#ifdef PMPL_USE_SIMULATOR
#include "Simulator/Simulation.h"
static const glutils::color regionColor{0, 0, 1, .5};
static constexpr unsigned int sleepTime = 10000;
#endif

#include "nonstd/io.h"


/*------------------------------- Construction -------------------------------*/

RegionKit::
RegionKit(XMLNode& _node) {
  m_debug = _node.Read("debug", false, m_debug, "Show debug messages");

  m_explore = _node.Read("explore", true, m_explore, 0., 1.,
      "Weight of explore vs. exploit in region selection probabilities");

  m_regionFactor = _node.Read("regionFactor", true,
      m_regionFactor, 1., std::numeric_limits<double>::max(),
      "Regions are this * robot's bounding sphere radius");

  m_penetrationFactor = _node.Read("penetration", true,
      m_penetrationFactor, std::numeric_limits<double>::min(), 1.,
      "Fraction of bounding sphere penetration that is considered touching");
}


RegionKit::
~RegionKit() {
  Clear();
}


void
RegionKit::
Clear() {
  // Release any remaining dynamic regions.
  for(auto& keyValue : m_regionData)
    delete keyValue.first;
  m_regionData.clear();
  m_visited.clear();
}

/*--------------------------------- Sampling ---------------------------------*/

const Boundary*
RegionKit::
SelectRegion() {
  // Update all region probabilities.
  std::vector<double> probabilities = ComputeProbabilities();

  // Construct with random number generator with the region probabilities.
  static std::default_random_engine generator(0);
  std::discrete_distribution<size_t> distribution(probabilities.begin(),
      probabilities.end());

  const size_t index = distribution(generator);
  const bool envSelected = index == m_regionData.size();

  if(m_debug) {
    std::cout << "RegionKit:: updated region selection probabilities ("
              << "last is whole env):\n\t";

    for(auto p : distribution.probabilities())
      std::cout << std::setprecision(4) << p << " ";

    std::cout << "\n\tSelected index " << index
              << (envSelected ? " (whole env)." : " ")
              << std::endl;
  }

  if(envSelected)
    return nullptr;

  // Get the selected region.
  auto it = m_regionData.begin();
  advance(it, index);

  if(m_debug) {
    auto c = it->first->GetCenter();
    std::cout << "\tRegion is " << it->first << " with center at "
              << Vector3d(c[0], c[1], c[2]) << ", success rate so far "
              << it->second.successes << " / " << it->second.attempts
              << "." << std::endl;
  }

  // total samples increment
  ++it->second.attempts;

  return it->first;
}


void
RegionKit::
IncrementFailure(const Boundary* const _region, const size_t _count) {
  // _region should be a const handle to a non-const boundary owned by this
  // object, so we should be safe to cast away the const.
  Boundary* region = const_cast<Boundary*>(_region);

  // Ensure that this region exists.
  auto iter = m_regionData.find(region);
  if(iter == m_regionData.end())
    throw RunTimeException(WHERE) << "Cannot increment success for non-existing "
                                  << "region '" << region << "'";

  iter->second.attempts += _count;
}


void
RegionKit::
IncrementSuccess(const Boundary* const _region, const size_t _count) {
  // _region should be a const handle to a non-const boundary owned by this
  // object, so we should be safe to cast away the const.
  Boundary* region = const_cast<Boundary*>(_region);

  // Ensure that this region exists.
  auto iter = m_regionData.find(region);
  if(iter == m_regionData.end())
    throw RunTimeException(WHERE) << "Cannot increment attempts for "
                                  << "non-existing region '" << region << "'";

  iter->second.successes += _count;
  iter->second.attempts  += _count;
}


const Vector3d
RegionKit::
GetVelocityBias(const Boundary* const _region) const {
  // _region should be a const handle to a non-const boundary owned by this
  // object, so we should be safe to cast away the const.
  Boundary* region = const_cast<Boundary*>(_region);

  // Get the region data.
  const auto& regionData = m_regionData.at(region);
  const size_t index = regionData.edgeIndex;

  // Find the skeleton edge path the region is traversing.
  auto edge = m_skeleton->FindEdge(regionData.edgeDescriptor);
  const auto& path = edge->property();

  // Helper to make the biasing direction and print debug info.
  auto makeBias = [&](const Vector3d& _start, const Vector3d& _end) {
    if(m_debug)
      std::cout << "Computed velocity bias: " << (_end - _start).normalize()
                << "\n\tStart: " << _start
                << "\n\tEnd:   " << _end
                << std::endl;
    return (_end - _start).normalize();
  };

  // If there is at least one valid path point after the current path index,
  // then return the direction to the next point.
  if(index < path.size() - 1) {
    if(m_debug)
      std::cout << "Biasing velocity along next path step"
                << "\n\tPath index: " << index
                << "\n\tPath size:  " << path.size()
                << std::endl;
    return makeBias(path[index], path[index + 1]);
  }

  // Otherwise, the region has reached a skeleton vertex.
  auto vertex = m_skeleton->GetGraph().find_vertex(edge->target());

  // If the vertex has no outgoing edges, this is the end of the skeleton. In
  // that case, use the previous biasing direction. All paths have at least two
  // points so this is safe.
  if(vertex->size() == 0) {
    if(m_debug)
      std::cout << "Biasing velocity along previous path step"
                << "\n\tPath index: " << index
                << "\n\tPath size:  " << path.size()
                << std::endl;
    return makeBias(path[index - 1], path[index]);
  }

  // Otherwise, randomly select an outgoing and use it's next point.
  auto eit = vertex->begin();
  const size_t nextEdgeIndex = LRand() % vertex->size();
  std::advance(eit, nextEdgeIndex);
  if(m_debug)
    std::cout << "Biasing velocity along next edge (index " << nextEdgeIndex
              << ")\n\tPath index: " << index
              << "\n\tPath size:  " << path.size()
              << "\n\tNext edge path size: " << eit->property().size()
              << std::endl;
  return makeBias(path[index], eit->property()[1]);
}

/*------------------------------------ I/O -----------------------------------*/

void
RegionKit::
Print(std::ostream& _os) const {
  _os << "RegionKit"
      << "\n\tRegion factor: " << m_regionFactor
      << "\n\tRegion radius: " << m_regionRadius
      << "\n\tPreference for explore in [0:1]:" << m_explore
      << "\n\tBounding sphere penetration fraction in [0:1]:"
      << m_penetrationFactor
      << std::endl;
}

/*---------------------------- Skeleton Following ----------------------------*/

void
RegionKit::
InitRegions(const Point3d& _start) {
  // Mark all nodes unvisited.
  m_visited.clear();
  auto& g = m_skeleton->GetGraph();
  for(auto vit = g.begin(); vit != g.end(); ++vit)
    m_visited[vit->descriptor()] = false;

  // Find the vertex nearest to start and create regions for each outgoing
  // edge.
  auto iter = m_skeleton->FindNearestVertex(_start);
  CreateRegions(iter);

  // Set up the skeleton visualization.
#ifdef PMPL_USE_SIMULATOR
  Simulation::Get()->AddWorkspaceSkeleton(m_skeleton, glutils::color::orange);
  usleep(sleepTime);
#endif
}


void
RegionKit::
CreateRegions(const Point3d& _p) {
  auto& g = m_skeleton->GetGraph();

  // Check each skeleton node to see if a new region should be created.
  for(auto iter = g.begin(); iter != g.end(); ++iter) {
    // Skip skeleton nodes that are too far away.
    const double dist = (iter->property() - _p).norm();
    if(dist >= m_regionRadius)
      continue;

    CreateRegions(iter);
  }
}


std::vector<Boundary*>
RegionKit::
CreateRegions(const WorkspaceSkeleton::vertex_iterator _iter) {
  // Skip skeleton nodes that are already visited.
  if(m_visited[_iter->descriptor()])
    return {};
  m_visited[_iter->descriptor()] = true;

  // Save the set of created regions to return.
  std::vector<Boundary*> newRegions;

  // Create a new region for each outgoing edge of this skeleton node.
  for(auto eit = _iter->begin(); eit != _iter->end(); ++eit) {
    auto r = new WorkspaceBoundingSphere(_iter->property(), m_regionRadius);
    m_regionData.emplace(r, RegionData(eit->descriptor()));
    newRegions.push_back(r);

#ifdef PMPL_USE_SIMULATOR
    // Show visualization if we are using the simulator.
    m_regionData[r].visualizationID = Simulation::Get()->AddBoundary(r,
        regionColor, true);
    usleep(sleepTime);
#endif

    if(m_debug)
      std::cout << "RegionKit:: created new region " << r << " with radius "
                << std::setprecision(4) << m_regionRadius
                << " on edge (" << eit->source() << ", "
                << eit->target() << ", " << eit->id() << ") "
                << "at skeleton vertex " << _iter->descriptor()
                << "(" << _iter->property() << ")."
                << std::endl;
  }

  return newRegions;
}


void
RegionKit::
AdvanceRegions(const Cfg& _cfg) {
  if(m_debug)
    std::cout << "RegionKit:: checking " << m_regionData.size()
              << " regions for contact with new configuration "
              << _cfg.PrettyPrint() << "."
              << std::endl;

  // Keep track of any newly reached vertices to spawn regions on their outbound
  // edges.
  std::queue<WorkspaceSkeleton::VD> newlyReachedVertices;

  // Iterate through all existing regions to see which should be advanced.
  for(auto iter = m_regionData.begin(); iter != m_regionData.end(); ) {
    Boundary* region = iter->first;

    // Advance this region until the robot at _cfg is no longer touching it.
    if(!AdvanceRegionToCompletion(_cfg, region)) {
      ++iter;
      continue;
    }

    // We have reached the end of this region's edge. Delete it and save the
    // target vertex. to spawn new regions.
    auto edgeIter = m_skeleton->FindEdge(iter->second.edgeDescriptor);
    const auto target = edgeIter->target();
    if(!m_visited[target])
      newlyReachedVertices.push(target);
    iter = m_regionData.erase(iter);
  }

  if(m_debug)
    std::cout << newlyReachedVertices.size()
              << " new vertices reached while advancing regions."
              << std::endl;

  // Create new regions for each newly reached vertex.
  while(!newlyReachedVertices.empty()) {
    // Pop the next vertex off the queue.
    WorkspaceSkeleton::VD targetVD = newlyReachedVertices.front();
    WorkspaceSkeleton::vertex_iterator target = m_skeleton->FindVertex(targetVD);
    newlyReachedVertices.pop();

    // Create regions at this vertex.
    std::vector<Boundary*> newRegions = CreateRegions(target);

    // Advance each new region.
    for(auto region : newRegions) {
      // Advance this region until the robot at _cfg is no longer touching it.
      if(!AdvanceRegionToCompletion(_cfg, region))
        continue;

      // We have reached the end of this region's edge. Delete it and save the
      // target vertex. to spawn new regions.
      auto iter = m_regionData.find(region);
      auto edgeIter = m_skeleton->FindEdge(iter->second.edgeDescriptor);
      newlyReachedVertices.push(edgeIter->target());
      m_regionData.erase(iter);
    }
  }
}


bool
RegionKit::
AdvanceRegionToCompletion(const Cfg& _cfg, Boundary* const _region) {
  // Find the edge path this region is traversing.
  auto& data = m_regionData[_region];
  auto edgeIter = m_skeleton->FindEdge(data.edgeDescriptor);
  const std::vector<Point3d>& path = edgeIter->property();
  size_t& i = data.edgeIndex;

  if(m_debug)
    std::cout << "\tChecking region " << _region << " at "
              << _region->GetCenter() << "."
              << "\n\t Region is at index " << i << " / " << path.size() - 1
              << std::endl;

  while(IsTouching(_cfg, _region)) {

    // If there are no more points left on this edge, this region is completed.
    if(i == path.size() - 1) {
      if(m_debug)
        std::cout << "\t Region has reached the end of its "
                  << "path, erasing it now. " << m_regionData.size() - 1
                  << " regions remain."
                  << std::endl;

      // Remove visualization if we are using it.
      #ifdef PMPL_USE_SIMULATOR
      usleep(sleepTime);
      Simulation::Get()->RemoveBoundary(data.visualizationID);
      usleep(sleepTime);
      #endif

      return true;
    }

    // Otherwise there are still points left; advance the region and index.
    const Point3d& current = path[i];
    const Point3d& next = path[++i];
    _region->SetCenter({next[0], next[1], next[2]});

    if(m_debug)
      std::cout << "\t Advancing region from index "
                << i - 1 << " to " << i << " / " << path.size() - 1
                << " (" << (next - current).norm() << " units)."
                << std::endl;

    // Advance visualization if we are using it.
    #ifdef PMPL_USE_SIMULATOR
    const glutils::transform t{1, 0, 0, 0,
                               0, 1, 0, 0,
                               0, 0, 1, 0,
                               float(next[0]), float(next[1]), float(next[2]), 1};
    Simulation::Get()->TransformBoundary(data.visualizationID, t);
    usleep(sleepTime);
    #endif
  }

  if(m_debug)
    std::cout << "\t Region is still traversing this edge." << std::endl;

  return false;
}


bool
RegionKit::
IsTouching(const Cfg& _cfg, const Boundary* const _region) const {
  // Compute the penetration distance required. We want the robot's bounding
  // sphere to penetrate the region by the fraction m_penetrationThreshold.
  const double robotRadius  = _cfg.GetMultiBody()->GetBoundingSphereRadius(),
               threshold    = 2 * robotRadius * m_penetrationFactor;

  // Compute the penetration distance (maximally enclosed bounding diameter).
  const Point3d robotCenter = _cfg.GetPoint();
  const double penetration = _region->GetClearance(robotCenter) + robotRadius;

  // The configuration is touching if the penetration exceeds the threshold.
  const bool touching = penetration >= threshold;

  if(m_debug)
    std::cout << "\t Touch test: " << (touching ? "passed" : "failed")
              << "\n\t  Bounding sphere: " << robotCenter << " ; " << robotRadius
              << "\n\t  Region:          " << _region->GetCenter() << " ; "
              << m_regionRadius
              << "\n\t  Bounding sphere penetrates by "
              << std::setprecision(4)
              << penetration << (touching ? " >= " : " < ") << threshold
              << " units."
              << std::endl;

  return touching;
}


std::vector<double>
RegionKit::
ComputeProbabilities() {
  /// @TODO The current implementation is O(|regions|) time, but we should be
  ///       able to do it in O(1) by keeping a running total weight and adjusting
  ///       it and the region weight after each sampling attempt. Main blocker
  ///       at this time is the probability distribution object, which we could
  ///       easily manually re-implement. Region deletion will still be
  ///       O(|regions|) though because we need to update all the probabilites.
  ///       Even an incremental solution here will be O(|regions|) on average.

  // Sum all weights of all current regions.
  double totalWeight = 0.;
  for(auto& keyValue : m_regionData) {
    auto& regionInfo = keyValue.second;

    // Compute weight of this region.
    regionInfo.weight = regionInfo.successes /
        static_cast<double>(regionInfo.attempts);

    // Add to total.
    totalWeight += regionInfo.weight;
  }

  // Compute the probabilities for the current regions.
  std::vector<double> probabilities;
  probabilities.reserve(m_regionData.size() + 1);

  const double explore = m_explore / (m_regionData.size() + 1);

  for(const auto& keyValue : m_regionData) {
    const auto& weight = keyValue.second.weight;
    const double exploit = (1 - m_explore) * weight / totalWeight;

    probabilities.emplace_back(exploit + explore);
  }

  // Get the probability for the whole environment.
  probabilities.emplace_back(explore);

  return probabilities;
}

/*----------------------------------------------------------------------------*/
