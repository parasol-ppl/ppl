#ifndef PMPL_REGION_KIT_H_
#define PMPL_REGION_KIT_H_

#include <iostream>
#include <limits>
#include <map>
#include <vector>

#include "Utilities/XMLNode.h"
#include "Workspace/WorkspaceSkeleton.h"

#include "Vector.h"

using namespace mathtool;

class Boundary;
class Cfg;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Creates and manages a set of dynamic sampling regions that follow a
/// workspace skeleton.
////////////////////////////////////////////////////////////////////////////////
class RegionKit final {

  private:

    ///@name Local Types
    ///@{

    /// Use the skeleton's name for a skeleton edge descriptor.
    using ED = WorkspaceSkeleton::ED;

    ////////////////////////////////////////////////////////////////////////////
    /// Auxiliary data for regions.
    ////////////////////////////////////////////////////////////////////////////
    struct RegionData {

      ///@name Internal State
      ///@{

      ED edgeDescriptor;     ///< Descriptor of the edge the region is traveling.
      size_t edgeIndex{0};   ///< Region is at this index on the skeleton edge.
      size_t successes{0};   ///< Number of valid samples in this region.
      size_t attempts{1};    ///< Number of sampling attempts in this region.
      double weight{0};      ///< Ratio of successful samples to total samples.

      /// Drawable ID when using simulator.
      size_t visualizationID{std::numeric_limits<size_t>::max()};

      ///@}
      ///@name Construction
      ///@{

      /// Construct region metadata from an edge descriptor.
      RegionData(const ED& _ed = ED()) : edgeDescriptor(_ed) {}

      ///@}

    };

    ///@}
    ///@name Internal State
    ///@{

    WorkspaceSkeleton* m_skeleton{nullptr};  ///< The workspace skeleton.

    /// The set of active dynamic sampling regions and associated metadata.
    std::unordered_map<Boundary*, RegionData> m_regionData;

    /// Keep track of which skeleton vertices we've visited.
    std::unordered_map<WorkspaceSkeleton::VD, bool> m_visited;

    /// The dynamic sampling regions will have radius equal to this times the
    /// robot's bounding sphere radius.
    double m_regionFactor{2};

    double m_regionRadius{0}; ///< The region radius.

    /// Weight of explore vs. exploit in region selection probabilities.
    /// Exploring is a uniform chance to select each region, while exploit
    /// favors successful regions.
    double m_explore{.5};

    /// A configuration is considered to be touching a region when this fraction
    /// of its bounding sphere penetrates into the region.
    double m_penetrationFactor{1};

    bool m_debug{false};         ///< Show debug messages?

    ///@}

  public:

    ///@name Construction
    ///@{

    RegionKit() = default;

    RegionKit(XMLNode& _node);

    ~RegionKit();

    /// Initialize a region kit for following a workspace skeleton.
    /// @param _skeleton The skeleton to follow.
    /// @param _point The workspace starting point.
    /// @param _robotRadius The robot bounding sphere radius.
    /// @param _label The label to use for the roadmap hook.
    /// @param _graph The roadmap to hook onto.
    template <typename RoadmapGraph>
    void
    Initialize(WorkspaceSkeleton* const _skeleton, const Point3d& _point,
        const double _robotRadius, const std::string& _label,
        RoadmapGraph* _graph);

    /// Release all regions and clear the internal structures.
    void Clear();

    ///@}
    ///@name Sampling
    ///@{

    /// Select a region based on its weight.
    const Boundary* SelectRegion();

    /// Increment the failed sampling attempts of a region.
    /// @param _region The region which generated an invalid sample.
    /// @param _count The number to increment.
    void IncrementFailure(const Boundary* const _region, const size_t _count = 1);

    /// Increment the sampling success of a region.
    /// @param _region The region which generated a successful sample.
    /// @param _count The number to increment.
    void IncrementSuccess(const Boundary* const _region, const size_t _count = 1);

    /// Get the velocity bias for a given sampling region.
    /// @param _region The sampling region.
    /// @return The prefered velocity direction for samples within _region.
    const Vector3d GetVelocityBias(const Boundary* const _region) const;

    ///@}
    ///@name I/O
    ///@{

    /// Print parameters.
    /// @param _os The out stream to write to.
    void Print(std::ostream& _os) const;

    ///@}

  private:

    ///@name Skeleton Following
    ///@{

    /// Create regions on each edge outbound from the skeleton vertex nearest to
    /// some starting point in workspace.
    /// @param _start Workspace point.
    void InitRegions(const Point3d& _start);

    /// Create new regions along outgoing edges at each unvisted vertex within
    /// a region radius from some point of interest.
    /// @param _p The point.
    void CreateRegions(const Point3d& _p);

    /// Create new regions along outgoing edges at an unvisted vertex.
    /// @param _iter The vertex iterator.
    /// @return The set of created regions.
    std::vector<Boundary*> CreateRegions(
        const WorkspaceSkeleton::vertex_iterator _iter);

    /// Test all regions for containment of new configuration and advance if
    /// necessary. Any completed regions will spawn new ones at their
    /// destinations outbound edges, which will also be advanced if necessary.
    /// @param _cfg The new configuration.
    void AdvanceRegions(const Cfg& _cfg);

    /// Advance a single region until it no longer touches a configuration.
    /// @param _cfg The configuration.
    /// @param _region The region to advance.
    /// @return True iff _region has reached the end of its edge.
    bool AdvanceRegionToCompletion(const Cfg& _cfg, Boundary* const _region);

    /// Test if a configuration is inside a region
    /// @param _cfg The configuration to test.
    /// @param _region The region.
    /// @return True if the robot's bounding sphere is at least one region
    ///         radius times one robot factor into the _region.
    bool IsTouching(const Cfg& _cfg, const Boundary* const _region) const;

    /// Compute the probabilities of selecting each sampling region and
    /// environment.
    std::vector<double> ComputeProbabilities();

    ///@}

};


template <typename RoadmapGraph>
void
RegionKit::
Initialize(WorkspaceSkeleton* const _skeleton, const Point3d& _point,
    const double _robotRadius, const std::string& _label, RoadmapGraph* _graph) {
  if(_graph->IsHook(RoadmapGraph::HookType::AddVertex, _label))
    _graph->RemoveHook(RoadmapGraph::HookType::AddVertex, _label);
  // Set the skeleton pointer and initialize the regions/auxiliary data.
  m_skeleton = _skeleton;
  m_regionRadius = m_regionFactor * _robotRadius;

  InitRegions(_point);

  // On each new sample, check if we need to advance our regions and generate
  // new ones. Add a roadmap hook to achieve this.
  auto addVertex = [this](typename RoadmapGraph::VI _vi) {
    if(this->m_debug)
      std::cout << "RegionKit:: checking region advancement..." << std::endl;

    this->CreateRegions(_vi->property().GetPoint());
    this->AdvanceRegions(_vi->property());
  };
  _graph->InstallHook(RoadmapGraph::HookType::AddVertex, _label, addVertex);
}

#endif
