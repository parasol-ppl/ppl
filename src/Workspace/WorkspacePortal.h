#ifndef WORKSPACE_PORTAL_H_
#define WORKSPACE_PORTAL_H_

#include <vector>

#include "Geometry/GMSPolyhedron.h"

class WorkspaceDecomposition;
class WorkspaceRegion;


////////////////////////////////////////////////////////////////////////////////
/// Describes a connection between two workspace regions.
///
/// This is a light-weight object: it represents points and facets shared
/// between regions, but does not store them explicitly. The shared objects are
/// instead computed as needed.
////////////////////////////////////////////////////////////////////////////////
class WorkspacePortal {

  public:

    ///@name Local Types
    ///@{

    typedef GMSPolygon Facet; ///< A triangle facet.

    ///@}
    ///@name Construction
    ///@{

    /// Default construction makes no sense for this object, but the stapl graph
    /// requires it anyway.
    WorkspacePortal();

    /// Construct a workspace portal between a source and target region in a
    /// given workspace decomposition.
    /// @param _wd The owning decomposition.
    /// @param _s The source region descriptor.
    /// @param _t the Target region descriptor.
    WorkspacePortal(WorkspaceDecomposition* const _wd, const size_t _s,
        const size_t _t);

    /// Change the owning decomposition. This only makes sense when we are
    /// copying/moving portals from one decomposition to another.
    /// @param _wd The new owning decomposition.
    void SetDecomposition(WorkspaceDecomposition* const _wd);

    ///@}
    ///@name Accessors
    ///@{

    const size_t GetSourceDescriptor() const noexcept;
    const size_t GetTargetDescriptor() const noexcept;

    const WorkspaceRegion& GetSource() const noexcept;
    const WorkspaceRegion& GetTarget() const noexcept;

    /// Get the distance between region centers through the facet midpoint.
    double GetWeight() const noexcept;

    ///@}
    ///@name Queries
    ///@{

    /// Find the set of points that are common to both source and target region.
    const std::vector<Point3d> FindPoints() const;

    /// Find the set of facets from the source region that connect the source to
    /// the target.
    const std::vector<const Facet*> FindFacets() const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    double ComputeWeight() const;

    ///@}
    ///@name Internal State
    ///@{

    WorkspaceDecomposition* m_decomposition{nullptr}; ///< Owning decomposition.

    size_t m_sourceIndex; ///< The source region.
    size_t m_targetIndex; ///< The target region.

    double m_weight;  ///< Distance between centers through the facet midpoint.

    ///@}

};

#endif
