#ifndef PMPL_GRID_OVERLAY_H_
#define PMPL_GRID_OVERLAY_H_

#include "ConfigurationSpace/Cfg.h"

#include "Transformation.h"
#include "Vector.h"

#include <array>
#include <memory>
#include <unordered_set>
#include <vector>

class Boundary;
class GMSPolyhedron;
class WorkspaceBoundingBox;
class WorkspaceDecomposition;
class WorkspaceRegion;


///////////////////////////////////////////////////////////////////////////////
/// A 3d grid overlay of a given boundary.
///
/// The cells are implicitly represented as either a tuple (x,y,z) or a single
/// 'cell index', which enumerates all cells with a single number. The grid is
/// defined over the minimum and maximum ranges of the boundary, so there may be
/// grid cells partially or completely outside of the boundary (depending its
/// shape and size).
///
/// @todo Generalize to support 2d grids as well as 3d.
///////////////////////////////////////////////////////////////////////////////
class GridOverlay {

  public:

    ///@name Local Types
    ///@{

    /// A mapping from a grid cell index to a set of workspace regions.
    typedef std::vector<std::vector<const WorkspaceRegion*>> DecompositionMap;

    /// A set of X, Y, Z indexes for a grid cell.
    typedef std::array<size_t, 3> IndexSet;

    /// The types of cell sets for use with LocateCells.
    enum class CellSet : unsigned char {Boundary = 1, Interior = 2, Closure = 3};

    ///@}
    ///@name Construction
    ///@{

    /// Construct a grid overlay with cells of a given length.
    /// @param _b The boundary to overlay.
    /// @param _length The cell length to use.
    GridOverlay(const Boundary* const _b, const double _length);

    ~GridOverlay() noexcept;

    ///@}
    ///@name Cell Finders
    ///@{

    /// Get the number of cells in the grid.
    size_t Size() const noexcept;

    /// Get the number of cells in a given dimension.
    /// @param _i The index for the dimension of interest.
    /// @return The number of cells in dimension _i.
    size_t Size(const size_t _i) const noexcept;

    /// Find the cell that contains the reference point of a configuration.
    /// @param _cfg The configuration to locate.
    /// @return The index of the cell which contains _cfg's translational DOFs.
    size_t LocateCell(const Cfg& _cfg) const;

    /// Find the cell that contains a reference point.
    /// @param _p The point to locate.
    /// @return The index of the cell which contains _p.
    size_t LocateCell(const Point3d& _p) const;

    /// Find the cells that touch a given boundary.
    /// @param _b The boundary of interest.
    /// @param _type The type of cell set to include.
    /// @return A set of cell indexes that touch _b as described by _type.
    std::unordered_set<size_t> LocateCells(const Boundary* const _b,
        const CellSet _type = CellSet::Closure) const;

    /// Find the cells that touch a given polyhedron.
    /// @param _polyhedron The polyhedron of interest.
    /// @param _transformation The transformation for _polyhedron.
    /// @param _type The type of cell set to include.
    /// @return A set of cell indexes that touch _polyhedron as described by
    ///         _type.
    std::unordered_set<size_t> LocateCells(const GMSPolyhedron& _polyhedron,
        const mathtool::Transformation& _transformation = {},
        const CellSet _type = CellSet::Closure) const;

    /// Find the cells that contain a given boundary's bounding box.
    /// @param _b The boundary of interest.
    /// @return A set of cell indexes that contain _b's bounding box.
    std::unordered_set<size_t> LocateBBXCells(const Boundary* const _b) const;

    /// Find the cells that contain a bounding box around two points.
    /// @param _min The low-range values.
    /// @param _max The high-range values.
    /// @return A set of cell indexes that contain the bounding box around _min,
    ///         _max.
    std::unordered_set<size_t> LocateBBXCells(const Point3d& _min,
        const Point3d& _max) const;

    /// Locate the neighbors of a cell which share a facet.
    /// @param _index The cell index.
    /// @return The neighbor cell indexes.
    std::unordered_set<size_t> LocateFacetNeighbors(const size_t _index) const;

    /// Locate the neighbors of a cell which share an edge but not a facet.
    /// @param _index The cell index.
    /// @return The neighbor cell indexes.
    std::unordered_set<size_t> LocateEdgeNeighbors(const size_t _index) const;

    /// Locate the neighbors of a cell which share a vertex but not an edge or
    /// facet.
    /// @param _index The cell index.
    /// @return The neighbor cell indexes.
    std::unordered_set<size_t> LocateVertexNeighbors(const size_t _index) const;

    /// Locate all neighbors of a cell.
    /// @param _index The cell index.
    /// @return The neighbor cell indexes.
    std::unordered_set<size_t> LocateAllNeighbors(const size_t _index) const;

    ///@}
    ///@name Cell Properties
    ///@{

    /// Get the length of the grid cells.
    double CellLength() const noexcept;

    /// Get the center of a cell.
    /// @param _index The cell index.
    /// @return The center of cell _index.
    Point3d CellCenter(const size_t _index) const noexcept;

    ///@}
    ///@name Decomposition Mapping
    ///@{

    /// Create a map from grid cell index to the set of decomposition regions
    /// which are near to or touch the grid cell.
    /// @param _decomposition The workspace decomposition object to map.
    /// @param _useCollisionDetection Use PQP collision detection to refine the
    ///                               test?
    /// @return A mapping m, where m[i] gives a set of _decomposition regions.
    ///         If not using PQP then the regions are those whos BBX touches
    ///         grid cell i, otherwise the regions actually touch cell i.
    DecompositionMap ComputeDecompositionMap(
        const WorkspaceDecomposition* const _decomposition,
        const bool _useCollisionDetection = false) const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Get the triple of x,y,z indexes that define the cell containing a point.
    /// @param _p The point of interest.
    /// @return The x,y,z indexes of the cell holding _p.
    IndexSet Cell(const Point3d& _p) const noexcept;

    /// Get the cell index from x,y,z indexes.
    size_t CellIndex(const size_t _x, const size_t _y, const size_t _z) const
      noexcept;

    /// @overload
    size_t CellIndex(const IndexSet& _indexes) const noexcept;

    /// Get the z-index of a cell index.
    size_t ZIndex(const size_t _index) const noexcept;

    /// Get the y-index of a cell index.
    size_t YIndex(const size_t _index) const noexcept;

    /// Get the x-index of a cell index.
    size_t XIndex(const size_t _index) const noexcept;

    /// Verify that a set of X, Y, Z indexes are valid for this grid.
    /// @param _indexes The X, Y, Z indexes for the cell in question.
    /// @return True if _indexes represents a valid cell in this grid.
    bool InGrid(const IndexSet& _indexes) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    const Boundary* const m_boundary; ///< The boundary of the grid.

    const double m_length;            ///< Length of a cell.
    size_t m_num[3];                  ///< The number of cells in each dimension.

    static constexpr bool m_debug{false};  ///< Enable debugging messages?

    mutable std::unique_ptr<GMSPolyhedron> m_polyhedron; ///< Box model 1.

    ///@}

  public:

    ///@name Testing
    ///@{

    /// Test this object.
    /// @param _trials Sample this number of grid cells to test. Or, use zero to
    ///                test all cells.
    /// @throw If the test fails.
    void Test(const size_t _trials = 0) const;

    ///@}

};

#endif
