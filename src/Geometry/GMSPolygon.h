#ifndef GMS_POLYGON_H_
#define GMS_POLYGON_H_

#include <cstddef>
#include <utility>
#include <vector>

#include "Orientation.h"
#include "Vector.h"
using namespace mathtool;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Geometry
/// Geometric structure for polygons, including vertex indexes, normal, and
/// area. The vertex indexes refer to an external vertex list, which is required
/// for accessing the points through this object.
///
/// Since we require triangulated models, this usually represents a triangle.
////////////////////////////////////////////////////////////////////////////////
class GMSPolygon {

  public:

    ///@name Local Types
    ///@{

    typedef typename std::vector<Point3d>      PointList;
    typedef typename std::vector<int>          IndexList;

    typedef typename IndexList::const_iterator const_iterator;

    ///@}
    ///@name Construction
    ///@{

    /// Default construction is provided for compatibility with STL
    /// containers only. It should not be called otherwise.
    GMSPolygon();

    /// Construct a polygon (triangle) from three vertex indexes.
    /// @param _v1 The first vertex index.
    /// @param _v2 The second vertex index.
    /// @param _v3 The third vertex index.
    /// @param _pts The vector holding the vertex data.
    GMSPolygon(const int _v1, const int _v2, const int _v3,
        const PointList& _pts);

    /// Construct a polygon from a range of vertex indexes.
    /// @param _begin An iterator to the front of the range (first index).
    /// @param _end An iterator to one-past-the-last of the range.
    /// @param _pts The vector holding the vertex data.
    GMSPolygon(const_iterator _begin, const_iterator _end,
        const PointList& _pts);

    ///@}
    ///@name Iterators
    ///@{
    /// Iterate over the vertex indices in this polygon.

    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;

    ///@}
    ///@name Accessors
    ///@}

    /// Access a vertex index in the polygon.
    /// @param _i The vertex index in range [0 : n - 1].
    /// @return The _i'th vertex index in this polygon.
    const int& operator[](const size_t _i) const noexcept;

    /// Get the number of vertices in the polygon.
    size_t GetNumVertices() const noexcept;

    /// Get the vertex referenced by the _i'th index in the vertex list.
    /// @param _i The vertex index in range [0 : n - 1].
    /// @return The vertex which is the _i'th point in this polygon.
    const Point3d& GetPoint(const size_t _i) const noexcept;

    /// Get the polygon's normal.
    Vector3d& GetNormal() noexcept;
    const Vector3d& GetNormal() const noexcept;

    /// Get the polygon's area.
    double GetArea() const noexcept;

    ///@}
    ///@name Modifiers
    ///@{

    void Reverse();       ///< Reverse the facing of this polygon.
    void ComputeNormal(); ///< Compute the normal and area for this polygon.

    ///@}
    ///@name Equality
    ///@{

    bool operator==(const GMSPolygon& _p) const noexcept;
    bool operator!=(const GMSPolygon& _p) const noexcept;

    ///@}
    ///@name Ordering
    ///@{
    /// Define an ordering of polygons based on surface area.

    bool operator<(const GMSPolygon& _other) const noexcept;
    bool operator>(const GMSPolygon& _other) const noexcept;

    ///@}
    ///@name Queries
    ///@{

    /// Test for three unique vertex indexes.
    const bool IsTriangle() const noexcept;

    /// Find the centroid of this polygon.
    const Point3d FindCenter() const noexcept;

    /// Test whether a point lies above or below the polygon plane. A
    /// point is considered above the plane when it is on the side where
    /// the normal faces outward.
    /// @param _p The point to check.
    const bool PointIsAbove(const Point3d& _p) const noexcept;

    /// Find a common vertex between two polygons.
    /// @param _p The other polygon under consideration.
    /// @return A common vertex index, or -1 if none exists.
    const int CommonVertex(const GMSPolygon& _p) const noexcept;

    /// Find the common edge between two polygons.
    /// @param _p The other polygon under consideration.
    /// @return A pair of vertex indexes if a common edge exists. If not, at
    ///         least one of the indexes will be -1.
    const std::pair<int, int> CommonEdge(const GMSPolygon& _p) const noexcept;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Rotate the vertex list so that the lowest-index point is always first.
    /// This is required to ensure that polygons constructed from points
    /// {1, 2, 3} and {2, 3, 1}, which represent the same structure, compare as
    /// equal.
    void AlignIndexes() noexcept;

    ///@}
    ///@name Internal State
    ///@{

    IndexList m_indexes;  ///< The vertex indexes in this polygon.
    Vector3d m_normal;    ///< The normal vector of this polygon.
    double m_area{0};     ///< Area of this polygon.

    /// The structure holding the points referenced by m_indexes.
    const PointList* m_pointList{nullptr};

    ///@}
};

#endif
