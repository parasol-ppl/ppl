#include <algorithm>

#include "WorkspaceRegion.h"
#include "WorkspaceDecomposition.h"

#include "Geometry/Boundaries/Boundary.h"


/*------------------------------ Construction --------------------------------*/

WorkspaceRegion::
WorkspaceRegion() = default;


WorkspaceRegion::
WorkspaceRegion(WorkspaceDecomposition* const _wd) : m_decomposition(_wd) {}


WorkspaceRegion::
WorkspaceRegion(const WorkspaceRegion& _other) {
  *this = _other;
}


WorkspaceRegion::
WorkspaceRegion(WorkspaceRegion&& _other) {
  *this = std::move(_other);
}


WorkspaceRegion::
~WorkspaceRegion() = default;


void
WorkspaceRegion::
SetDecomposition(WorkspaceDecomposition* const _wd) {
  m_decomposition = _wd;
  const auto& pointList = m_decomposition->GetPoints();

  // Update the facets' point lists.
  for(auto& facet : m_facets)
    facet = Facet(facet.begin(), facet.end(), pointList);
}

/*-------------------------------- Assignment --------------------------------*/

WorkspaceRegion&
WorkspaceRegion::
operator=(const WorkspaceRegion& _other) {
  m_decomposition = _other.m_decomposition;
  m_points = _other.m_points;
  m_facets = _other.m_facets;

  if(_other.m_boundary)
    m_boundary = _other.m_boundary->Clone();

  return *this;
}


WorkspaceRegion&
WorkspaceRegion::
operator=(WorkspaceRegion&& _other) = default;

/*--------------------------------- Equality ---------------------------------*/

bool
WorkspaceRegion::
operator==(const WorkspaceRegion& _region) const noexcept {
  return m_decomposition == _region.m_decomposition
     and m_points == _region.m_points
     and m_facets == _region.m_facets;
}


bool
WorkspaceRegion::
operator!=(const WorkspaceRegion& _region) const noexcept {
  return !(*this == _region);
}

/*--------------------------------- Modifiers --------------------------------*/

void
WorkspaceRegion::
AddPoint(const size_t _i) {
  m_points.emplace_back(_i);
}


void
WorkspaceRegion::
AddFacet(Facet&& _f) {
  m_facets.emplace_back(std::move(_f));
}


void
WorkspaceRegion::
AddBoundary(std::unique_ptr<Boundary>&& _b) {
  m_boundary = std::move(_b);
}

/*--------------------------------- Accessors --------------------------------*/

const size_t
WorkspaceRegion::
GetNumPoints() const noexcept {
  return m_points.size();
}


const size_t
WorkspaceRegion::
GetNumFacets() const noexcept {
  return m_facets.size();
}


const Point3d&
WorkspaceRegion::
GetPoint(const size_t _i) const noexcept {
  return m_decomposition->GetPoint(m_points[_i]);
}


const std::vector<Point3d>
WorkspaceRegion::
GetPoints() const noexcept {
  std::vector<Point3d> out;
  for(const auto& index : m_points)
    out.push_back(m_decomposition->GetPoint(index));
  return out;
}


const std::vector<WorkspaceRegion::Facet>&
WorkspaceRegion::
GetFacets() const noexcept {
  return m_facets;
}


const Boundary*
WorkspaceRegion::
GetBoundary() const noexcept {
  return m_boundary.get();
}

/*---------------------------------- Queries ---------------------------------*/

const bool
WorkspaceRegion::
HasPoint(const Point3d& _p) const noexcept {
  for(const auto& index : m_points)
    if(m_decomposition->GetPoint(index) == _p)
      return true;
  return false;
}


const Point3d
WorkspaceRegion::
FindCenter() const noexcept {
  Point3d center;
  for(const auto& index : m_points)
    center += m_decomposition->GetPoint(index);
  center /= m_points.size();
  return center;
}


const std::vector<Point3d>
WorkspaceRegion::
FindSharedPoints(const WorkspaceRegion& _wr) const noexcept {
  // Find shared indexes.
  std::vector<size_t> mine = m_points;
  std::vector<size_t> theirs = _wr.m_points;
  std::sort(mine.begin(), mine.end());
  std::sort(theirs.begin(), theirs.end());

  std::vector<size_t> shared;
  std::set_intersection(mine.begin(), mine.end(), theirs.begin(), theirs.end(),
      std::back_inserter(shared));

  // Get points from indexes.
  std::vector<Point3d> out;
  for(const auto& index : shared)
    out.push_back(m_decomposition->GetPoint(index));

  return out;
}


const std::vector<const WorkspaceRegion::Facet*>
WorkspaceRegion::
FindSharedFacets(const WorkspaceRegion& _wr) const noexcept {
  std::vector<const Facet*> out;
  for(const auto& myFacet : m_facets) {
    // Flip facet orientation before comparing as the normals will be reversed.
    for(auto theirFacet : _wr.m_facets) {
      // Use a copy to maintain const-ness.
      /// @todo Eliminate this extraneous copy. Perhaps check if = reverse within
      ///       facet class.
      theirFacet.Reverse();
      if(myFacet != theirFacet)
        continue;
      out.push_back(&myFacet);
      break;
    }
  }
  return out;
}

/*----------------------------------------------------------------------------*/
