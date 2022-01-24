#include "WorkspacePortal.h"
#include "WorkspaceDecomposition.h"
#include "WorkspaceRegion.h"

/*--------------------------- Construction -----------------------------------*/

WorkspacePortal::
WorkspacePortal() : m_sourceIndex(-1), m_targetIndex(-1),
    m_weight(std::numeric_limits<double>::infinity()) { }


WorkspacePortal::
WorkspacePortal(WorkspaceDecomposition* const _wd, const size_t _s,
    const size_t _t) : m_decomposition(_wd), m_sourceIndex(_s),
    m_targetIndex(_t), m_weight(ComputeWeight()) {
}


void
WorkspacePortal::
SetDecomposition(WorkspaceDecomposition* const _wd) {
  m_decomposition = _wd;
}

/*-------------------------------- Accessors ---------------------------------*/

const size_t
WorkspacePortal::
GetSourceDescriptor() const noexcept {
  return m_sourceIndex;
}


const size_t
WorkspacePortal::
GetTargetDescriptor() const noexcept {
  return m_targetIndex;
}


const WorkspaceRegion&
WorkspacePortal::
GetSource() const noexcept {
  return m_decomposition->GetRegion(m_sourceIndex);
}


const WorkspaceRegion&
WorkspacePortal::
GetTarget() const noexcept {
  return m_decomposition->GetRegion(m_targetIndex);
}


double
WorkspacePortal::
GetWeight() const noexcept {
  return m_weight;
}

/*-------------------------------- Queries -----------------------------------*/

const std::vector<Point3d>
WorkspacePortal::
FindPoints() const {
  return GetSource().FindSharedPoints(GetTarget());
}


const std::vector<const WorkspacePortal::Facet*>
WorkspacePortal::
FindFacets() const {
  return GetSource().FindSharedFacets(GetTarget());
}

/*--------------------------------- Helpers ----------------------------------*/

double
WorkspacePortal::
ComputeWeight() const {
  double bestDistance = std::numeric_limits<double>::infinity();

  const Point3d sourceCenter = GetSource().FindCenter(),
                targetCenter = GetTarget().FindCenter();

  for(auto facet : FindFacets()) {
    const Point3d facetCenter = facet->FindCenter();
    const double distance = (sourceCenter - facetCenter).norm()
                          + (facetCenter - targetCenter).norm();
    bestDistance = std::min(bestDistance, distance);
  }

  return bestDistance;
}

/*----------------------------------------------------------------------------*/
