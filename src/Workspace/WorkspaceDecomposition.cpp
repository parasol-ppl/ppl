#include "WorkspaceDecomposition.h"

#include "Geometry/Boundaries/TetrahedralBoundary.h"
#include "Simulator/Conversions.h"
#include "Utilities/SSSP.h"

#include "containers/sequential/graph/algorithms/dijkstra.h"

#include "glutils/obj_file.h"
#include "glutils/triangulated_model.h"

#include <unordered_map>

#ifndef INVALID_VID
#define INVALID_VID (std::numeric_limits<size_t>::max())
#endif


/*------------------------------- Construction -------------------------------*/

WorkspaceDecomposition::
WorkspaceDecomposition() = default;


WorkspaceDecomposition::
WorkspaceDecomposition(const WorkspaceDecomposition& _other) {
  *this = _other;
}


WorkspaceDecomposition::
WorkspaceDecomposition(WorkspaceDecomposition&& _other) {
  *this = std::move(_other);
}


WorkspaceDecomposition::
~WorkspaceDecomposition() = default;

/*-------------------------------- Assignment --------------------------------*/

WorkspaceDecomposition&
WorkspaceDecomposition::
operator=(const WorkspaceDecomposition& _other) {
  // Copy base class parts first.
  static_cast<RegionGraph&>(*this) = static_cast<const RegionGraph&>(_other);

  // Copy extra bits.
  m_points = _other.m_points;
  m_dual = _other.m_dual;

  UpdateDecompositionPointers();

  m_finalized = _other.m_finalized;
  return *this;
}


WorkspaceDecomposition&
WorkspaceDecomposition::
operator=(WorkspaceDecomposition&& _other) {
  // Move base class parts first.
  static_cast<RegionGraph&>(*this) = std::move(static_cast<RegionGraph&>(_other));

  // Move extra bits.
  m_points = std::move(_other.m_points);
  m_dual = std::move(_other.m_dual);

  UpdateDecompositionPointers();

  m_finalized = _other.m_finalized;
  return *this;
}

/*----------------------------- Point Accessors ------------------------------*/

const size_t
WorkspaceDecomposition::
GetNumPoints() const noexcept {
  return m_points.size();
}


const Point3d&
WorkspaceDecomposition::
GetPoint(const size_t _i) const noexcept {
  return m_points[_i];
}


const std::vector<Point3d>&
WorkspaceDecomposition::
GetPoints() const noexcept {
  return m_points;
}

/*----------------------------- Region Accessors -----------------------------*/

const size_t
WorkspaceDecomposition::
GetNumRegions() const noexcept {
  return get_num_vertices();
}


const WorkspaceRegion&
WorkspaceDecomposition::
GetRegion(const size_t _i) const noexcept {
  return find_vertex(_i)->property();
}


const WorkspaceDecomposition::vertex_descriptor
WorkspaceDecomposition::
GetDescriptor(const WorkspaceRegion& _region) const noexcept {
  for(const_vertex_iterator iter = this->begin(); iter != this->end(); ++iter)
    if(iter->property() == _region)
      return iter->descriptor();

  return INVALID_VID;
}

/*----------------------------- Accessors ------------------------------------*/

const WorkspacePortal&
WorkspaceDecomposition::
GetPortal(const size_t _i1, const size_t _i2) const noexcept {
  const_vertex_iterator vert;
  const_adj_edge_iterator edge;
  RegionGraph::edge_descriptor ed(_i1, _i2);
  find_edge(ed, vert, edge);
  return edge->property();
}

/*---------------------------- Dual Graph Accessors --------------------------*/

WorkspaceDecomposition::DualGraph&
WorkspaceDecomposition::
GetDualGraph() noexcept {
  return m_dual;
}


const WorkspaceDecomposition::DualGraph&
WorkspaceDecomposition::
GetDualGraph() const noexcept {
  return m_dual;
}

/*----------------------------- Modifiers ------------------------------------*/

void
WorkspaceDecomposition::
AddPoint(const Point3d& _p) {
  AssertMutable();
  m_points.push_back(_p);
}


void
WorkspaceDecomposition::
AddTetrahedralRegion(const int _pts[4]) {
  AssertMutable();
  WorkspaceRegion wr(this);

  // Add points to the region.
  for(size_t i = 0; i < 4; ++i)
    wr.AddPoint(_pts[i]);

  // Add facets to the region. Create a facet for each combination of points,
  // and ensure that their normals face away from the center point.
  const Point3d center = wr.FindCenter();
  for(size_t i = 0; i < 4; ++i) {
    WorkspaceRegion::Facet f(_pts[i], _pts[(i + 1) % 4], _pts[(i + 2) % 4],
        m_points);
    f.ComputeNormal();
    if(f.PointIsAbove(center))
      f.Reverse();
    wr.AddFacet(std::move(f));
  }

  // Create a tetrahedral boundary object for the region.
  /// @todo The boundary currently double-stores the points. We would like to
  /// have a non-mutable boundary that holds only references to the points.
  wr.AddBoundary(std::unique_ptr<TetrahedralBoundary>(
      new TetrahedralBoundary(wr.GetPoints())
  ));

  // Add region to the graph.
  add_vertex(std::move(wr));
}


void
WorkspaceDecomposition::
AddPortal(const size_t _s, const size_t _t) {
  AssertMutable();
  add_edge(_s, _t, WorkspacePortal(this, _s, _t));
}


void
WorkspaceDecomposition::
Finalize() {
  ComputeDualGraph();
  m_finalized = true;
}

/*------------------------------------ I/O -----------------------------------*/

void
WorkspaceDecomposition::
Print(std::ostream& _os) const {
  // Print counts.
  _os << "WorkspaceDecomposition " << this
      << "\n\tNum vertices: " << this->get_num_vertices()
      << "\n\tNum edges: " << this->get_num_edges();

  for(auto vi = this->begin(); vi != this->end(); ++vi) {
    _os << "\n\t  Vertex " << vi->descriptor() << " " << &vi->property();
    for(auto ei = vi->begin(); ei != vi->end(); ++ei)
      _os << "\n\t    Edge to " << ei->target();
  }

  _os << std::endl;
}


void
WorkspaceDecomposition::
WriteObj(const std::string& _filename) const {
  // Create a triangulated model.
  glutils::triangulated_model t;

  // Add each region.
  for(auto vi = this->begin(); vi != this->end(); ++vi) {
    // Get the facets.
    const std::vector<WorkspaceRegion::Facet>& facets = vi->property().GetFacets();

    // Add each facet to the model.
    for(const auto& facet : facets)
      t.add_facet(t.add_point(ToGLUtils(facet.GetPoint(0))),
                  t.add_point(ToGLUtils(facet.GetPoint(1))),
                  t.add_point(ToGLUtils(facet.GetPoint(2))));
  }

  // Output the model to an obj file.
  glutils::obj_file file(_filename);
  file << t;
}

/*------------------------------- Path Finding -------------------------------*/

std::vector<size_t>
WorkspaceDecomposition::
FindPath(const size_t _source, const size_t _target) const {
  if(_source == INVALID_VID or _target == INVALID_VID)
    return {};

  /// @todo Remove the const-cast after STAPL fixes its API.
  WorkspaceDecomposition* wd = const_cast<WorkspaceDecomposition*>(this);

  // Set up an early termination criterion so that we quit early if we find the
  // _target node.
  SSSPTerminationCriterion<WorkspaceDecomposition> quit(
      [_target](typename WorkspaceDecomposition::vertex_iterator& _vi,
         const SSSPOutput<WorkspaceDecomposition>& _sssp) {
        return _vi->descriptor() == _target ? SSSPTermination::EndSearch
                                            : SSSPTermination::Continue;
      }
  );
  const SSSPOutput<WorkspaceDecomposition> sssp = DijkstraSSSP(wd, {_source}, quit);

  // If the end node has no parent, there is no path.
  if(!sssp.parent.count(_target))
    return {};

  // Extract the path.
  std::vector<vertex_descriptor> path;
  path.push_back(_target);

  vertex_descriptor current = _target;
  do {
    current = sssp.parent.at(current);
    path.push_back(current);
  } while(current != _source);
  std::reverse(path.begin(), path.end());

  return path;
}


std::vector<size_t>
WorkspaceDecomposition::
FindPath(const WorkspaceRegion* const _source,
    const WorkspaceRegion* const _target) const {
  return FindPath(GetDescriptor(*_source), GetDescriptor(*_target));
}


std::vector<size_t>
WorkspaceDecomposition::
FindNeighborhood(const std::vector<size_t>& _roots, const double _threshold) const {
  /// @todo Remove the const-cast after STAPL fixes its API.
  WorkspaceDecomposition* wd = const_cast<WorkspaceDecomposition*>(this);

  // Define a path weight function that skips targets if the new distance
  // exceeds the neighborhood threshold.
  SSSPPathWeightFunction<WorkspaceDecomposition> weight(
      [&_threshold](adj_edge_iterator& _ei,
                    const double _sourceDistance,
                    const double _targetDistance) {
        const double edgeWeight  = _ei->property().GetWeight(),
                     newDistance = _sourceDistance + edgeWeight;

        if(newDistance > _threshold)
          return std::numeric_limits<double>::infinity();
        return newDistance;
      }
  );

  // Run SSSP from the set of roots to find the neighborhood. Return the set of
  // all discovered nodes.
  SSSPOutput<WorkspaceDecomposition> sssp = DijkstraSSSP(wd, _roots, weight);
  return sssp.ordering;
}

/*------------------------------ Helpers -------------------------------------*/

void
WorkspaceDecomposition::
AssertMutable() const {
  if(m_finalized)
    throw PMPLException("WorkspaceDecomposition error", WHERE, "Can't modify "
        "the decomposition after it has been finalized.");
}


void
WorkspaceDecomposition::
ComputeDualGraph() {
  // Make vertices, ensuring that descriptors match.
  for(const_vertex_iterator vit = begin(); vit != end(); ++vit)
    m_dual.add_vertex(vit->descriptor(), vit->property().FindCenter());

  // Make edges.
  for(const_edge_iterator iter = edges_begin(); iter != edges_end(); ++iter) {
    size_t s = iter->source();
    size_t t = iter->target();
    double length = (m_dual.find_vertex(t)->property() -
                     m_dual.find_vertex(s)->property()).norm();
    m_dual.add_edge(s, t, length);
  }
}


void
WorkspaceDecomposition::
UpdateDecompositionPointers() {
  for(auto vi = this->begin(); vi != this->end(); ++vi) {
    vi->property().SetDecomposition(this);
    for(auto ei = vi->begin(); ei != vi->end(); ++ei)
      ei->property().SetDecomposition(this);
  }
}

/*----------------------------------------------------------------------------*/
