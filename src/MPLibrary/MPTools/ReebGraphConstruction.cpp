#include "ReebGraphConstruction.h"

#include <queue>
#include <unordered_map>

#include <containers/sequential/graph/algorithms/dijkstra.h>
#include <containers/sequential/graph/algorithms/graph_input_output.h>

#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"
#include "Workspace/WorkspaceDecomposition.h"
#include "Workspace/WorkspaceSkeleton.h"
#include "Utilities/IOUtils.h"
#include "Utilities/XMLNode.h"


/*------------------------- Static Initializers ------------------------------*/

ReebGraphConstruction::Parameters ReebGraphConstruction::m_defaultParams;

/*------------------------------- Construction -------------------------------*/

ReebGraphConstruction::
ReebGraphConstruction() : m_params(m_defaultParams) {}


void
ReebGraphConstruction::
SetDefaultParameters(XMLNode& _node) {
  auto& filename = m_defaultParams.filename;
  filename = _node.Read("filename", false, filename,
      "Filename for read or write ReebGraph operations.");

  // If the filename isn't empty, prepend the base path from the XML file.
  if(!filename.empty())
    filename = GetPathName(_node.Filename()) + filename;

  m_defaultParams.write = _node.Read("write", false, m_defaultParams.write,
      "Write Reeb Graph to file");

  m_defaultParams.read = _node.Read("read", false, m_defaultParams.read,
      "Read in Reeb Graph to file");

  m_defaultParams.debug = _node.Read("debug", false, m_defaultParams.debug,
      "Show debug messages?");
}

/*------------------------------- Operations ---------------------------------*/

void
ReebGraphConstruction::
Construct(const WorkspaceDecomposition* _decomposition) {
  if(!m_params.filename.empty() && m_params.read)
    Read(m_params.filename);
  else {
    Initialize(_decomposition);
    Construct();
    Embed(_decomposition);

    if(!m_params.filename.empty() && m_params.write)
      Write(m_params.filename);
  }
}


WorkspaceSkeleton
ReebGraphConstruction::
GetSkeleton() {
  typedef WorkspaceSkeleton::GraphType Graph;
  Graph g;

  // Copy vertices.
  for(auto vit = m_reebGraph.begin(); vit != m_reebGraph.end(); ++vit)
    g.add_vertex(vit->descriptor(), vit->property().m_vertex);

  // Copy edges.
  for(auto eit = m_reebGraph.edges_begin(); eit != m_reebGraph.edges_end(); ++eit)
  {
    // Make sure this is a real path.
    if(eit->property().m_path.empty()) {
      if(m_params.debug)
        std::cout << "ReebGraph:: skipping empty path when building skeleton."
                  << std::endl;
      continue;
    }
    g.add_edge(eit->descriptor(), eit->property().m_path);
  }

  WorkspaceSkeleton skeleton;
  skeleton.SetGraph(g);
  if(!m_params.filename.empty() && m_params.write)
    skeleton.Write("skeleton_" + m_params.filename);
  return skeleton;
}

/*----------------------------------- I/O ------------------------------------*/

void
ReebGraphConstruction::
Read(const string& _filename) {
  ifstream ifs(_filename);
  m_reebGraph.clear();
  m_reebGraph.set_lazy_update(true);
  stapl::sequential::read_graph(m_reebGraph, ifs);
  m_reebGraph.set_lazy_update(false);
}


void
ReebGraphConstruction::
Write(const string& _filename) {
  ofstream ofs(_filename);
  stapl::sequential::write_graph(m_reebGraph, ofs);
}

/*----------------------------------- I/O ------------------------------------*/

void
ReebGraphConstruction::
Initialize(const WorkspaceDecomposition* _tetrahedralization) {
  // Copy vertices.
  m_vertices = _tetrahedralization->GetPoints();

  // Copy triangular faces from tetrahedrons. We want a mapping from each
  // triangle to the pair of tetrahedra bordering it.
  map<Triangle, unordered_set<size_t>> triangles;
  size_t count = 0;
  for(auto tetra = _tetrahedralization->begin();
      tetra != _tetrahedralization->end(); ++tetra) {
    for(const auto& facet : tetra->property().GetFacets()) {
      // Sort point indexes so opposite-facing triangles have the same
      // representation.
      int v[3] = {facet[0], facet[1], facet[2]};
      sort(v, v + 3);
      triangles[Triangle(v[0], v[1], v[2])].insert(count);
    }
    ++count;
  }

  m_triangles.reserve(triangles.size());
  copy(triangles.begin(), triangles.end(), back_inserter(m_triangles));
}


void
ReebGraphConstruction::
Construct() {
  /// Morse function over model (currently height)
  /// @param _v Vertex
  /// @todo Generalize to allow something other than height
  auto f = [](const Vector3d& _v) {
    return _v[1];
  };

  // Add all vertices of tetrahedralization
  for(size_t i = 0; i < m_vertices.size(); ++i)
    CreateNode(i, m_vertices[i], f(m_vertices[i]));

  // Precompute absolute ordering of vertices to optimize vertex comparison
  vector<size_t> order;
  for(size_t i = 0; i < m_vertices.size(); ++i)
    order.emplace_back(i);
  ReebNodeComp rnc;
  sort(order.begin(), order.end(), [&](size_t _a, size_t _b) {
      return rnc(m_reebGraph.find_vertex(_a)->property(),
                 m_reebGraph.find_vertex(_b)->property());
  });
  for(size_t i = 0; i < order.size(); ++i)
    m_reebGraph.find_vertex(order[i])->property().m_order = i;

  // Precompute ordering of triangles proper usage of MergePaths function
  for(auto& t : m_triangles) {
    size_t v[3] = {get<0>(t.first), get<1>(t.first), get<2>(t.first)};
    sort(v, v+3, [&](size_t _a, size_t _b) {
        return m_reebGraph.find_vertex(_a)->property().m_order <
            m_reebGraph.find_vertex(_b)->property().m_order;
        });

    t.first = make_tuple(v[0], v[1], v[2]);
  }

  //Add each edge to ReebGraph as separate ReebArc
  for(auto& t : m_triangles) {
    size_t v[3] = {get<0>(t.first), get<1>(t.first), get<2>(t.first)};

    MeshEdge* e0 = CreateArc(v[0], v[2], t.second);
    MeshEdge* e1 = CreateArc(v[0], v[1], t.second);
    MeshEdge* e2 = CreateArc(v[1], v[2], t.second);

    ++e0->m_numTris;
    ++e1->m_numTris;
    ++e2->m_numTris;
  }

  //Call MergePaths on each triangle
  for(auto& t : m_triangles) {
    size_t v[3] = {get<0>(t.first), get<1>(t.first), get<2>(t.first)};

    MeshEdge* e0 = CreateArc(v[0], v[2]);
    MeshEdge* e1 = CreateArc(v[0], v[1]);
    MeshEdge* e2 = CreateArc(v[1], v[2]);

    MergePaths(e0, e1, e2);

    --e0->m_numTris;
    --e1->m_numTris;
    --e2->m_numTris;

    //If edge is done being processed - delete. This is an optimization
    if(e0->m_numTris == 0)
      DeleteMeshEdge(e0);
    if(e1->m_numTris == 0)
      DeleteMeshEdge(e1);
    if(e2->m_numTris == 0)
      DeleteMeshEdge(e2);
  }

  //Remove all 2nodes from graph
  Remove2Nodes();
}


void
ReebGraphConstruction::
CreateNode(size_t _i, const Vector3d& _v, double _w) {
  m_reebGraph.add_vertex(_i, ReebNode(_i, _v, _w));
}


ReebGraphConstruction::MeshEdge*
ReebGraphConstruction::
CreateArc(size_t _s, size_t _t, const unordered_set<size_t>& _tetra) {
  //If edge does not exist add an edge in both tetrahedron edges set and a
  //corresponding reeb arc.
  MeshEdge m(_s, _t, &m_reebGraph);
  if(!m_edges.count(&m)) {
    MeshEdge* m2 = new MeshEdge(_s, _t, &m_reebGraph);
    m_edges.insert(m2);
    RGEID eid = m_reebGraph.add_edge(_s, _t, ReebArc(_s, _t, m2));
    m2->m_arcs.insert(eid);
  }

  //Associate tetrahedrons with edge's reeb arcs
  MeshEdge* e = *m_edges.find(&m);
  for(auto& a : e->m_arcs)
    GetReebArc(a).m_tetra.insert(_tetra.begin(), _tetra.end());

  return e;
}


void
ReebGraphConstruction::
MergePaths(MeshEdge* _e0, MeshEdge* _e1, MeshEdge* _e2) {
  //Merge edges _e0 and _e1 of triangle
  ArcSet& a0 = _e0->m_arcs;
  ArcSet& a1 = _e1->m_arcs;
  GlueByMergeSorting(a0, _e0, a1, _e1);

  //Merge edges _e0 and _e2 of triangle
  ArcSet& a2 = _e0->m_arcs;
  ArcSet& a3 = _e2->m_arcs;
  GlueByMergeSorting(a2, _e0, a3, _e2);
}


void
ReebGraphConstruction::
GlueByMergeSorting(ArcSet& _a0, MeshEdge* _e0, ArcSet& _a1, MeshEdge* _e1) {
  ReebArcComp rac(&m_reebGraph);
  for(auto asit0 = _a0.begin(), asit1 = _a1.begin();
      asit0 != _a0.end() && asit1 != _a1.end();) {

    //Skip same edge
    if(*asit0 == *asit1) {
      ++asit0; ++asit1;
      continue;
    }

    //Not valid edges to merge continue
    if(asit0->source() != asit1->source()) {
      if(rac(*asit0, *asit1))
        ++asit0;
      else
        ++asit1;
      continue;
    }

    ReebNode& n0 = m_reebGraph.find_vertex(asit0->target())->property();
    ReebNode& n1 = m_reebGraph.find_vertex(asit1->target())->property();

    //Merge based on order of target of edges
    if(n0.m_order < n1.m_order) {
      MergeArcs(*asit0, *asit1);
      ++asit0;
      asit1 = _a1.begin();
    }
    else {
      MergeArcs(*asit1, *asit0);
      ++asit1;
      asit0 = _a0.begin();
    }
  }
}


void
ReebGraphConstruction::
MergeArcs(RGEID _a0, RGEID _a1) {
  ReebArc& a0 = GetReebArc(_a0);
  ReebArc& a1 = GetReebArc(_a1);

  //Merge data into a0
  for(auto& edge : a1.m_edges) {
    edge->m_arcs.insert(_a0);
    edge->m_arcs.erase(_a1);
    a0.m_edges.insert(edge);
  }
  a0.m_tetra.insert(a1.m_tetra.begin(), a1.m_tetra.end());

  //Case: targets are not equal. Insert new edge from a0's target to a1's target
  if(_a0.target() != _a1.target()) {
    ReebArc a = a1;
    a.m_source = a0.m_target;
    RGEID neweid = m_reebGraph.add_edge(a.m_source, a.m_target, a);
    for(auto& edge : GetReebArc(neweid).m_edges)
      edge->m_arcs.insert(neweid);
  }

  //Delete a1
  m_reebGraph.delete_edge(_a1);
}


ReebGraphConstruction::ReebArc&
ReebGraphConstruction::
GetReebArc(RGEID _a) {
  ReebGraph::adj_edge_iterator ei;
  ReebGraph::vertex_iterator vi;
  m_reebGraph.find_edge(_a, vi, ei);
  return ei->property();
}


void
ReebGraphConstruction::
DeleteMeshEdge(MeshEdge* _e) {
  for(auto& a : _e->m_arcs)
    GetReebArc(a).m_edges.erase(_e);
}


void
ReebGraphConstruction::
Remove2Nodes() {
  bool removed;
  do {
    removed = false;
    for(auto vit = m_reebGraph.begin(); vit != m_reebGraph.end(); ++vit) {
      //detect 2-node
      if(vit->predecessors().size() == 1 && vit->size() == 1) {
        //get in and out edge
        auto pred = m_reebGraph.find_vertex(vit->predecessors()[0]);
        RGEID in, out;
        for(auto eit : *pred) {
          if(eit.target() == vit->descriptor())
            in = eit.descriptor();
        }
        out = (*vit->begin()).descriptor();

        ReebArc& ain = GetReebArc(in);
        ReebArc& aout = GetReebArc(out);

        //Merge arcs
        RGEID neweid = m_reebGraph.add_edge(ain.m_source, aout.m_target, ReebArc(ain.m_source, aout.m_target));

        ReebArc& newa = GetReebArc(neweid);
        for(auto& edge : ain.m_edges) {
          edge->m_arcs.insert(neweid);
          edge->m_arcs.erase(in);
          newa.m_edges.insert(edge);
        }
        newa.m_tetra.insert(ain.m_tetra.begin(), ain.m_tetra.end());
        for(auto& edge : aout.m_edges) {
          edge->m_arcs.insert(neweid);
          edge->m_arcs.erase(out);
          newa.m_edges.insert(edge);
        }
        newa.m_tetra.insert(aout.m_tetra.begin(), aout.m_tetra.end());

        //Delete old edges and delete old vertex
        m_reebGraph.delete_edge(in);
        m_reebGraph.delete_edge(out);
        m_reebGraph.delete_vertex(vit->descriptor());
        --vit;
        removed = true;
      }
    }
  } while(removed);
}


void
ReebGraphConstruction::
Embed(const WorkspaceDecomposition* _tetrahedralization) {
  /// @TODO We must cast away the constness here due to STAPL problems with
  ///       const-qualified vertex references. If STAPL ever fixes that, we can
  ///       get rid of this.
  WorkspaceDecomposition* t = const_cast<WorkspaceDecomposition*>(_tetrahedralization);
  auto& dualGraph = t->GetDualGraph();

  // Embed ReebNodes in freespace by finding its closest adjacent tetrahedron.
  for(auto rit = m_reebGraph.begin(); rit != m_reebGraph.end(); ++rit) {
    double minDist = numeric_limits<double>::max();
    size_t closestID = -1;
    const Vector3d& reebNode = rit->property().m_vertex;

    // Check each dual graph node for proximity to this reeb graph node.
    for(auto dit = dualGraph.begin(); dit != dualGraph.end(); ++dit) {
      // Make sure the associated workspace region includes this reebNode.
      if(!_tetrahedralization->GetRegion(dit->descriptor()).HasPoint(reebNode))
        continue;
      const double dist = (dit->property() - reebNode).norm();
      if(dist < minDist) {
        minDist = dist;
        closestID = dit->descriptor();
      }
    }
    if(closestID == size_t(-1))
      throw RunTimeException(WHERE, "Could not embed reeb node " +
          std::to_string(rit->descriptor()) + ", which shouldn't happen!");
    rit->property().m_tetra = closestID;
    rit->property().m_vertex = dualGraph.find_vertex(closestID)->property();
  }

  std::set<vector<size_t>> uniquePaths;
  size_t duplicatePathCount = 0;

  // Embed ReebArcs in freespace.
  for(auto eit = m_reebGraph.edges_begin(); eit != m_reebGraph.edges_end();
      ++eit) {
    ReebArc& arc = eit->property();
    arc.m_path.clear();

    // Get start and end vertices. Skip this iteration if they are the same.
    auto sourceit = m_reebGraph.find_vertex(eit->source());
    auto targetit = m_reebGraph.find_vertex(eit->target());
    if(sourceit->property().m_tetra == targetit->property().m_tetra)
      continue;

    // @TODO Disabled as suspect for causing long paths with loops.
    //// Weight all tetrahedron on reeb arc to bias search.
    //auto WeightGraph = [&](const double _factor) {
    //  // Iterate over all tetrahedra in this reeb arc's path.
    //  for(auto tetraid : arc.m_tetra) {
    //    auto vit = dualGraph.find_vertex(tetraid);

    //    // Iterate over all outoing edges in the dual graph.
    //    for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
    //      // Skip neighbors that aren't in this reeb arc's tetrahedra.
    //      if(std::find(arc.m_tetra.begin(), arc.m_tetra.end(), eit->target()) ==
    //          arc.m_tetra.end())
    //        continue;

    //      // Weight the edge to the dualgraph neighbor.
    //      auto targetit = dualGraph.find_vertex(eit->target());
    //      eit->property() = _factor *
    //        (vit->property() - targetit->property()).norm();
    //    }
    //  }
    //};

    //WeightGraph(1e-4);

    // Find path in the dual graph from the start to end node.
    vector<size_t> pathVID;
    stapl::sequential::find_path_dijkstra(
        dualGraph,
        sourceit->property().m_tetra, targetit->property().m_tetra,
        pathVID, numeric_limits<double>::max());

    // Unweight the tetrahedron.
    //WeightGraph(1e4);

    // If the path is empty, something is wrong.
    if(pathVID.empty())
      throw PMPLException("ReebGraph error", WHERE, "Could not find a path "
          "between distinct tetrahera in the dual graph.");

    // Check that we haven't already embedded this path. If so, move on to the
    // next one.
    /// @TODO This really shouldn't be necessary. Needs further investigation
    ///       when time allows.
    const auto check = uniquePaths.insert(pathVID);
    if(!check.second) {
      ++duplicatePathCount;
      continue;
    }

    // For each tetrahedron pair in the path, find their shared facet and insert
    // its mid point into the path for proper transition between the two. This
    // avoids the possibility of clipping obstacles when jumping between
    // tetrahedron midpoints.
    for(auto current = pathVID.begin(), next = current + 1;
        next != pathVID.end(); ++current, ++next) {
      // Add the center of the current tetrahedron to the path.
      arc.m_path.push_back(dualGraph.find_vertex(*current)->property());

      // Get the facets that join the current and next tetrahedron.
      const auto& portal = _tetrahedralization->GetPortal(*current, *next);
      const auto facets = portal.FindFacets();
      if(facets.size() > 1)
        throw PMPLException("ReebGraph error", WHERE, "A portal between "
            "workspace regions has more than one facet, which isn't possible "
            "for tetrahedral regions.");

      // Add the centroid of the joining facet to the path.
      arc.m_path.push_back(facets.front()->FindCenter());
    }

    // Add the center of the last tetrahedron as the end of the path.
    arc.m_path.push_back(dualGraph.find_vertex(pathVID.back())->property());
  }

  if(m_params.debug)
    cout << "ReebGraph:: duplicated path count: " << duplicatePathCount << endl;

  // After embedding the reeb graph, it is possible for multiple reeb graph
  // vertices to be mapped to the same tetrahedron. Resulting in duplicate
  // vertices.
  Uniqueify();
}


void
ReebGraphConstruction::
Uniqueify() {
  typedef ReebGraph::vertex_descriptor VID;
  size_t deletedVertices = 0, deletedEdges = 0;

  // Make a helper to identify incoming edges (stapl's directed_preds graph
  // offers no easy way to do this for multigraphs).
  auto FindInboundEdges = [this](const ReebGraph::vertex_iterator _vertex) {
    std::vector<ReebGraph::adj_edge_iterator> inEdges;

    const auto& predecessors = _vertex->predecessors();
    for(const auto pred : predecessors) {
      auto predIter = m_reebGraph.find_vertex(pred);
      for(auto eit = predIter->begin(); eit != predIter->end(); ++eit)
        if(eit->target() == _vertex->descriptor())
          inEdges.push_back(eit);
    }

    return inEdges;
  };

  // Construct a map of all vertices that have the same embedded point.
  map<Point3d, vector<VID>> duplicateMap;
  for(const auto it : m_reebGraph)
    duplicateMap[it.property().m_vertex].push_back(it.descriptor());

  // Combine all duplicates into one unique vertex.
  for(const auto& it : duplicateMap) {
    // Pick the unique vertex's descriptor as the front of the duplicate set and
    // call it theVID.
    const auto& duplicateVIDs = it.second;
    const auto theVID = duplicateVIDs.front();

    // Re-target all edges touching the duplicates so that all outgoing edges
    // leave theVID and all incoming edges land on theVID.
    for(auto vit = duplicateVIDs.begin() + 1; vit != duplicateVIDs.end(); ++vit) {
      const auto vertex = m_reebGraph.find_vertex(*vit);

      // Add vertex's out-edges to theVID.
      for(auto eit = vertex->begin(); eit != vertex->end(); ++eit)
        m_reebGraph.add_edge(theVID, eit->target(), eit->property());

      // Add vertex's in-edges to theVID.
      auto inEdges = FindInboundEdges(vertex);
      for(auto eit : inEdges)
        m_reebGraph.add_edge(eit->source(), theVID, eit->property());

      // Now the duplicate's edges have been copied to theVID, so we can
      // delete the duplicate vertex (also takes care of its edges).
      m_reebGraph.delete_vertex(*vit);
    }
    deletedVertices += duplicateVIDs.size() - 1;
  }

  // We might now have multiple copies of the same edge leaving the combined
  // vertices. Check for those and remove any we find.
  for(const auto& it : duplicateMap) {
    // Get the surviving vertex from this duplicate set.
    const auto vid = it.second.front();
    const auto vertex = m_reebGraph.find_vertex(vid);

    vector<ReebGraph::edge_descriptor> edgesToDelete;
    std::set<std::vector<Vector3d>> uniquePaths;

    // Check all the outgoing edges and collect the descriptors of those with
    // non-unique paths.
    for(auto eit = vertex->begin(); eit != vertex->end(); ++eit) {
      const auto it = uniquePaths.emplace(eit->property().m_path);
      if(!it.second)
        edgesToDelete.push_back(eit->descriptor());
    }

    uniquePaths.clear();

    // Check all the incoming edges and collect the descriptors of those with
    // non-unique paths.
    auto inEdges = FindInboundEdges(vertex);
    for(auto eit : inEdges) {
      const auto it = uniquePaths.emplace(eit->property().m_path);
      if(!it.second)
        edgesToDelete.push_back(eit->descriptor());
    }

    // Remove edges with non-unique paths.
    for(const auto edgeDescriptor : edgesToDelete)
      m_reebGraph.delete_edge(edgeDescriptor);
    deletedEdges += edgesToDelete.size();
  }

  if(m_params.debug)
    std::cout << "ReebGraph::Uniqueify:: deleted " << deletedVertices
              << " duplicate vertices and " << deletedEdges
              << " duplicate edges."
              << std::endl;
}


/*---------------------------- iostream Operators ----------------------------*/

istream&
operator>>(istream& _is, ReebGraphConstruction::ReebNode& _rn) {
  return _is >> _rn.m_vertexIndex >> _rn.m_vertex >> _rn.m_w >> _rn.m_order;
}


ostream&
operator<<(ostream& _os, const ReebGraphConstruction::ReebNode& _rn) {
  return _os << _rn.m_vertexIndex << " " << _rn.m_vertex << " " << _rn.m_w << " "
             << _rn.m_order;
}


istream&
operator>>(istream& _is, ReebGraphConstruction::ReebArc& _ra) {
  size_t sz;
  _is >> _ra.m_source >> _ra.m_target >> sz;
  Vector3d p;
  _ra.m_path.clear();
  _ra.m_path.reserve(sz);
  for(size_t i = 0; i < sz; ++i) {
    _is >> p;
    _ra.m_path.push_back(p);
  }
  return _is;
}


ostream&
operator<<(ostream& _os, const ReebGraphConstruction::ReebArc& _ra) {
  _os << _ra.m_source << " " << _ra.m_target << " " << _ra.m_path.size();
  for(const auto& p : _ra.m_path)
    _os << " " << p;
  return _os;
}

/*----------------------------------------------------------------------------*/
