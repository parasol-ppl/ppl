#include "WorkspaceSkeleton.h"

#include <containers/sequential/graph/algorithms/graph_input_output.h>

#include "nonstd/io.h"
/*--------------------------------- Locators ---------------------------------*/

WorkspaceSkeleton::vertex_iterator
WorkspaceSkeleton::
FindNearestVertex(const mathtool::Point3d& _target) {
  double closestDistance = std::numeric_limits<double>::max();
  vertex_iterator closestVI;

  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit) {
    const double distance = (vit->property() - _target).norm();
    if(distance < closestDistance) {
      closestDistance = distance;
      closestVI = vit;
    }
  }
  return closestVI;
}

WorkspaceSkeleton::vertex_iterator
WorkspaceSkeleton::
find_vertex(WorkspaceSkeleton::VD _vd){
  return m_graph.find_vertex(_vd);
}


WorkspaceSkeleton::vertex_iterator
WorkspaceSkeleton::
FindVertex(const VD _vertexDescriptor) {
  return m_graph.find_vertex(_vertexDescriptor);
}


WorkspaceSkeleton::adj_edge_iterator
WorkspaceSkeleton::
FindEdge(const ED& _ed) {
  vertex_iterator vit;
  adj_edge_iterator eit;
  if(!m_graph.find_edge(_ed, vit, eit))
    throw RunTimeException(WHERE) << "Requested non-existing edge ("
                                  << _ed.id() << "|" << _ed.source()
                                  << "," << _ed.target() << ")";
  return eit;
}


std::vector<WorkspaceSkeleton::adj_edge_iterator>
WorkspaceSkeleton::
FindInboundEdges(const VD& _vertexDescriptor) {
  return FindInboundEdges(m_graph.find_vertex(_vertexDescriptor));
}


std::vector<WorkspaceSkeleton::adj_edge_iterator>
WorkspaceSkeleton::
FindInboundEdges(const vertex_iterator& _vertexIter) {
  std::vector<adj_edge_iterator> inEdges;

  // stapl's directed_preds_graph only tells us the predecessors; it doesn't let
  // us iterate over inbound edges. Because we have multi-edges, finding all
  // inbound edges thus requires that we iterate over all out-bound edges of the
  // predecessors and collect their iterators.
  const std::vector<VD>& predecessors = _vertexIter->predecessors();
  for(const auto pred : predecessors) {
    auto predIter = m_graph.find_vertex(pred);
    for(auto eit = predIter->begin(); eit != predIter->end(); ++eit)
      if(eit->target() == _vertexIter->descriptor())
        inEdges.push_back(eit);
  }

  return inEdges;
}

/*--------------------------------- Modifiers --------------------------------*/

WorkspaceSkeleton
WorkspaceSkeleton::
Direct(const mathtool::Point3d& _start) {
  GraphType skeletonGraph;

  // Define coloring for the breadth-first direction algorithm.
  enum Color {
    White, // The vertex is undiscovered.
    Gray,  // The vertex is discovered but not visited.
    Black  // The vertex is visited.
  };
  std::unordered_map<VD, Color> visited;

  // Copy nodes into the new directed skeleton, preserving the descriptors.
  // Also mark each node as undiscovered.
  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit) {
    skeletonGraph.add_vertex(vit->descriptor(), vit->property());
    visited[vit->descriptor()] = White;
  }

  // Specialized BFS to make flow network
  //
  // Differs from regular BFS because:
  // - Treats skeleton as undirected graph even though it is directed. We do
  //   this because we may want to re-direct a skeleton from a different vertex.
  // - Computes a graph instead of BFS tree, i.e., cross edges are added
  //
  // Also note that the skeleton is a multi-edge graph, so there may be multiple
  // edges between any two vertices.

  // Find the vertex that is nearest to _start, and copy edges from the original
  // skeleton so that all flow away from it.
  VD closest = FindNearestVertex(_start)->descriptor();

  std::queue<VD> q;
  q.push(closest);
  while(!q.empty()) {
    // Get the next node in the queue and visit it.
    VD current = q.front();
    q.pop();
    visited[current] = Black;

    auto currentIter = m_graph.find_vertex(current);

    // Copy edges leaving the current vertex, and mark their targets as
    // discovered.
    for(auto eit = currentIter->begin(); eit != currentIter->end(); ++eit) {
      VD target = eit->target();
      switch(visited.at(target)) {
        case White:
          // Node was undiscovered, discover it and continue with the copy.
          visited[target] = Gray;
          q.push(target);
        case Gray:
          // Node was previously discovered but hasn't been visited yet. Copy
          // the path from current to target into the directed skeleton.
          skeletonGraph.add_edge(eit->descriptor(), eit->property());
          break;
        default:
          // Target node was previously visited. We don't copy the edge because
          // the target is already on a shorter directed path from the start to
          // the current vertex.
          break;
      }
    }

    // Copy edges inbound on the current vertex, and mark their sources as
    // discovered.
    const auto inEdges = FindInboundEdges(currentIter);
    for(auto pit : inEdges) {
      VD source = pit->source();
      switch(visited.at(source)) {
        case White:
          // Node was undiscovered, discover it and continue with the copy.
          visited[source] = Gray;
          q.push(source);
        case Gray:
          {
            // Node was previously discovered but hasn't been visited yet.
            // Copy the reversed edge (from source to current) into the
            // directed skeleton.
            const std::vector<mathtool::Point3d>& path = pit->property();
            skeletonGraph.add_edge(ED(current, source),
                std::vector<mathtool::Point3d>(path.rbegin(), path.rend()));
            break;
          }
        default:
          // Source node was previously visited. We don't copy the edge
          // because the source is already on a shorter directed path to
          // the current vertex.
          break;
      }
    }
  }

  WorkspaceSkeleton skeleton;
  skeleton.SetGraph(skeletonGraph);
  skeleton.m_start = closest;
  return skeleton;
}


void
WorkspaceSkeleton::
Prune(const mathtool::Point3d& _goal) {
  // This function only makes sense after calling Direct. Ensure that we've set
  // m_start to something valid.
  if(m_start == std::numeric_limits<VD>::max())
    throw RunTimeException(WHERE, "Cannot prune the skeleton without directing "
        "it first.");

  // Find the vertex in the skeleton that is closest to the goal point.
  auto iter = FindNearestVertex(_goal);
  if(iter->descriptor() == m_start)
    throw RunTimeException(WHERE, "Requested pruning of workspace skeleton "
        "where the start and goal points are nearest the same vertex. This "
        "produces an empty skeleton.");

  if(m_debug)
    std::cout << "WorkspaceSkeleton:: pruning for goal point " << _goal
              << " (VID " << iter->descriptor() << ")."
              << std::endl;

  // Initialize a list of vertices to prune with every vertex in the graph.
  std::vector<VD> toPrune;
  toPrune.reserve(m_graph.get_num_vertices());
  for(const auto& v : m_graph)
    toPrune.push_back(v.descriptor());

  // Remove vertices from the prune list by starting from the goal and working
  // backwards up the incoming edges. Don't prune any vertex that is an ancestor
  // of the goal.
  std::queue<VD> q;
  q.push(iter->descriptor());
  do {
    // back track every vertex in the flow graph
    VD current = q.front();
    q.pop();
    // try to find current vertex in the toPrune list
    // i.e., the list in which all vertices are not currently found to be an
    // ancestor yet
    auto iter = std::find(toPrune.begin(), toPrune.end(), current);
    // if found, we can erase it from the list so that we won't be pruning it
    if(iter != toPrune.end())
      toPrune.erase(iter);
    // if not found, just keep it there

    // the backtrack BFS logic
    for(auto ancestor : m_graph.find_vertex(current)->predecessors())
      q.push(ancestor);
  } while(!q.empty());

  // Remove the vertices we aren't keeping.
  for(auto vd : toPrune)
    if(m_graph.find_vertex(vd) != m_graph.end())
      m_graph.delete_vertex(vd);

  if(m_debug)
    std::cout << "\tPruned " << toPrune.size() << " vertices."
              << std::endl;
}

void
WorkspaceSkeleton::
RefineEdges(double _maxLength){
  std::vector<std::vector<Point3d>> refinedVertices;
  //std::vector<WorkspaceSkeleton::ED> originalEdges;
  //for(auto vi = m_graph.begin(); vi != m_graph.end(); vi++){
    //for(auto ei = vi->begin(); ei != vi->end(); ei++){
    for(auto ei = m_graph.edges_begin(); ei != m_graph.edges_end(); ei++){
      auto source = m_graph.find_vertex(ei->source())->property();
      auto target = m_graph.find_vertex(ei->target())->property();
      //original_edges.emplace_back(source,target);
      //if(target[2] != 0 or source[2] != 0)
      //  continue; //This is a hack because invalid edges are being examined
      const double distance = (source - target).norm();
      if(distance < _maxLength)
        continue;
      //size_t divisions = (size_t)std::ceil(distance/_maxLength);
      std::vector<Point3d> newVertices = {m_graph.find_vertex(ei->source())->property()};
      auto& intermediates = ei->property();
      double currentDistance = 0;
      for(size_t i = 1; i < intermediates.size(); i++){
        double step = (intermediates[i-1] - intermediates[i]).norm();
        currentDistance += step;
        //newVertices.push_back(m_graph.add_vertex(intermediates[i*divisions]));
        if(currentDistance > _maxLength){
          newVertices.push_back(intermediates[i-1]);
          currentDistance = step;
        }
      }
      newVertices.push_back(m_graph.find_vertex(ei->target())->property());
      refinedVertices.push_back(newVertices);
    }
  //}
  for(auto edge : refinedVertices){
    size_t firstVID = FindNearestVertex(edge[0])->descriptor();
    size_t vd1 = firstVID;
    size_t vd2 = firstVID;
    for(size_t i = 1; i < edge.size()-1; i++){
      vd2 = m_graph.add_vertex(edge[i]);
      m_graph.add_edge(vd1,vd2);
      vd1 = vd2;;
    }
    size_t lastVID = FindNearestVertex(edge.back())->descriptor();
    m_graph.add_edge(vd2,lastVID);
    m_graph.delete_edge(firstVID,lastVID);
  }
}


void
WorkspaceSkeleton::
DoubleEdges() {
  for(auto vi = m_graph.begin(); vi != m_graph.end(); ++vi) {
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      // Make a reverse descriptor.
      ED reverseEd = reverse(ei->descriptor());

      // If it already exists in the graph, then this edge was added here by
      // reversing an original edge. Skip.
      vertex_iterator vit;
      adj_edge_iterator eit;
      const bool exists = m_graph.find_edge(reverseEd, vit, eit);
      if(exists)
        continue;

      // Get the path and make a reversed copy.
      auto path = ei->property();
      std::reverse(path.begin(), path.end());

      // Add an edge in reverse orientation with the same edge ID.
      add_internal_edge(m_graph, reverseEd, path);
    }
  }
}


/*--------------------------- Setters & Getters ------------------------------*/

void
WorkspaceSkeleton::
SetGraph(GraphType& _graph) noexcept {
  this->m_graph = _graph;
}


WorkspaceSkeleton::GraphType&
WorkspaceSkeleton::
GetGraph() noexcept {
  return m_graph;
}


const WorkspaceSkeleton::GraphType&
WorkspaceSkeleton::
GetGraph() const noexcept {
  return m_graph;
}

/*------------------------------------- I/O helpers ---------------------------------*/

void
WorkspaceSkeleton::
Write(const std::string& _file) {
  std::ofstream ofs(_file);

  ofs << m_graph.get_num_vertices() << " " << m_graph.get_num_edges() << std::endl;

  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit)
    ofs << vit->descriptor() << " " << vit->property() << std::endl;

  for(auto eit = m_graph.edges_begin(); eit != m_graph.edges_end(); ++eit)	{
    ofs << eit->source() << " " << eit->target() << " ";
    auto prop = eit->property();
    ofs << prop.size() << " ";
    for(auto v: prop)
      ofs << v << " ";
    ofs << std::endl;
  }
  ofs.close();
}

void
WorkspaceSkeleton::
Read(const std::string& _file) {
  std::ifstream ifs(_file);

  size_t nVerts, nEdges;

  ifs >> nVerts >> nEdges;

  for(size_t vit = 0 ; vit != nVerts; ++vit) {
    size_t id;
    Point3d data;
    ifs >> id >> data;
    m_graph.add_vertex(data);
  }

  for(size_t eit = 0; eit != nEdges; ++eit) {
    size_t source, target, propSize;
    std::vector<Point3d> edgeProperty;
    ifs >> source >> target >> propSize;
    for (size_t propit = 0; propit < propSize; ++propit) {
      Point3d prop;
      ifs >> prop;
      edgeProperty.push_back(prop);
    }
    m_graph.add_edge(source, target, edgeProperty);
    edgeProperty.clear();
  }
}

/*----------------------------------------------------------------------------*/
