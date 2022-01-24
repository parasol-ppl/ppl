#ifndef PMPL_ROADMAP_GRAPH_H_
#define PMPL_ROADMAP_GRAPH_H_

#include "MPProblem/Environment/Environment.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/RuntimeUtils.h"

#ifdef _PARALLEL
#include <containers/graph/dynamic_graph.hpp>
//#include "graph/view/graph_view_property_adaptor.h"
//#include <containers/graph/view/graph_view_property_adaptor.hpp>
#include <containers/sequential/graph/algorithms/connected_components.h>
#include <containers/graph/algorithms/graph_io.hpp>
#else
#include <containers/sequential/graph/graph.h>
#include <containers/sequential/graph/graph_util.h>
#include <containers/sequential/graph/vertex_iterator_adaptor.h>
#include <containers/sequential/graph/algorithms/connected_components.h>
#include <containers/sequential/graph/algorithms/graph_input_output.h>
#endif

#ifndef INVALID_VID
#define INVALID_VID (std::numeric_limits<size_t>::max())
#endif

#ifndef INVALID_EID
#define INVALID_EID (std::numeric_limits<size_t>::max())
#endif

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/CCTracker.h"

#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class Robot;


////////////////////////////////////////////////////////////////////////////////
/// Graph data structure of robot configurations (vertices) connected by local
/// plans (edges).
///
/// We often want to do some extra stuff whenever the roadmap is modified. To
/// support that, this object can install hook functions for each of the four
/// modifying events (add/delete a vertex/edge). Note that there is no
/// particular ordering to the hook execution - this is deliberate because we
/// will create a maintenance nightmare if hooks are allowed to depend on each
/// other. As such, no two hooks should modify the same data. Terrible and
/// unpredictable things will happen if you do this!
///
/// @tparam Vertex The vertex or configuration type.
/// @tparam Edge The edge or local plan type.
////////////////////////////////////////////////////////////////////////////////
template <typename Vertex, typename Edge>
class RoadmapGraph : public
#ifdef _PARALLEL
    stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Vertex, Edge>
#else
    stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Vertex,
        Edge>
#endif
{

  public:

    ///@name Local Types
    ///@{

    using STAPLGraph =
#ifdef _PARALLEL
    stapl::dynamic_graph<stapl::DIRECTED,stapl::NONMULTIEDGES,Vertex, Edge>
#else
    stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, Vertex, Edge>
#endif
    ;

    // Descriptor types.
    typedef typename STAPLGraph::vertex_descriptor        VID;
    typedef typename STAPLGraph::edge_descriptor          EID;
    typedef typename EID::edge_id_type                    EdgeID;
    typedef typename std::unordered_set<VID>              VertexSet;

    // Iterator types.
    typedef typename STAPLGraph::vertex_iterator          VI;
    typedef typename STAPLGraph::adj_edge_iterator        EI;
    typedef typename STAPLGraph::const_vertex_iterator    CVI;
    typedef typename STAPLGraph::const_adj_edge_iterator  CEI;

    // Property types.
    typedef typename STAPLGraph::vertex_property          VP;
    typedef typename STAPLGraph::edge_property            EP;
    // Useful names for the property types.
    typedef Vertex                                        CfgType;
    typedef Edge                                          EdgeType;

    typedef stapl::sequential::vector_property_map<STAPLGraph, size_t> ColorMap;

    // Hook function types.
    typedef std::function<void(VI)> VertexHook;
    typedef std::function<void(EI)> EdgeHook;
    enum class HookType {AddVertex, DeleteVertex, AddEdge, DeleteEdge};

    // CC Tracker.
    typedef CCTracker<RoadmapGraph<Vertex, Edge>> CCTrackerType;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a roadmap for a given robot.
    /// @param _r The given robot.
    RoadmapGraph(Robot* const _r);

    ///@}
    ///@name Move and Copy
    ///@{
    /// Move and copy operations do not copy hook functions.

    /// Construct a copy of the given roadmap.
    /// @param _r The given roadmap.
    RoadmapGraph(const RoadmapGraph& _r);
    /// Construct a copy of the given roadmap.
    /// @param _r The given roadmap.
    RoadmapGraph(RoadmapGraph&& _r);

    /// Copy the given roadmap into the current.
    /// @param _r The given roadmap.
    RoadmapGraph& operator=(const RoadmapGraph& _r);
    /// Copy the given roadmap into the current.
    /// @param _r The given roadmap.
    RoadmapGraph& operator=(RoadmapGraph&& _r);

    ///@}
    ///@name Equality
    ///@{

    /// Check if the current and given roadmaps are equal.
    /// @param _r The given roadmap.
    /// @return True is equal, false otherwise.
    bool operator==(const RoadmapGraph& _r) const noexcept;

    /// Check if the current and given roadmaps are unequal.
    /// @param _r The given roadmap.
    /// @return True is unequal, false otherwise.
    bool operator!=(const RoadmapGraph& _r) const noexcept;

    ///@}
    ///@name Modifiers
    ///@{

    /// Add a new unique vertex to the graph. If it already exists, a warning
    /// will be printed to cerr.
    /// @param _v The vertex to add.
    /// @return A new VID of the added vertex, or the VID of the existing vertex.
    virtual VID AddVertex(const Vertex& _v) noexcept;

    // Add a new unique vertex to the graph with a designated descriptor. If it
    // already exists or the descriptor is already in use, a warning will be
    // printed to cerr.
    // @param _vid The desired descriptor.
    // @param _v The vertex property.
    // @return A new VID of the added vertex, or the VID of the existing vertex
    //virtual VID AddVertex(const VID _vid, const Vertex& _v) noexcept;

    /// Add a vertex to the graph without checking for uniqueness.
    /// @param _v The vertex to add.
    /// @return A new VID of the added vertex, or the VID of the existing vertex.
    virtual VID AddDuplicateVertex(const Vertex& _v) noexcept;

    /// Remove a vertex (and attached edges) from the graph if it exists.
    /// @param _v The vertex descriptor.
    virtual void DeleteVertex(const VID _v) noexcept;

    /// Add an edge from source to target.
    /// @param _source The source vertex.
    /// @param _target The target vertex.
    /// @param _w  The edge property.
    virtual void AddEdge(const VID _source, const VID _target, const Edge& _w)
        noexcept;

    /// Add edges both ways between source and target vertices.
    /// @param _source The source vertex.
    /// @param _target The target vertex.
    /// @param _w  The edge properties (source to target first).
    virtual void AddEdge(const VID _source, const VID _target,
        const std::pair<Edge, Edge>& _w) noexcept;

    /// Remove an edge from the graph if it exists.
    /// @param _source The source vertex.
    /// @param _target The target vertex.
    virtual void DeleteEdge(const VID _source, const VID _target) noexcept;

    /// Remove an edge from the graph if it exists.
    /// @param _iterator An iterator to the edge.
    virtual void DeleteEdge(EI _iterator) noexcept;

    /// Set the robot pointer on all configurations in the map.
    void SetRobot(Robot* const _r) noexcept;

    /// Copy the nodes and edges from another roadmap and append them to this.
    /// Assumes the configurations are compatible with this roadmap's robot.
    /// @param _r The roadmap to copy from.
    void AppendRoadmap(const RoadmapGraph& _r);

    /// Set the CC tracker.
    /// @param _stats Optional stat class for performance profiling.
    void SetCCTracker(StatClass* const _stats = nullptr);

    ///@}
    ///@name Queries
    ///@{

    /// Get the number of vertices in the roadmap.
    size_t Size() const noexcept;

    /// Check if a vertex is present in the graph.
    /// @param _vid  The vertex descriptor
    /// @return True if the vertex descriptor was found in the graph.
    bool IsVertex(const VID _vid) const noexcept;

    /// Check if a vertex is present in the graph.
    /// @param _v  The vertex property to seek.
    /// @return True if the vertex property was found in the graph.
    bool IsVertex(const Vertex& _v) const noexcept;

    /// Check if a vertex is present in the graph and retrieve a const iterator
    /// to it if so.
    /// @param _v  The vertex property to seek.
    /// @param _vi A vertex iterator, set to the located vertex or end if not
    ///            found.
    /// @return True if the vertex property was found in the graph.
    bool IsVertex(const Vertex& _v, CVI& _vi) const noexcept;

    /// Check if an edge is present between two vertices.
    /// @param _source The source vertex.
    /// @param _target The target vertex.
    /// @return True if an edge exists from source to target.
    bool IsEdge(const VID _source, const VID _target) const noexcept;

    /// Get the descriptor of a vertex property if it exists in the graph, or
    /// INVALID_VID otherwise.
    template <typename T>
    VID GetVID(const T& _t) const noexcept;
    VID GetVID(const VI& _t) const noexcept;
    VID GetVID(const Vertex& _t) const noexcept;

    /// Get the set of predecessors for a given vertex.
    /// @param _vid The vertex descriptor.
    /// @return The set of VIDs which have _vid as a child node.
    const VertexSet& GetPredecessors(const VID _vid) const noexcept;

    /// Get the descriptor of the last vertex added to the graph.
    VID GetLastVID() const noexcept;

    /// Each time the roadmap is modified, we update the timestamp.
    size_t GetTimestamp() const noexcept;

    /// Get the set of all VIDs in the roadmap.
    const VertexSet& GetAllVIDs() const noexcept;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the robot represented by this roadmap.
    Robot* GetRobot() const noexcept;

    /// Get the connected component tracker.
    CCTrackerType* GetCCTracker() const noexcept;

    /// Retrieve a reference to a vertex property by descriptor or iterator.
    template <typename T>
    VP& GetVertex(T& _t) noexcept;
    VP& GetVertex(VI& _t) noexcept;
    VP& GetVertex(VID _t) noexcept;

    /// Retrieve a constant reference to a vertex property by descriptor or
    /// iterator.
    template <typename T>
    const VP& GetVertex(T& _t) const noexcept;
    const VP& GetVertex(CVI& _t) const noexcept;
    const VP& GetVertex(VID _t) const noexcept;

    /// Get the set of VIDs which are children of a given vertex.
    /// @param _vid The VID of the given vertex.
    /// @return The VIDs of each node u for which and edge (_vid, u) exists.
    std::vector<VID> GetChildren(const VID _vid) const noexcept;

    /// Retrieve an edge from the graph.
    /// @param _source The source node VID.
    /// @param _target The target node VID.
    /// @param _ei An edge iterator, set to the specified edge if found or end
    ///            otherwise.
    /// @return True if the edge was located.
    bool GetEdge(const VID _source, const VID _target, EI& _ei) noexcept;

    /// Retrieve an edge from the graph.
    /// @param _source The source node VID.
    /// @param _target The target node VID.
    /// @param _ei An edge iterator, set to the specified edge if found or end
    ///            otherwise.
    /// @return True if the edge was located.
    /// @overload For const edge iterator.
    bool GetEdge(const VID _source, const VID _target, CEI& _ei) const noexcept;

    EP& GetEdge(const VID _source, const VID _target) noexcept;
    EP& GetEdge(const EID _descriptor) noexcept;

    ///@}
    ///@name Hooks
    ///@{
    /// Hooks are arbitrary functions that are attached to roadmap events. I.e.,
    /// whenever a vertex is added, a set of functions should be called (hooks).
    /// There is a set of hooks for each of the four modifying actions
    /// (add/delete a vertex/edge).
    ///
    /// IMPORTANT: Hooks for 'add' events execute immediately after the event,
    ///            while hooks for 'delete' events execute immediately prior.
    ///            This ensures that the iterator and neighbor information are
    ///            valid in both cases.
    ///
    /// IMPORTANT: Dependencies between hooks create data races. To avoid
    ///            problems, any piece of data that is modified by one hook
    ///            should not be read or modified by any other hook.

    /// Check if a hook with a given type and label is installed.
    /// @param _type The hook type.
    /// @param _label The unique label.
    /// @return True if a hook of the specified type and label is present.
    bool IsHook(const HookType, const std::string& _label) const;

    /// Install a vertex hook. It will be called each time a new vertex is
    /// added.
    /// @param _type The hook type (vertex add/delete).
    /// @param _label The unique label.
    /// @param _h The hook function.
    void InstallHook(const HookType _type, const std::string& _label,
        const VertexHook& _h);

    /// Install an edge hook. It will be called each time a new edge is added.
    /// @param _type The hook type (edge add/delete).
    /// @param _label The unique label.
    /// @param _h The hook function.
    void InstallHook(const HookType _type, const std::string& _label,
        const EdgeHook& _h);

    /// Remove a hook.
    /// @param _type The hook type.
    /// @param _label The unique label.
    void RemoveHook(const HookType _type, const std::string& _label);

    /// Disable the hooks. This is useful for making temporary additions and
    /// deletions to the roadmap without triggering the hook functions. Be sure
    /// to re-enable them after, and to only use this in isolated code segments
    /// where you are sure that we won't miss any real nodes.
    void DisableHooks() noexcept;

    /// Enable the hook functions (default).
    void EnableHooks() noexcept;

    /// Uninstall all hooks. Should only be used at the end of a library run to
    /// clean the roadmap object.
    virtual void ClearHooks() noexcept;

    ///@}
    ///@name I/O
    ///@{

    /// Read in a roadmap (.map) file.
    /// @param _filename The name of the map file to read.
    /// @note Temporarily moved to non-member function.
    //virtual void Read(const std::string& _filename);

    // Temporary work-around for calling the add vertex/edge hooks.
    template <typename RG>
    friend void Read(RG* _g, const std::string& _filename);

		template <typename RG>
		friend void ReadMessage(RG* _g, const std::string& _msg); 
		//void ReadMessage(RoadmapGraph* _g, const std::string& _msg); 
    /// Write the current roadmap out to a roadmap (.map) file.
    /// @param _filename The name of the map file to write to.
    /// @param _env The environment for which this map was constructed.
    virtual void Write(const std::string& _filename, Environment* _env) const;

    ///@}

#ifdef _PARALLEL
    // Temporarily wrapper for some graph methods
    // Until full migration and change of names in STAPL is completed
    size_t get_num_edges() {return this->num_edges();}
    size_t get_num_vertices() const {return this->num_vertices();}
    size_t get_degree(const VID& _vd) {return this->find_vertex(_vd)->size();}
    size_t get_out_degree(const VID& _vd) {return this->get_degree(_vd);}
#endif

  protected:

    ///@name Base Class Unhiding
    ///@{

    /// Unhide the add and delete vertex and edge function within private
    /// encapsulation to prevent users of the RoadmapGraph from calling those
    /// functions.
    using STAPLGraph::add_vertex;
    using STAPLGraph::add_edge;
    using STAPLGraph::delete_vertex;
    using STAPLGraph::delete_edge;

    ///@}
    ///@name Helpers
    ///@{

    /// Execute the AddVertex hooks.
    /// @param _iterator An iterator to the newly added vertex.
    void ExecuteAddVertexHooks(const VI _iterator) noexcept;

    /// Execute the DeleteVertex hooks.
    /// @param _iterator An iterator to the to-be-deleted vertex.
    void ExecuteDeleteVertexHooks(const VI _iterator) noexcept;

    /// Execute the AddEdge hooks.
    /// @param _iterator An iterator to the newly added edge.
    void ExecuteAddEdgeHooks(const EI _iterator) noexcept;

    /// Execute the DeleteEdge hooks.
    /// @param _iterator An iterator to the to-be-deleted edge.
    void ExecuteDeleteEdgeHooks(const EI _iterator) noexcept;

    /// Helper for printing hook type names.
    /// @param _t A hook type.
    /// @return The string representation of _t.
    std::string ToString(const HookType& _t) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    Robot* m_robot{nullptr};  ///< The robot this roadmap is for.

    size_t m_timestamp{0};    ///< Tracks the number of changes to the graph.

    bool m_enableHooks{true}; ///< Use hook functions?

    /// Hook functions to call when adding a vertex.
    std::unordered_map<std::string, VertexHook> m_addVertexHooks;
    /// Hook functions to call when deleting a vertex.
    std::unordered_map<std::string, VertexHook> m_deleteVertexHooks;

    /// Hook functions to call when adding an edge.
    std::unordered_map<std::string, EdgeHook> m_addEdgeHooks;
    /// Hook functions to call when deleting an edge.
    std::unordered_map<std::string, EdgeHook> m_deleteEdgeHooks;

    /// Tracks weak CCs within the roadmap.
    std::unique_ptr<CCTrackerType> m_ccTracker;

    /// Tracks predecessor information. We use this instead of switching to a
    /// STAPL directed_preds graph because (a) directed_preds uses a vector for
    /// storage of VIDs, making all changes linear-time operations in the
    /// in-degree of each vertex, and (b) the STAPL API is not interchangable as
    /// it should be, so switching causes ridiculous compiler errors.
    std::unordered_map<VID, VertexSet> m_predecessors;

    /// A set of all VIDs in the roadmap. We track this to make nearest-neighbor
    /// queries more efficient.
    VertexSet m_allVIDs;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename Vertex, typename Edge>
RoadmapGraph<Vertex, Edge>::
RoadmapGraph(Robot* const _r) : m_robot(_r) { }

/*------------------------------ Move and Copy -------------------------------*/

template <typename Vertex, typename Edge>
RoadmapGraph<Vertex, Edge>::
RoadmapGraph(const RoadmapGraph& _r) : RoadmapGraph(_r.m_robot) {
  *this = _r;
}


template <typename Vertex, typename Edge>
RoadmapGraph<Vertex, Edge>::
RoadmapGraph(RoadmapGraph&& _r) : RoadmapGraph(_r.m_robot) {
  *this = std::move(_r);
}


template <typename Vertex, typename Edge>
RoadmapGraph<Vertex, Edge>&
RoadmapGraph<Vertex, Edge>::
operator=(const RoadmapGraph& _r) {
  // Clear any hooks installed on this map.
  ClearHooks();

  // Copy the stapl graph and our add-ons.
  STAPLGraph::operator=(_r);
  m_robot        = _r.m_robot;
  m_timestamp    = _r.m_timestamp;
  m_predecessors = _r.m_predecessors;

  // If the other graph had a CC tracker, copy it and point at this roadmap.
  if(_r.m_ccTracker) {
    m_ccTracker.reset(new CCTrackerType(*_r.m_ccTracker));
    m_ccTracker->SetRoadmap(this);
  }

  return *this;
}


template <typename Vertex, typename Edge>
RoadmapGraph<Vertex, Edge>&
RoadmapGraph<Vertex, Edge>::
operator=(RoadmapGraph&& _r) {
  // Clear any hooks installed on this map.
  ClearHooks();

  // Move the stapl graph and our add-ons.
  STAPLGraph::operator=(std::move(_r));
  m_robot        = _r.m_robot;
  m_timestamp    = _r.m_timestamp;
  m_predecessors = std::move(_r.m_predecessors);

  // If the other graph had a CC tracker, move it and point at this roadmap.
  if(_r.m_ccTracker) {
    m_ccTracker = std::move(_r.m_ccTracker);
    m_ccTracker->SetRoadmap(this);
  }

  // Clear any hooks on the other to prevent cross-talk with the CCTracker.
  _r.ClearHooks();

  return *this;
}

/*------------------------------- Equality -----------------------------------*/

template <typename Vertex, typename Edge>
bool
RoadmapGraph<Vertex, Edge>::
operator==(const RoadmapGraph& _r) const noexcept {
  // First do fast checks on addess and sizes.
  if(this == &_r)
    return true;
  if(this->Size() != _r.Size() or this->get_num_edges() != _r.get_num_edges())
    return false;

  // Check vertices and edges.
  for(auto va = this->begin(); va != this->end(); ++va) {
    // Find the matching descriptor in _r.
    auto vb = _r.find_vertex(va->descriptor());

    // Check that the property and number of edges are the same.
    if(va->property() != vb->property() or va->size() != vb->size())
      return false;

    // Check that the edges are the same. Do not assume the edge ordering is
    // the same, only worry about source/target correspondance.
    for(auto ea = va->begin(); ea != va->end(); ++ea) {
      CEI eb;
      if(!_r.GetEdge(ea->source(), ea->target(), eb)
          or ea->property() != eb->property())
        return false;
    }
  }

  return  true;
}


template <typename Vertex, typename Edge>
bool
RoadmapGraph<Vertex, Edge>::
operator!=(const RoadmapGraph& _r) const noexcept {
  return !(*this == _r);
}

/*------------------------------- Modifiers ----------------------------------*/

template <typename Vertex, typename Edge>
typename RoadmapGraph<Vertex, Edge>::VID
RoadmapGraph<Vertex, Edge>::
AddVertex(const Vertex& _v) noexcept {
  // Find the vertex and ensure it does not already exist.
  CVI vi;
  if(IsVertex(_v, vi)) {
    std::cerr << "\nRoadmapGraph::AddVertex: vertex " << vi->descriptor()
              << " already in graph, not adding again."
              << std::endl;
    return vi->descriptor();
  }

  // The vertex does not exist. Add it now.
  const VID vid = this->add_vertex(_v);
  m_predecessors[vid];
  m_allVIDs.insert(vid);
  ++m_timestamp;

  // Execute post-add hooks and update vizmo debug.
  ExecuteAddVertexHooks(this->find_vertex(vid));

  return vid;
}

template <typename Vertex, typename Edge>
typename RoadmapGraph<Vertex, Edge>::VID
RoadmapGraph<Vertex, Edge>::
AddDuplicateVertex(const Vertex& _v) noexcept {
  
	const VID vid = this->add_vertex(_v);
  ++m_timestamp;

  // Execute post-add hooks and update vizmo debug.
  ExecuteAddVertexHooks(this->find_vertex(vid));
  VDAddNode(_v);

  return vid;
}

template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
DeleteVertex(const VID _v) noexcept {
  // Find the vertex and crash if it doesn't exist.
  VI vi = this->find_vertex(_v);
  if(vi == this->end())
    throw RunTimeException(WHERE) << "VID " << _v << " is not in the graph.";

  // We must manually delete the edges rather than letting STAPL do this in
  // order to call the DeleteEdge hooks.

  // Delete the outbound edges.
  for(auto edge = vi->begin(); edge != vi->end(); edge = vi->begin())
    DeleteEdge(edge);

  // Delete the inbound edges.
  auto predIter = m_predecessors.find(_v);
  VertexSet& preds = predIter->second;
  while(!preds.empty()) {
    const VID predecessor = *preds.begin();
    DeleteEdge(predecessor, _v);
  }

  // Execute pre-delete hooks.
  ExecuteDeleteVertexHooks(vi);

  // Delete the vertex.
  this->delete_vertex(vi->descriptor());
  m_predecessors.erase(predIter);
  m_allVIDs.erase(_v);
  ++m_timestamp;
}


template <class Vertex, class Edge>
void
RoadmapGraph<Vertex, Edge>::
AddEdge(const VID _source, const VID _target, const Edge& _w) noexcept {
  // Let the add_edge function handle checking for existance incase we are using
  // a multigraph.
  const auto edgeDescriptor = this->add_edge(_source, _target, _w);
  const bool notNew = edgeDescriptor.id() == INVALID_EID;

  if(notNew) {
    std::cerr << "\nRoadmapGraph::AddEdge: edge (" << _source << ", "
              << _target << ") already exists, not adding."
              << std::endl;
    return;
  }

  // Add the source as a predecessor of target.
  m_predecessors[_target].insert(_source);
  ++m_timestamp;

  // Execute post-add hooks.
  VI vi;
  EI ei;
  this->find_edge(edgeDescriptor, vi, ei);
  ExecuteAddEdgeHooks(ei);
}


template <class Vertex, class Edge>
void
RoadmapGraph<Vertex, Edge>::
AddEdge(const VID _source, const VID _target, const std::pair<Edge, Edge>& _w)
    noexcept {
  AddEdge(_source, _target, _w.first);
  AddEdge(_target, _source, _w.second);
}


template <class Vertex, class Edge>
void
RoadmapGraph<Vertex, Edge>::
DeleteEdge(const VID _source, const VID _target) noexcept {
  // Find the edge and crash if it isn't found.
  const EID edgeDescriptor(_source, _target);
  VI dummy;
  EI edgeIterator;

  const bool found = this->find_edge(edgeDescriptor, dummy, edgeIterator);
  if(!found)
    throw RunTimeException(WHERE) << "Edge (" << _source << ", " << _target
                                  << ") does not exist.";

  DeleteEdge(edgeIterator);
}


template <class Vertex, class Edge>
void
RoadmapGraph<Vertex, Edge>::
DeleteEdge(EI _iterator) noexcept {
  // Execute pre-delete hooks.
  const VID source = _iterator->source(),
            target = _iterator->target();
  ExecuteDeleteEdgeHooks(_iterator);

  // Delete the edge.
  this->delete_edge(_iterator->descriptor());
  m_predecessors[target].erase(source);
  ++m_timestamp;
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
SetRobot(Robot* const _r) noexcept {
  m_robot = _r;
  for(VI vi = this->begin(); vi != this->end(); ++vi)
    vi->property().SetRobot(_r);
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
AppendRoadmap(const RoadmapGraph& _r) {
  // Copy vertices and map the change in VIDs.
  std::unordered_map<VID, VID> oldToNew;
  for(auto vit = _r.begin(); vit != _r.end(); ++vit) {
    const VID oldVID = vit->descriptor();
    const VID newVID = AddVertex(vit->property());
    oldToNew[oldVID] = newVID;
  }

  // Copy edges.
  for(auto vit = _r.begin(); vit != _r.end(); ++vit) {
    for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
      const VID source = oldToNew[eit->source()];
      const VID target = oldToNew[eit->target()];
      if(!IsEdge(source, target))
        AddEdge(source, target, eit->property());
    }
  }
}


template <typename Vertex, typename Edge>
inline
void
RoadmapGraph<Vertex, Edge>::
SetCCTracker(StatClass* const _stats) {
  m_ccTracker.reset(new CCTrackerType(this));
  if(_stats)
    m_ccTracker->SetStatClass(_stats);
}

/*-------------------------------- Queries -----------------------------------*/

template <typename Vertex, typename Edge>
inline
size_t
RoadmapGraph<Vertex, Edge>::
Size() const noexcept {
  return this->get_num_vertices();
}


template <typename Vertex, typename Edge>
inline
bool
RoadmapGraph<Vertex, Edge>::
IsVertex(const VID _vid) const noexcept {
  return this->find_vertex(_vid) != this->end();
}

template <typename Vertex, typename Edge>
inline
bool
RoadmapGraph<Vertex, Edge>::
IsVertex(const Vertex& _v) const noexcept {
  CVI vi;
  return IsVertex(_v, vi);
}


template <typename Vertex, typename Edge>
inline
bool
RoadmapGraph<Vertex, Edge>::
IsVertex(const Vertex& _v, CVI& _vi) const noexcept {
#ifndef _PARALLEL
  for(CVI vi = this->begin(); vi != this->end(); ++vi){
    if(vi->property() == _v){
      _vi = vi;
      return true;
    }
  }
#else
  std::cerr << "WARNING::STAPL working on fixing problem with const iterators\n";
#endif
  return false;
}


template <typename Vertex, typename Edge>
inline
bool
RoadmapGraph<Vertex, Edge>::
IsEdge(const VID _source, const VID _target) const noexcept {
  CEI ei;
  CVI vi;
  return this->find_edge(EID(_source, _target), vi, ei);
}


template <typename Vertex, typename Edge>
template <typename T>
inline
typename RoadmapGraph<Vertex, Edge>::VID
RoadmapGraph<Vertex, Edge>::
GetVID(const T& _t) const noexcept {
  return *_t;
}


template <typename Vertex, typename Edge>
inline
typename RoadmapGraph<Vertex, Edge>::VID
RoadmapGraph<Vertex, Edge>::
GetVID(const VI& _t) const noexcept {
  return _t->descriptor();
}


template <typename Vertex, typename Edge>
inline
typename RoadmapGraph<Vertex, Edge>::VID
RoadmapGraph<Vertex, Edge>::
GetVID(const Vertex& _t) const noexcept {
  CVI vi;
  if(IsVertex(_t, vi))
    return vi->descriptor();
  return INVALID_VID;
}


template <typename Vertex, typename Edge>
inline
const typename RoadmapGraph<Vertex, Edge>::VertexSet&
RoadmapGraph<Vertex, Edge>::
GetPredecessors(const VID _vid) const noexcept {
  return m_predecessors.at(_vid);
}


template <typename Vertex, typename Edge>
inline
typename RoadmapGraph<Vertex, Edge>::VID
RoadmapGraph<Vertex, Edge>::
GetLastVID() const noexcept {
  if(this->get_num_vertices() == 0)
    return INVALID_VID;

  return (--this->end())->descriptor();
}


template <typename Vertex, typename Edge>
inline
size_t
RoadmapGraph<Vertex, Edge>::
GetTimestamp() const noexcept {
  return m_timestamp;
}


template <typename Vertex, typename Edge>
inline
const typename RoadmapGraph<Vertex, Edge>::VertexSet&
RoadmapGraph<Vertex, Edge>::
GetAllVIDs() const noexcept {
  return m_allVIDs;
}

/*------------------------------- Accessors ----------------------------------*/

template <typename Vertex, typename Edge>
inline
Robot*
RoadmapGraph<Vertex, Edge>::
GetRobot() const noexcept {
  return m_robot;
}


template <typename Vertex, typename Edge>
inline
typename RoadmapGraph<Vertex, Edge>::CCTrackerType*
RoadmapGraph<Vertex, Edge>::
GetCCTracker() const noexcept {
  return m_ccTracker.get();
}


template <typename Vertex, typename Edge>
template <typename T>
inline
typename RoadmapGraph<Vertex, Edge>::VP&
RoadmapGraph<Vertex, Edge>::
GetVertex(T& _t) noexcept {
  return GetVertex(VID(*_t));
}


template <typename Vertex, typename Edge>
inline
typename RoadmapGraph<Vertex, Edge>::VP&
RoadmapGraph<Vertex, Edge>::
GetVertex(VI& _t) noexcept {
  return _t->property();
}


template <typename Vertex, typename Edge>
inline
typename RoadmapGraph<Vertex, Edge>::VP&
RoadmapGraph<Vertex, Edge>::
GetVertex(VID _t) noexcept {
  VI iter = this->find_vertex(_t);
  if(iter == this->end())
    throw RunTimeException(WHERE) << "Requested node '" << _t
                                  << "', which is not in the graph.";

  return iter->property();
}


template <typename Vertex, typename Edge>
template <typename T>
inline
const typename RoadmapGraph<Vertex, Edge>::VP&
RoadmapGraph<Vertex, Edge>::
GetVertex(T& _t) const noexcept {
  return GetVertex(VID(*_t));
}


template <typename Vertex, typename Edge>
inline
const typename RoadmapGraph<Vertex, Edge>::VP&
RoadmapGraph<Vertex, Edge>::
GetVertex(CVI& _t) const noexcept {
  return (*_t).property();
}


template <typename Vertex, typename Edge>
inline
const typename RoadmapGraph<Vertex, Edge>::VP&
RoadmapGraph<Vertex, Edge>::
GetVertex(VID _t) const noexcept {
  CVI iter = this->find_vertex(_t);
  if(iter == this->end())
    throw RunTimeException(WHERE) << "Requested node '" << _t
                                  << "', which is not in the graph.";

  return (*iter).property();
}


template <typename Vertex, typename Edge>
std::vector<typename RoadmapGraph<Vertex, Edge>::VID>
RoadmapGraph<Vertex, Edge>::
GetChildren(const VID _vid) const noexcept {
  auto vi = this->find_vertex(_vid);
  if(vi == this->end())
    throw RunTimeException(WHERE) << "Requested children of node '" << _vid
                                  << "', which is not in the graph.";

  std::vector<VID> output;
  for(const auto& e : *vi)
    output.push_back(e.target());
  return output;
}


template <typename Vertex, typename Edge>
inline
bool
RoadmapGraph<Vertex, Edge>::
GetEdge(const VID _source, const VID _target, EI& _ei) noexcept {
  VI vi;
  return this->find_edge(EID(_source, _target), vi, _ei);
}


template <typename Vertex, typename Edge>
inline
bool
RoadmapGraph<Vertex, Edge>::
GetEdge(const VID _source, const VID _target, CEI& _ei) const noexcept {
  CVI vi;
  return this->find_edge(EID(_source, _target), vi, _ei);
}


template <typename Vertex, typename Edge>
inline
typename RoadmapGraph<Vertex, Edge>::EP&
RoadmapGraph<Vertex, Edge>::
GetEdge(const VID _source, const VID _target) noexcept {
  EI ei;
  if(!GetEdge(_source, _target, ei))
    throw RunTimeException(WHERE) << "Requested non-existent edge ("
                                  << _source << ", " << _target << ").";

  return ei->property();
}


template <typename Vertex, typename Edge>
inline
typename RoadmapGraph<Vertex, Edge>::EP&
RoadmapGraph<Vertex, Edge>::
GetEdge(const EID _descriptor) noexcept {
  return GetEdge(_descriptor.source(), _descriptor.target());
}

/*--------------------------------- Hooks ------------------------------------*/

template <typename Vertex, typename Edge>
bool
RoadmapGraph<Vertex, Edge>::
IsHook(const HookType _type, const std::string& _label) const {
  switch(_type) {
    case HookType::AddVertex:
      return m_addVertexHooks.count(_label);
    case HookType::DeleteVertex:
      return m_deleteVertexHooks.count(_label);
    case HookType::AddEdge:
      return m_addEdgeHooks.count(_label);
    case HookType::DeleteEdge:
      return m_deleteEdgeHooks.count(_label);
    default:
      throw RunTimeException(WHERE) << "Unrecognized hook type '"
                                    << ToString(_type) << "'.";
  }
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
InstallHook(const HookType _type, const std::string& _label,
    const VertexHook& _h) {
  // Ensure that the hook does not already exist.
  if(IsHook(_type, _label))
    throw RunTimeException(WHERE) << "Hook of type '" << ToString(_type)
                                  << "' with label '" << _label
                                  << "' already exists.";

  switch(_type) {
    case HookType::AddVertex:
      m_addVertexHooks[_label] = _h;
      break;
    case HookType::DeleteVertex:
      m_deleteVertexHooks[_label] = _h;
      break;
    case HookType::AddEdge:
    case HookType::DeleteEdge:
      throw RunTimeException(WHERE) << "Edge hook type '" << ToString(_type)
                                    << "' used with vertex hook function "
                                    << "labeled '" << _label << "'.";
    default:
      throw RunTimeException(WHERE) << "Unrecognized hook type '"
                                    << ToString(_type)
                                    << "' with label '" << _label << "'.";
  }
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
InstallHook(const HookType _type, const std::string& _label, const EdgeHook& _h) {
  // Ensure that the hook does not already exist.
  if(IsHook(_type, _label))
    throw RunTimeException(WHERE) << "Hook of type '" << ToString(_type)
                                  << "' with label '" << _label
                                  << "' already exists.";

  switch(_type) {
    case HookType::AddEdge:
      m_addEdgeHooks[_label] = _h;
      break;
    case HookType::DeleteEdge:
      m_deleteEdgeHooks[_label] = _h;
      break;
    case HookType::AddVertex:
    case HookType::DeleteVertex:
      throw RunTimeException(WHERE) << "Vertex hook type '" << ToString(_type)
                                    << "' used with edge hook function labeled '"
                                    << _label << "'.";
    default:
      throw RunTimeException(WHERE) << "Unrecognized hook type '"
                                    << ToString(_type)
                                    << "' with label '" << _label << "'.";
  }
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
RemoveHook(const HookType _type, const std::string& _label) {
  if(!IsHook(_type, _label))
    throw RunTimeException(WHERE) << "Hook of type '" << ToString(_type)
                                  << "' with label '" << _label
                                  << "' does not exist.";

  switch(_type) {
    case HookType::AddVertex:
      m_addVertexHooks.erase(_label);
      break;
    case HookType::DeleteVertex:
      m_deleteVertexHooks.erase(_label);
      break;
    case HookType::AddEdge:
      m_addEdgeHooks.erase(_label);
      break;
    case HookType::DeleteEdge:
      m_deleteEdgeHooks.erase(_label);
      break;
    default:
      throw RunTimeException(WHERE) << "Unrecognized hook type '"
                                    << ToString(_type)
                                    << "' with label '" << _label << "'.";
  }
}


template <typename Vertex, typename Edge>
inline
void
RoadmapGraph<Vertex, Edge>::
DisableHooks() noexcept {
  m_enableHooks = false;
}


template <typename Vertex, typename Edge>
inline
void
RoadmapGraph<Vertex, Edge>::
EnableHooks() noexcept {
  m_enableHooks = true;
}


template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
ClearHooks() noexcept {
  m_addVertexHooks.clear();
  m_deleteVertexHooks.clear();
  m_addEdgeHooks.clear();
  m_deleteEdgeHooks.clear();
}

/*----------------------------------- I/O ------------------------------------*/


#if 1
/// Read in a roadmap (.map) file.
/// @param _g The graph to populate from the map file.
/// @param _filename The name of the map file to read.
/// @note This is a non-member to avoid problems with explicit instantiation of
///       this function for robot groups. It should be returned to a member
///       function when we have an elegant means for implementing group roadmap
///       input.
template <typename RoadmapGraph>
void
Read(RoadmapGraph* _g, const std::string& _filename) {
  std::ifstream ifs(_filename);
  if(!ifs)
    throw ParseException(WHERE) << "Cannot open file '" << _filename << "'.";

  std::string tag;
  bool headerParsed = false;
  int graphStart = 0;
  // Read the file until we find the GRAPHSTART tag.
  while(!headerParsed) {
    // Mark our position and read the next line.
    graphStart = ifs.tellg();
    if(!(ifs >> tag))
      throw ParseException(WHERE) << "Error reading map file '" << _filename
                                  << "' - GRAPHSTART tag is missing.";

    // If we find the GRAPHSTART tag, we are done.
    if(tag.find("GRAPHSTART") != std::string::npos)
      headerParsed = true;
  }

  if(!_g->GetRobot())
    RunTimeException(WHERE) << "Must specify robot when reading in roadmaps.";

  typedef typename RoadmapGraph::STAPLGraph STAPLGraph;
  typedef typename RoadmapGraph::vertex_property Vertex;
  typedef typename RoadmapGraph::edge_property Edge;

  // Set the input robot for our edge class.
  /// @TODO this is a bad way to handle the fact that it's necessary to know
  /// the robot type (non/holonomic) when reading and writing.
  Edge::inputRobot = _g->GetRobot();
  Vertex::inputRobot = _g->GetRobot();

  // Set ifs back to the line with the GRAPHSTART tag and read in the graph.
  ifs.seekg(graphStart, ifs.beg);
  stapl::sequential::read_graph<STAPLGraph>(*_g, ifs);

  // Unset the input robot for our edge class.
  Edge::inputRobot = nullptr;
  Vertex::inputRobot = nullptr;

  for(auto vi = _g->begin(); vi != _g->end(); ++vi)
    _g->ExecuteAddVertexHooks(vi);
  for(auto vi = _g->begin(); vi != _g->end(); ++vi)
    for(auto ei = vi->begin(); ei != vi->end(); ++ei)
      _g->ExecuteAddEdgeHooks(ei);
}
#else
template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
Read(const std::string& _filename) {
  std::ifstream ifs(_filename);
  if(!ifs)
    throw ParseException(WHERE) << "Cannot open file '" << _filename << "'.";

  std::string tag;
  bool headerParsed = false;
  int graphStart = 0;
  // Read the file until we find the GRAPHSTART tag.
  while(!headerParsed) {
    // Mark our position and read the next line.
    graphStart = ifs.tellg();
    if(!(ifs >> tag))
      throw ParseException(WHERE) << "Error reading map file '" << _filename
                                  << "' - GRAPHSTART tag is missing.";

    // If we find the GRAPHSTART tag, we are done.
    if(tag.find("GRAPHSTART") != std::string::npos)
      headerParsed = true;
  }

  if(!GetRobot())
    RunTimeException(WHERE) << "Must specify robot when reading in roadmaps.";

  // Set the input robot for our edge class.
  /// @TODO this is a bad way to handle the fact that it's necessary to know
  /// the robot type (non/holonomic) when reading and writing.
  Edge::inputRobot = GetRobot();
  Vertex::inputRobot = GetRobot();

  // Set ifs back to the line with the GRAPHSTART tag and read in the graph.
  ifs.seekg(graphStart, ifs.beg);
  stapl::sequential::read_graph<STAPLGraph>(*this, ifs);

  // Unset the input robot for our edge class.
  Edge::inputRobot = nullptr;
  Vertex::inputRobot = nullptr;
}
#endif

/// Read in a message.
/// @param _g The graph to populate from the message.
/// @param _msg The message containing the graph.
/// @note This is a non-member to avoid problems with explicit instantiation of
///       this function for robot groups. It should be returned to a member
///       function when we have an elegant means for implementing group roadmap
///       input.
template <typename RoadmapGraph>
void
ReadMessage(RoadmapGraph* _g, const std::string& _msg) {
  std::stringstream ss(_msg);

  std::string tag;
  bool headerParsed = false;
  int graphStart = 0;
  // Read the file until we find the GRAPHSTART tag.
  while(!headerParsed) {
    // Mark our position and read the next line.
    graphStart = ss.tellg();
    if(!(ss >> tag))
      throw ParseException(WHERE) << "Error reading msg '" << _msg
                                  << "' - GRAPHSTART tag is missing.";

    // If we find the GRAPHSTART tag, we are done.
    if(tag.find("GRAPHSTART") != std::string::npos)
      headerParsed = true;
  }

  if(!_g->GetRobot())
    RunTimeException(WHERE) << "Must specify robot when reading in roadmaps.";

  typedef typename RoadmapGraph::STAPLGraph STAPLGraph;
  typedef typename RoadmapGraph::vertex_property Vertex;
  typedef typename RoadmapGraph::edge_property Edge;

  // Set the input robot for our edge class.
  /// @TODO this is a bad way to handle the fact that it's necessary to know
  /// the robot type (non/holonomic) when reading and writing.
  Edge::inputRobot = _g->GetRobot();
  Vertex::inputRobot = _g->GetRobot();

  // Set ss back to the line with the GRAPHSTART tag and read in the graph.
  ss.seekg(graphStart, ss.beg);
  stapl::sequential::read_graph<STAPLGraph>(*_g, ss);

  // Unset the input robot for our edge class.
  Edge::inputRobot = nullptr;
  Vertex::inputRobot = nullptr;

  for(auto vi = _g->begin(); vi != _g->end(); ++vi)
    _g->ExecuteAddVertexHooks(vi);
  for(auto vi = _g->begin(); vi != _g->end(); ++vi)
    for(auto ei = vi->begin(); ei != vi->end(); ++ei)
      _g->ExecuteAddEdgeHooks(ei);
}

template <typename Vertex, typename Edge>
void
RoadmapGraph<Vertex, Edge>::
Write(const std::string& _filename, Environment* _env) const {
  std::ofstream ofs(_filename);
  ofs << "#####ENVFILESTART#####" << std::endl
      << _env->GetEnvFileName() << std::endl
      << "#####ENVFILESTOP#####" << std::endl;

#ifndef _PARALLEL
  stapl::sequential::write_graph(*this, ofs);
#else
  ofs.close();
  stapl::graph_view<STAPLGraph> gv(*this);
  write_PMPL_graph(gv, _filename);
#endif

}

/*-------------------------------- Helpers -----------------------------------*/

template <typename Vertex, typename Edge>
inline
void
RoadmapGraph<Vertex, Edge>::
ExecuteAddVertexHooks(const VI _iterator) noexcept {
  if(!m_enableHooks)
    return;

  if(m_ccTracker)
    m_ccTracker->AddVertex(_iterator);

  for(auto& hook : m_addVertexHooks)
    hook.second(_iterator);
}


template <typename Vertex, typename Edge>
inline
void
RoadmapGraph<Vertex, Edge>::
ExecuteDeleteVertexHooks(const VI _iterator) noexcept {
  if(!m_enableHooks)
    return;

  for(auto& hook : m_deleteVertexHooks)
    hook.second(_iterator);

  if(m_ccTracker)
    m_ccTracker->DeleteVertex(_iterator);
}


template <typename Vertex, typename Edge>
inline
void
RoadmapGraph<Vertex, Edge>::
ExecuteAddEdgeHooks(const EI _iterator) noexcept {
  if(!m_enableHooks)
    return;

  if(m_ccTracker)
    m_ccTracker->AddEdge(_iterator);

  for(auto& hook : m_addEdgeHooks)
    hook.second(_iterator);
}


template <typename Vertex, typename Edge>
inline
void
RoadmapGraph<Vertex, Edge>::
ExecuteDeleteEdgeHooks(const EI _iterator) noexcept {
  if(!m_enableHooks)
    return;

  for(auto& hook : m_deleteEdgeHooks)
    hook.second(_iterator);

  if(m_ccTracker)
    m_ccTracker->DeleteEdge(_iterator);
}


template <typename Vertex, typename Edge>
std::string
RoadmapGraph<Vertex, Edge>::
ToString(const HookType& _t) const noexcept {
  switch(_t) {
    case HookType::AddVertex:
      return "AddVertex";
    case HookType::DeleteVertex:
      return "DeleteVertex";
    case HookType::AddEdge:
      return "AddEdge";
    case HookType::DeleteEdge:
      return "DeleteEdge";
    default:
      return std::string(1, char(_t));
  }
}

/*----------------------------------------------------------------------------*/

/// Find the intersection of two vertex sets.
/// @param _a The first set.
/// @param _b The second set.
/// @return The intersection of _a and _b.
inline
std::unordered_set<size_t>
VertexSetIntersection(const std::unordered_set<size_t>& _a,
    const std::unordered_set<size_t>& _b) {
  using VertexSet = std::unordered_set<size_t>;

  // Find the smaller of the two sets.
  const VertexSet* smaller{nullptr},
                 * larger{nullptr};
  if(_a.size() < _b.size()) {
    smaller = &_a;
    larger  = &_b;
  }
  else {
    smaller = &_b;
    larger  = &_a;
  }

  // Test each element in the smaller set for membership in the larger one.
  VertexSet intersection;
  std::copy_if(smaller->begin(), smaller->end(),
               std::inserter(intersection, intersection.end()),
               [larger](const size_t _vid) {return larger->count(_vid);});
  return intersection;
}


/// Check if two vertex sets have any members in common.
/// @param _a The first set.
/// @param _b The second set.
/// @return True if there are any vertices in both _a and _b.
inline
bool
HaveCommonVertex(const std::unordered_set<size_t>& _a,
                 const std::unordered_set<size_t>& _b) {
  using VertexSet = std::unordered_set<size_t>;

  // Find the smaller of the two sets.
  const VertexSet* smaller,
                 * larger;
  if(_a.size() < _b.size()) {
    smaller = &_a;
    larger  = &_b;
  }
  else {
    smaller = &_b;
    larger  = &_a;
  }

  // Test each element in the smaller set for membership in the larger one.
  return std::any_of(smaller->begin(), smaller->end(),
                     [larger](const size_t _vid) {return larger->count(_vid);});
}


/// Select a random element from a vertex set (or other unordered set).
/// @param _set The set.
/// @return A random element from within.
/// @note This is a two-stage random process that is far more efficient than
///       simply std::advancing the begin iterator, which costs linear time. The
///       solution here is to randomly select a bucket and then choose a random
///       element within. This is linear in the bucket size (much smaller than
///       the whole container).
template <typename T, typename... Rest>
inline
const T&
RandomElement(const std::unordered_set<T, Rest...>& _set) noexcept {
  if(_set.empty())
    throw RunTimeException(WHERE) << "Can't select a random element from an "
                                  << "empty set.";
#if 1
  // Pick a random bucket within the set.
  size_t bucketIndex, bucketSize, tries = 0;
  do {
    ++tries;
    bucketIndex = DRand() * _set.bucket_count();
    bucketSize  = _set.bucket_size(bucketIndex);
  }
  while(bucketSize == 0);

  const size_t elementIndex = DRand() * bucketSize;
#if 0
  std::cout << "\nset size = " << _set.size()
            << "\ntries = " << tries
            << "\nbucketIndex = "  << bucketIndex
            << "\nbucketSize  = "  << bucketSize
            << "\nelementIndex = " << elementIndex
            << "\nelement = "
            << *std::next(_set.begin(bucketIndex), elementIndex)
            << std::endl;
#endif
  return *std::next(_set.begin(bucketIndex), elementIndex);
#else
  // This is the fully linear solution, which works better for very small sets.
  // It is retained here for reference.
  return *std::next(_set.begin(), DRand() * _set.size());
#endif
}

/*----------------------------------------------------------------------------*/

#endif
