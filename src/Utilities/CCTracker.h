#ifndef PMPL_CC_TRACKER_H_
#define PMPL_CC_TRACKER_H_

#include "Utilities/MetricUtils.h"

#include "nonstd/call_on_destruct.h"
#include "nonstd/io.h"

#include <cstddef>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>


////////////////////////////////////////////////////////////////////////////////
/// Track the set of CC's in a roadmap.
///
/// For directed graphs, we will compute weak connectivity because it describes
/// the typical usage for RRT and nonholonomic robots. One should recall, however,
/// that in such cases shared CC membership does not imply connectivity.
///
/// For this to work correctly with directed trees, one must ensure that the
/// graph always remains a tree. For instance, if you rewire the tree by switching a
/// vertex's parent, you must delete the old edge before adding the new one.
///
/// @warning This class assumes that either all edges are bidirectional, or the
///          roadmap is a single tree. It will not work correctly (silent
///          errors) on a non-tree directed graph!
/// @todo Add support for non-tree directed graphs. We need to use predecessor
///       information to do this, which we now have in the roadmap graph.
/// @todo The stat class tracking adds a lot of complication. Remove it once
///       we're sure the performance is fast enough to forgo tracking.
////////////////////////////////////////////////////////////////////////////////
template <typename RoadmapType>
class CCTracker final {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename RoadmapType::VID               VID;
    typedef typename RoadmapType::vertex_iterator   vertex_iterator;
    typedef typename RoadmapType::adj_edge_iterator edge_iterator;
    typedef typename RoadmapType::VertexSet         VertexSet;

    ///@}
    ///@name Local Types
    ///@{

    typedef CCTracker<RoadmapType>         CCTrackerType;
    typedef std::unordered_set<VertexSet*> ComponentSet;

    /// An event hook for component creation. Called after creation.
    /// @param Pointer to this.
    /// @param The newly created component.
    typedef std::function<void(const CCTrackerType* const,
        const VertexSet* const)>
        CreatedHook;

    /// An event hook for component deletion. Called prior to deletion.
    /// @param Pointer to this.
    /// @param The to-be deleted component.
    typedef std::function<void(const CCTrackerType* const,
        const VertexSet* const)>
        DeletedHook;

    /// An event hook for component breaks. Called after breaking.
    /// @param Pointer to this.
    /// @param The component which is now broken.
    /// @param The new component split off from the old (does not trigger a
    ///        create event).
    typedef std::function<void(const CCTrackerType* const,
        const VertexSet* const, const VertexSet* const)>
        BrokenHook;

    /// An event hook for component merges. Called prior to merging.
    /// @param Pointer to this.
    /// @param The component which will be merged into.
    /// @param The component which will be removed after merging (does not
    ///        trigger a delete event).
    typedef std::function<void(const CCTrackerType* const,
        const VertexSet* const, const VertexSet* const)>
        MergedHook;

    ///@}
    ///@name Construction
    ///@{

    /// Constructor.
    /// @param _r The roadmap for which we will track CCs.
    CCTracker(RoadmapType* const _r);

    ~CCTracker();

    ///@}
    ///@name Move and Copy
    ///@{
    /// @note If this object is moved/copied as part of a move/copy on the
    ///       roadmap, we must also update the roadmap pointer and hooks.
    /// @note Move will retain stat class tracking but copying does not. This is
    ///       because moving generally pertains to an internal re-organization
    ///       of the same roadmap, while copying usually indicates moving a
    ///       roadmap to another solution.

    CCTracker(const CCTracker&);
    CCTracker(CCTracker&&);

    CCTracker& operator=(const CCTracker&);
    CCTracker& operator=(CCTracker&&);

    ///@}
    ///@name Modifiers
    ///@{

    /// Set the roadmap object.
    void SetRoadmap(RoadmapType* const _r) noexcept;

    /// Set the stat class pointer if we wish to do performance profiling.
    void SetStatClass(StatClass* const _stats) const noexcept;

    ///@}
    ///@name Query Functions
    ///@{
    /// These functions answer questions about the CCs in the graph.

    /// Get the number of connected components in the graph.
    size_t GetNumCCs() const noexcept;

    /// Get the CC which contains a representative node.
    /// @param _vid The representative's descriptor.
    /// @return The CC which contains _vid.
    const VertexSet* GetCC(const VID _vid) const noexcept;

    /// Get a set of representative VIDs for each CC.
    /// @return The descriptor for one vertex from each CC.
    VertexSet GetRepresentatives() const noexcept;

    /// Check if two vertices are in the same CC.
    /// @param _vid1 The descriptor of the first vertex.
    /// @param _vid2 The descriptor of the second vertex.
    /// @return True if _vid1 and _vid2 are in the same CC.
    bool InSameCC(const VID _vid1, const VID _vid2) const noexcept;

    ///@}
    ///@name Iteration
    ///@{
    /// Iterate over the CCs.

    typename ComponentSet::const_iterator begin() const noexcept;
    typename ComponentSet::const_iterator end() const noexcept;

    ///@}
    ///@name Event Hooks
    ///@{
    /// These functions install and manage hook functions which are called in
    /// response to changes in the components. Each has a unique label;
    /// duplicates will trigger exceptions.

    void InstallCreatedHook(const std::string& _label, const CreatedHook& _h);
    void InstallDeletedHook(const std::string& _label, const DeletedHook& _h);
    void InstallBrokenHook(const std::string& _label, const BrokenHook& _h);
    void InstallMergedHook(const std::string& _label, const MergedHook& _h);

    ///@}
    ///@name Optimization
    ///@{
    /// Updates can be disabled to optimize rewiring operations (where we change
    /// the graph without changing connectivity).

    void Disable();
    void Enable();

    ///@}
    ///@name Debug
    ///@{

    /// Print the full set of CC information.
    /// @param _os The ostream to print to.
    /// @param _indent A set of indent characters to print before each line.
    void Print(std::ostream& _os, const std::string& _indent = {}) const;

    ///@}
    ///@name Roadmap Callback Functions
    ///@{
    /// These functions accept graph updates and adjust the internal CCs
    /// accordingly. They should be installed as hooks in the roadmap.

    /// Update for a vertex added to the graph.
    /// @param _vi The vertex iterator.
    void AddVertex(const vertex_iterator _vi) noexcept;

    /// Update for a vertex deleted from the graph.
    /// @param _vi The vertex iterator.
    void DeleteVertex(const vertex_iterator _vi) noexcept;

    /// Update for an edge added to the graph.
    /// @param _ei The edge iterator.
    void AddEdge(const edge_iterator _ei) noexcept;

    /// Update for an edge deleted from the graph.
    /// @param _ei The edge iterator.
    void DeleteEdge(const edge_iterator _ei) noexcept;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Find the CC containing a given vertex.
    /// @param _vid The vertex descriptor.
    /// @return A CC which holds _vid.
    const VertexSet* FindCC(const VID _vid) const noexcept;

    /// Recompute all CCs from scratch. Complexity is linear in the size of the
    /// roadmap (O(V+E)).
    void RecomputeCCs() noexcept;

    /// @overload
    VertexSet* FindCC(const VID _vid) noexcept;

    /// Run a breadth-first search on the roadmap from a particular start node.
    /// @param _start The start node.
    /// @param _goals If provided, stop early once we've seen all of these. This
    ///               is used to check that _start can reach all vertices in
    ///               _goals.
    /// @return The set of discovered vertices.
    VertexSet BFS(const VID _start, VertexSet _goals = {}) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    RoadmapType* m_roadmap{nullptr};                ///< The roadmap.
    ComponentSet m_components;                      ///< The set of components.
    std::unordered_map<VID, VertexSet*> m_vidToCC;  ///< Maps descriptor to CC.

    // Clock helpers
    mutable std::function<void(const std::string&)> m_clockStart;
    mutable std::function<void(const std::string&)> m_clockStop;

    // Hooks
    std::unordered_map<std::string, CreatedHook> m_createdHooks;
    std::unordered_map<std::string, DeletedHook> m_deletedHooks;
    std::unordered_map<std::string, BrokenHook>  m_brokenHooks;
    std::unordered_map<std::string, MergedHook>  m_mergedHooks;
    bool m_enable{true};

    static constexpr const bool s_debug = false;  ///< Enable debug messages.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename RoadmapType>
CCTracker<RoadmapType>::
CCTracker(RoadmapType* const _r) : m_roadmap(_r) {
  RecomputeCCs();

  // Initialize the clock start/stoppers.
  SetStatClass(nullptr);
}


template <typename RoadmapType>
CCTracker<RoadmapType>::
~CCTracker() {
  for(VertexSet* const cc : m_components)
    delete cc;
}

/*------------------------------ Move and Copy -------------------------------*/

template <typename RoadmapType>
CCTracker<RoadmapType>::
CCTracker(const CCTracker& _other) {
  *this = _other;
}


template <typename RoadmapType>
CCTracker<RoadmapType>::
CCTracker(CCTracker&& _other) {
  *this = std::move(_other);
}


template <typename RoadmapType>
CCTracker<RoadmapType>&
CCTracker<RoadmapType>::
operator=(const CCTracker& _other) {
  m_roadmap    = _other.m_roadmap;
  m_clockStart = _other.m_clockStart;
  m_clockStop  = _other.m_clockStop;

  // Copy the components. This must be done manually or we'll be sharing
  // pointed-to VertexSets with _other.
  m_components.clear();
  m_components.reserve(_other.m_components.size());
  m_vidToCC.clear();
  m_vidToCC.reserve(_other.m_vidToCC.size());
  for(const VertexSet* const otherCC : _other.m_components) {
    VertexSet* const myCC = new VertexSet;
    m_components.insert(myCC);
    for(const VID vid : *otherCC) {
      myCC->insert(vid);
      m_vidToCC.emplace(vid, myCC);
    }
  }

  return *this;
}


template <typename RoadmapType>
CCTracker<RoadmapType>&
CCTracker<RoadmapType>::
operator=(CCTracker&& _other) {
  m_roadmap    = _other.m_roadmap;
  m_components = std::move(_other.m_components);
  // Clear _other's components to ensure they aren't deleted with it.
  _other.m_components.clear();
  m_vidToCC    = std::move(_other.m_vidToCC);
  m_clockStart = std::move(_other.m_clockStart);
  m_clockStop  = std::move(_other.m_clockStop);

  return *this;
}

/*--------------------------------- Queries ----------------------------------*/

template <typename RoadmapType>
inline
void
CCTracker<RoadmapType>::
SetRoadmap(RoadmapType* const _r) noexcept {
  m_roadmap = _r;
}


template <typename RoadmapType>
inline
void
CCTracker<RoadmapType>::
SetStatClass(StatClass* const _stats) const noexcept {
  // If we got a null pointer, set the clock stop/start functions to dummies.
  if(!_stats) {
    m_clockStart = [](const std::string&){};
    m_clockStop  = [](const std::string&){};
    return;
  }

  m_clockStart = [_stats](const std::string& _label) {
    _stats->StartClock(_label);
  };
  m_clockStop = [_stats](const std::string& _label) {
    _stats->StopClock(_label);
  };
}

/*--------------------------------- Queries ----------------------------------*/

template <typename RoadmapType>
inline
size_t
CCTracker<RoadmapType>::
GetNumCCs() const noexcept {
  return m_components.size();
}


template <typename RoadmapType>
const typename CCTracker<RoadmapType>::VertexSet*
CCTracker<RoadmapType>::
GetCC(const VID _vid) const noexcept {
  return FindCC(_vid);
}



template <typename RoadmapType>
typename CCTracker<RoadmapType>::VertexSet
CCTracker<RoadmapType>::
GetRepresentatives() const noexcept {
  VertexSet representatives;
  for(const VertexSet* const cc : m_components)
    representatives.insert(*cc->begin());
  return representatives;
}


template <typename RoadmapType>
inline
bool
CCTracker<RoadmapType>::
InSameCC(const VID _vid1, const VID _vid2) const noexcept {
  return FindCC(_vid1) == FindCC(_vid2);
}

/*-------------------------------- Iteration ---------------------------------*/

template <typename RoadmapType>
inline
typename CCTracker<RoadmapType>::ComponentSet::const_iterator
CCTracker<RoadmapType>::
begin() const noexcept {
  return m_components.begin();
}


template <typename RoadmapType>
inline
typename CCTracker<RoadmapType>::ComponentSet::const_iterator
CCTracker<RoadmapType>::
end() const noexcept {
  return m_components.end();
}

/*-------------------------------- Event Hooks -------------------------------*/

template <template <typename...> class MapType, typename ValueType,
          typename... Rest>
void
MapUniqueInsert(MapType<std::string, ValueType, Rest...>& _map,
    const std::string& _label, const ValueType& _value) noexcept {
  if(_map.count(_label))
    throw RunTimeException(WHERE) << "Map already has an element labeled '"
                                  << _label << "'.";
  _map.emplace(_label, _value);
}


template <typename RoadmapType>
void
CCTracker<RoadmapType>::
InstallCreatedHook(const std::string& _label, const CreatedHook& _h) {
  MapUniqueInsert(m_createdHooks, _label, _h);
}


template <typename RoadmapType>
void
CCTracker<RoadmapType>::
InstallDeletedHook(const std::string& _label, const DeletedHook& _h) {
  MapUniqueInsert(m_deletedHooks, _label, _h);
}


template <typename RoadmapType>
void
CCTracker<RoadmapType>::
InstallBrokenHook(const std::string& _label, const BrokenHook& _h) {
  MapUniqueInsert(m_brokenHooks, _label, _h);
}


template <typename RoadmapType>
void
CCTracker<RoadmapType>::
InstallMergedHook(const std::string& _label, const MergedHook& _h) {
  MapUniqueInsert(m_mergedHooks, _label, _h);
}


template <typename RoadmapType>
void
CCTracker<RoadmapType>::
Disable() {
  m_enable = false;
}


template <typename RoadmapType>
void
CCTracker<RoadmapType>::
Enable() {
  m_enable = true;
}

/*----------------------------------- Debug ----------------------------------*/

template <typename RoadmapType>
void
CCTracker<RoadmapType>::
Print(std::ostream& _os, const std::string& _indent) const {
  _os << _indent << "There are " << GetNumCCs() << " CCs." << std::endl;
  for(const VertexSet* const cc : m_components)
    _os << _indent << "  CC " << cc << ": " << *cc << std::endl;
}

/*--------------------------------- Updates ----------------------------------*/

template <typename RoadmapType>
void
CCTracker<RoadmapType>::
AddVertex(const vertex_iterator _vi) noexcept {
  if(!m_enable)
    return;

  m_clockStart("CCTracker::AddVertex");
  nonstd::call_on_destruct stopper(m_clockStop, "CCTracker::AddVertex");

  // Ensure we haven't already added this vertex.
  const VID vid = _vi->descriptor();
  if(m_vidToCC.count(vid))
    throw RunTimeException(WHERE) << "Vertex " << vid << " is already in a CC.";

  // Create a new CC for this vertex.
  VertexSet* const cc = new VertexSet{vid};
  m_components.emplace(cc);
  m_vidToCC.emplace(vid, cc);

  if(s_debug)
    std::cout << "Added vertex " << vid << " to new CC " << cc << ". "
              << "There are now " << GetNumCCs() << " CCs."
              << std::endl;

  for(const auto& hook : m_createdHooks)
    hook.second(this, cc);
}


template <typename RoadmapType>
void
CCTracker<RoadmapType>::
DeleteVertex(const vertex_iterator _vi) noexcept {
  if(!m_enable)
    return;

  m_clockStart("CCTracker::DeleteVertex");
  nonstd::call_on_destruct stopper(m_clockStop, "CCTracker::DeleteVertex");

  // We will assume that the edges are already deleted, as will normally be done
  // by the roadmap. Enforce that assumption now.
  if(_vi->begin() != _vi->end())
    throw RunTimeException(WHERE) << "Edges should be deleted prior to deleting "
                                  << "a vertex.";

  // Locate the CC.
  const VID vid = _vi->descriptor();
  VertexSet* const cc = FindCC(vid);

  // Assert that this is a CC of one vertex.
  if(cc->size() != 1)
    throw RunTimeException(WHERE) << "Found CC for vertex " << vid
                                  << " with " << cc->size()
                                  << " != 1 vertices.";
  // Assert that the one vertex is the one being deleted.
  if(!cc->count(vid))
    throw RunTimeException(WHERE) << "CC for vertex " << vid
                                  << " is missing the query VID. Contents: "
                                  << *cc;

  for(const auto& hook : m_deletedHooks)
    hook.second(this, cc);

  // Remove the CC.
  m_components.erase(cc);
  delete cc;
  m_vidToCC.erase(vid);

  if(s_debug) {
    std::cout << "Deleted vertex " << vid << " and CC " << cc << "."
              << std::endl;
    Print(std::cout, "  ");
  }

  return;
}


template <typename RoadmapType>
void
CCTracker<RoadmapType>::
AddEdge(const edge_iterator _ei) noexcept {
  if(!m_enable)
    return;

  /// @note This is linear in the size of the smaller CC if a merge occurs or
  ///       constant amortized otherwise.
  m_clockStart("CCTracker::AddEdge");
  nonstd::call_on_destruct stopper(m_clockStop, "CCTracker::AddEdge");

  // Adding an edge should always merge two CCs, which may already be the same.
  const VID sourceVID = (*_ei).source(),
            targetVID = (*_ei).target();
  VertexSet* const sourceCC = FindCC(sourceVID),
           * const targetCC = FindCC(targetVID);

  if(s_debug)
    std::cout << "Added edge (" << sourceVID << ", " << targetVID << ")."
              << endl;

  // If the two are already the same, there is nothing to do.
  if(sourceCC == targetCC) {
    if(s_debug)
      std::cout << "  Vertices are already in the same CC."
                << std::endl;
    return;
  }

  // Otherwise determine which CC is larger (by number of vertices).
  VertexSet* smallCC, * largeCC;
  if(sourceCC->size() < targetCC->size()) {
    smallCC = sourceCC;
    largeCC = targetCC;
  }
  else {
    smallCC = targetCC;
    largeCC = sourceCC;
  }

  // Call hooks before merging.
  for(const auto& hook : m_mergedHooks)
    hook.second(this, largeCC, smallCC);

  // Merge the smaller CC into the larger one.
  std::copy(smallCC->begin(), smallCC->end(),
      std::inserter(*largeCC, largeCC->begin()));

  // Update the CCs for the vertices which used to be in the small set.
  for(const VID vid : *smallCC)
    m_vidToCC[vid] = largeCC;

  // Remove the small CC.
  m_components.erase(smallCC);
  delete smallCC;

  if(s_debug) {
    std::cout << "  Merged CC " << smallCC << " into " << largeCC << "."
              << std::endl;
    Print(std::cout, "  ");
  }
}


template <typename RoadmapType>
void
CCTracker<RoadmapType>::
DeleteEdge(const edge_iterator _ei) noexcept {
  if(!m_enable)
    return;

  m_clockStart("CCTracker::DeleteEdge");
  nonstd::call_on_destruct stopper(m_clockStop, "CCTracker::DeleteEdge");

  // Can't use -> here because there's no const-qualified version in STAPL.
  const VID source = (*_ei).source(),
            target = (*_ei).target();

  if(s_debug)
    std::cout << "Deleted edge (" << source << ", " << target << ")."
              << endl;

  // Assert that both cfgs started out in the same CC.
  VertexSet* const sourceCC = FindCC(source),
           * const targetCC = FindCC(target);
  if(sourceCC != targetCC) {
    const VertexSet discovered = BFS(0);
    std::cout << "  Roadmap has " << m_roadmap->Size() << " vertices.\n"
              << "  CC from root has " << discovered.size() << " vertices:\n"
              << "\t" << discovered
              << std::endl;
    throw RunTimeException(WHERE) << "Edge vertices are not in the same CC.";
  }

  // Check if the reverse edge exists. If so, this is an undirected graph and
  // the (weak) CC is unchanged.
  {
    edge_iterator ei;
    const bool exists = m_roadmap->GetEdge(target, source, ei);
    if(exists) {
      if(s_debug)
        std::cout << "  Vertices are still connected (reverse edge exists)."
                  << std::endl;
      return;
    }
  }

  // This is either a directed tree with a now broken CC or an undirected graph
  // with a maybe broken CC. Run a BFS from target and stop early if we find the
  // source (which indicates undirected graph).
  VertexSet discovered = BFS(target, {source});

  // If we found the source vertex, the vertices are still connected.
  if(discovered.count(source)) {
    if(s_debug)
      std::cout << "  Vertices are still connected."
                << std::endl;
    return;
  }

  // Otherwise they are now disconnected. Move the discovered vertices into
  // their own new CC.
  VertexSet* const newCC = new VertexSet;
  m_components.insert(newCC);
  for(const VID vid : discovered) {
    sourceCC->erase(vid);
    newCC->insert(vid);
    m_vidToCC[vid] = newCC;
  }

  if(s_debug) {
    std::cout << "  Split off new CC " << newCC << " from "
              << m_vidToCC.at(source) << "."
              << std::endl;
    Print(std::cout, "  ");
  }

  for(const auto& hook : m_brokenHooks)
    hook.second(this, sourceCC, newCC);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename RoadmapType>
const typename CCTracker<RoadmapType>::VertexSet*
CCTracker<RoadmapType>::
FindCC(const VID _vid) const noexcept {
  // Assert that we have the CC id for this vertex.
  auto iter = m_vidToCC.find(_vid);
  if(iter == m_vidToCC.end())
    throw RunTimeException(WHERE) << "No CC found for vertex " << _vid << ".";

  // Assert that the component is present in the component set.
  VertexSet* const cc = iter->second;
  if(!m_components.count(cc))
    throw RunTimeException(WHERE) << "No CC " << cc << " found for vertex "
                                  << _vid << ".";
  return cc;
}


template <typename RoadmapType>
inline
typename CCTracker<RoadmapType>::VertexSet*
CCTracker<RoadmapType>::
FindCC(const VID _vid) noexcept {
  // Use const version and cast back (since we know we have a mutable handle
  // this is OK).
  const VertexSet* const cc = const_cast<const CCTracker<RoadmapType>*>(this)->
      FindCC(_vid);
  return const_cast<VertexSet*>(cc);
}


template <typename RoadmapType>
void
CCTracker<RoadmapType>::
RecomputeCCs() noexcept {
  /// @warning This function assumes that VIDs grow in order of construction to
  ///          handle directed graphs.
  /// @TODO Remove the assumption. We will need to use predecessor information
  ///       to do that.

  // Clear any previous CC info.
  m_components.clear();
  m_vidToCC.clear();

  // Build a list of all vertices.
  std::set<VID> unmappedDescriptors;
  for(auto iter = m_roadmap->begin(); iter != m_roadmap->end(); ++iter)
    unmappedDescriptors.insert(iter->descriptor());

  // Find CCs while unmapped vertices remain.
  while(!unmappedDescriptors.empty()) {
    // Get the next descriptor and compute its CC.
    const VID current = *unmappedDescriptors.begin();
    VertexSet* const cc = new VertexSet(BFS(current));
    m_components.insert(cc);

    // Remove the discovered CC from the unmapped vertex set.
    for(const VID vid : *cc) {
      unmappedDescriptors.erase(vid);
      m_vidToCC.emplace(vid, cc);
    }
  }
}


template <typename RoadmapType>
typename CCTracker<RoadmapType>::VertexSet
CCTracker<RoadmapType>::
BFS(const VID _start, VertexSet _goals) const noexcept {
  // If we received a set of goal VIDs, we are only interested in determining
  // whether the _start is connected to all of them. In that case, we will stop
  // early upon discovering this information.
  const bool testGoals = !_goals.empty();

  // Set up a discovered list and queue for BFS.
  VertexSet discovered{_start};
  std::queue<VID> queue;
  queue.push(_start);

  // Run BFS.
  while(!queue.empty()) {
    // Get the next node in the queue.
    const VID current = queue.front();
    queue.pop();

    // Enqueue undiscovered children.
    auto vi = m_roadmap->find_vertex(current);
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      // Get the child VID.
      const VID child = ei->target();

      // Skip if the child is discovered.
      if(discovered.count(child))
        continue;
      discovered.insert(child);

      // Check if this child is a goal.
      if(testGoals and _goals.count(child)) {
        // The child is a goal, remove it from the goals list.
        _goals.erase(child);

        // If we've visited all goals, we're done.
        if(_goals.empty())
          return discovered;
      }

      // Enqueue the child.
      queue.push(child);
    }
  }

  // The set of discovered nodes comprise the CC.
  return discovered;
}

/*----------------------------------------------------------------------------*/

#endif
