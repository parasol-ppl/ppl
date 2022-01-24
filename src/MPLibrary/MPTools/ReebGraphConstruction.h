#ifndef PMPL_REEB_GRAPH_CONSTRUCTION_H_
#define PMPL_REEB_GRAPH_CONSTRUCTION_H_

#include <vector>
#include <unordered_set>
using namespace std;

#include <boost/functional/hash.hpp>

#include <containers/sequential/graph/directed_preds_graph.h>

#include <Vector.h>
using namespace mathtool;

#include "Utilities/IOUtils.h"

class Environment;
class WorkspaceDecomposition;
class WorkspaceSkeleton;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MPTools
/// Compute an embedded reeb graph of free workspace.
///
/// Computation requires a proper tetrahedralization for the computation to
/// work. ReebGraph comprises of ReebNodes and ReebArcs. Nodes represent the
/// critical values of a Morse function, i.e., min, max, saddle, and Arcs are
/// homotopy classes to get between these nodes.
///
/// For details of algorithm see: V. Pascucci, G. Scorzelli, P-T. Bremer, A.
/// Mascarenhas, "Robust On-line Computation of Reeb Graphs: Simplicity and
/// Speed," ACM Trans. on Graphics, 26.3.58, July 2007, and In. Proc. of ACM
/// SIGGRAPH, 2007.
////////////////////////////////////////////////////////////////////////////////
class ReebGraphConstruction {

  public:

    ///@name Local Types
    ///@{

    /// The parameters for this object.
    struct Parameters {
      std::string filename;  ///< Input/output filename for embedded reeb graph.
      bool write{false};     ///< Output embedded reeb graph?
      bool read{false};     ///< Read in embedded reeb graph?
      bool debug{false};     ///< Show debug messages?
    };

    static Parameters m_defaultParams; ///< The default parameters.

    typedef tuple<size_t, size_t, size_t> Triangle; ///< Triangle: 3 indices for
                                                    ///< vertex array.

    ////////////////////////////////////////////////////////////////////////////
    /// Vertex property of ReebGraph
    ////////////////////////////////////////////////////////////////////////////
    struct ReebNode {

      /// @param _vIndx Vertex Index
      /// @param _v Vertex
      /// @param _w Morse function value
      ReebNode(size_t _vIndx = -1, const Vector3d& _v = Vector3d(),
          double _w = numeric_limits<double>::max()) :
          m_vertexIndex(_vIndx), m_vertex(_v), m_w(_w) {}

      friend istream& operator>>(istream& _is, ReebNode& _rn);
      friend ostream& operator<<(ostream& _os, const ReebNode& _rn);

      size_t m_vertexIndex; ///< Vertex Index
      Vector3d m_vertex;    ///< Vertex
      double m_w;           ///< Morse function value
      size_t m_order{0};    ///< Total ordering of vertex
      size_t m_tetra{0};    ///< Tetrahedron index from embedding process
    };

    ////////////////////////////////////////////////////////////////////////////
    /// Comparator of ReebNodes. Ordering like tuple: (1) morse function
    /// value (2-4) vertex[x-z] (5) vertex index
    ////////////////////////////////////////////////////////////////////////////
    struct ReebNodeComp {

      /// @param _a Node 1
      /// @param _b Node 2
      /// @return _a < _b
      bool operator()(const ReebNode& _a, const ReebNode& _b) {

        /// Vertex x, y, z ordering
        /// @param _a Vertex 1
        /// @param _b Vertex 2
        /// @return _a < _b
        auto vertComp = [](const Vector3d _a, const Vector3d _b) {
          return _a[0] - _b[0] > ReebTolerance() ||
            (fabs(_a[0] - _b[0]) < ReebTolerance() && (_a[1] - _b[1] > ReebTolerance() ||
            (fabs(_a[1] - _b[1]) < ReebTolerance() && _a[2] - _b[2] > ReebTolerance())));
        };

        const Vector3d& a = _a.m_vertex;
        const Vector3d& b = _b.m_vertex;
        bool ret = _b.m_vertexIndex != _a.m_vertexIndex &&
          (_a.m_w - _b.m_w > ReebTolerance() ||
           (fabs(_a.m_w - _b.m_w) < ReebTolerance() &&
            (vertComp(a, b) || (a == b &&
            _a.m_vertexIndex < _b.m_vertexIndex))));
        return ret;
      }

      /// @return Floating-point tolerance for equality
      static constexpr double ReebTolerance() {return 0.000001;}
    };

    struct MeshEdge;

    ////////////////////////////////////////////////////////////////////////////
    /// Edge property of ReebGraph. Arc stores set of MeshEdges and
    /// tetrahedron correspondingly.
    ////////////////////////////////////////////////////////////////////////////
    struct ReebArc {

      /// @param _s Source vertex index
      /// @param _t Target vertex index
      /// @param _m Corresponding mesh edge for initializing
      ReebArc(size_t _s = -1, size_t _t = -1, MeshEdge* _m = nullptr) :
        m_source(_s), m_target(_t) {
          if(_m)
            m_edges.insert(_m);
        }

      friend istream& operator>>(istream& _is, ReebArc& _ra);
      friend ostream& operator<<(ostream& _os, const ReebArc& _ra);

      size_t m_source;                  ///< Source vertex index
      size_t m_target;                  ///< Target vertex index
      unordered_set<MeshEdge*> m_edges; ///< Related mesh edges
      unordered_set<size_t> m_tetra;    ///< Related tetrahedron
      vector<Vector3d> m_path;          ///< Embedded ReebArc
    };

    typedef stapl::sequential::directed_preds_graph<
      stapl::MULTIEDGES, ReebNode, ReebArc>
      ReebGraph; ///< ReebGraph is a directed, multiedge graph between ReebNode
                 ///< and ReebArc
    typedef ReebGraph::edge_descriptor RGEID; ///< ReebGraph edge descriptor

    ////////////////////////////////////////////////////////////////////////////
    /// Comparator of reeb arcs. Provides total ordering based on (1) source
    /// vertex ordering (2) target vertex ordering (3) Edge id
    ////////////////////////////////////////////////////////////////////////////
    struct ReebArcComp {

      /// @param _rg Pointer to ReebGraph
      ReebArcComp(ReebGraph* _rg) : m_rg(_rg) {}

      /// @param _a Edge descriptor 1
      /// @param _b Edge descriptor 2
      /// @return _a < _b
      bool operator()(const RGEID& _a, const RGEID& _b) {
        bool ret = false;
        ReebNode& s0 = m_rg->find_vertex(_a.source())->property();
        ReebNode& s1 = m_rg->find_vertex(_b.source())->property();
        if(s0.m_order < s1.m_order)
          ret = true;
        else if(s0.m_order == s1.m_order) {
          ReebNode& t0 = m_rg->find_vertex(_a.target())->property();
          ReebNode& t1 = m_rg->find_vertex(_b.target())->property();
          if(t0.m_order < t1.m_order)
            ret = true;
          else if(t0.m_order == t1.m_order)
            if(_a.id() < _b.id())
              ret = true;
        }
        return ret;
      }

      ReebGraph* m_rg; ///< ReebGraph pointer
    };

    typedef set<RGEID, ReebArcComp> ArcSet; ///< Set of ReebArcs

    ////////////////////////////////////////////////////////////////////////////
    /// Tetrahedralization edge. Stores associates ReebEdges
    ////////////////////////////////////////////////////////////////////////////
    struct MeshEdge {

      /// @param _s Source vertex index
      /// @param _t Target vertex index
      /// @param _rg ReebGraph pointer
      MeshEdge(size_t _s, size_t _t, ReebGraph* _rg) :
        m_source(_s), m_target(_t), m_numTris(0), m_arcs(ReebArcComp(_rg)) {
        }

      size_t m_source;  ///< Source vertex index
      size_t m_target;  ///< Target vertex index
      size_t m_numTris; ///< Number of triangles which it is incident on
      ArcSet m_arcs;    ///< Set of Reeb Arcs
    };

    ////////////////////////////////////////////////////////////////////////////
    /// Hash function for MeshEdge. Hash based on source/target pair.
    ////////////////////////////////////////////////////////////////////////////
    struct MeshEdgeHash {

      /// @param _m MeshEdge pointer
      /// @return Hash value
      size_t operator()(const MeshEdge* const _m) const {
        return h(make_pair(_m->m_source, _m->m_target));
      }

      boost::hash<pair<size_t, size_t>> h; ///Hash function for evaluation
    };

    ////////////////////////////////////////////////////////////////////////////
    /// Mesh edge equivalence. Based on source/target pair.
    ////////////////////////////////////////////////////////////////////////////
    struct MeshEdgeEq {

      /// @param _a MeshEdge 1
      /// @param _b MeshEdge 2
      /// @return _a == _b
      bool operator()(const MeshEdge* const _a, const MeshEdge* const _b) const {
        return _a->m_source == _b->m_source && _a->m_target == _b->m_target;
      }
    };

    typedef stapl::sequential::directed_preds_graph<
      stapl::MULTIEDGES, Vector3d, vector<Vector3d>
        > FlowGraph; ///< Flow graph is a directed multiedge graph of points and
                     ///< paths. Is a flow of the embedded ReebGraph.

    ///@}
    ///@name Construction
    ///@{

    ReebGraphConstruction();

    /// Set the default parameters from an XML node.
    /// @param _node The XML node to parse.
    static void SetDefaultParameters(XMLNode& _node);

    ///@}
    ///@name Operations
    ///@{

    /// Construct a Reeb graph of an environment decomposition.
    /// @param _decomposition The workspace decomposition to use.
    /// @param _baseInputPath Base file path for input files.
    /// @param _baseFilename Base filename used for saving models
    void Construct(const WorkspaceDecomposition* _decomposition);

    /// Extract a workspace skeleton from the Reeb graph.
    /// @TODO Make this a const function if STAPL ever fixes the sequential
    ///       graph.
    WorkspaceSkeleton GetSkeleton();

    ///@}
    ///@name Accessors
    ///@{
    ///@TODO Fix const-ness of accessors when STAPL fixes sequential graph.

    /// Get the embedding graph
    ReebGraph& GetReebGraph() {return m_reebGraph;}

    ///@}
    ///@name I/O
    ///@{

    /// Read embedding graph from file
    /// @param _filename Filename
    void Read(const string& _filename);

    /// Write embedding graph to file
    /// @param _filename Filename
    void Write(const string& _filename);

    ///@}

  private:

    ///@name Construction Helpers
    ///@{

    /// Populate ReebGraph structures for construction
    /// @param _decomposition The workspace decomposition to use.
    void Initialize(const WorkspaceDecomposition* _decomposition);

    /// Construct Reeb Graph based on algorithm presented in class description
    ///
    /// Construction Algorithm
    /// - Input: Tetrahedralization t
    /// - Output: Reeb Graph
    /// - 1. Create ReebNode for each vertex of t
    /// - 2. Create ReebArc for each edge of t
    /// - 3. For each triangle in t
    /// - 4.   MergePaths(t)
    /// - 5.   if any edge of t is finished, remove edge from Reeb Graph
    /// - 6. Return Reeb Graph
    void Construct();

    /// Add ReebNode for vertex \c _v
    /// @param _i Index of vertex
    /// @param _v Vertex point
    /// @param _w Morse function value of vertex
    void CreateNode(size_t _i, const Vector3d& _v, double _w);

    /// Add ReebArc to graph for mesh edge
    /// @param _s Source vertex index
    /// @param _t Target vertex index
    /// @param _tetra Subset of tetrahedrons adjecent to edge
    /// @return MeshEdge between vertices \c _s and \c _t
    ///
    /// If edge between \c _s and \c _t has not been encountered, a ReebArc will
    /// be created first before returning appropriate edge of
    /// tetrahedralization. All tetrahedron in \c _tetra will be associated to
    /// proper ReebArcs.
    MeshEdge* CreateArc(size_t _s, size_t _t,
        const unordered_set<size_t>& _tetra = unordered_set<size_t>());

    /// Merge triangle of tetrahedralization
    /// @param _e0 Edge 1
    /// @param _e1 Edge 2
    /// @param _e2 Edge 3
    ///
    /// \c _e0, \c _e1, \c _e2 must have proper order. \c _e0 and \c _e1 must
    /// share maximum vertex of triangle. \c _e0 and \c _e2 must share minimum.
    ///
    /// Algorithm from paper
    /// - 1. a0 = Arc(e0); a1 = Arc(e1); a2 = Arc(e2)
    /// - 2. GlueByMergeSorting(a0, a1, e0, e1)
    /// - 3. GlueByMergeSorting(a0, a1, e0, e2)
    void MergePaths(MeshEdge* _e0, MeshEdge* _e1, MeshEdge* _e2);

    /// Merge two ReebArcs. Similar process to merging two arrays in
    ///        MergeSort
    /// @param _a0 ArcSet 1
    /// @param _e0 Edge 1
    /// @param _a1 ArcSet 2
    /// @param _e1 Edge 2
    ///
    /// Algorithm From paper
    /// - while(a0 and a1 are "valid" arcs) do
    /// -   n0 <- BottomNode(a0)
    /// -   n1 <- BottomNode(a1)
    /// -   if f(n0) > f(n1) then
    /// -     MergeArcs(a0, a1)
    /// -   else
    /// -     MergeArcs(a1, a0)
    /// -   a0 <- NextArcMappedToEdge(a0, e0)
    /// -   a1 <- NextArcMappedToEdge(a1, e1)
    void GlueByMergeSorting(ArcSet& _a0, MeshEdge* _e0,
        ArcSet& _a1, MeshEdge* _e1);

    /// Merge second reeb arc into first
    /// @param _a0 ReebArc 1
    /// @param _a1 ReebArc 2
    ///
    /// Merge data of \c _a1 into \c _a0. Sources of \c _a0 and \c _a1 must be
    /// equal. Two cases occur with targets of edges. If targets are equivalent
    /// \c _a1 is entirely removed from the ReebGraph. Otherwise, \c _a1 is
    /// replaced by an edge which spans the target of \c _a0 to the target of
    /// \c _a1.
    void MergeArcs(RGEID _a0, RGEID _a1);

    /// @param _a ReebGraph edge descriptor
    /// @return ReebArc
    ReebArc& GetReebArc(RGEID _a);

    /// Remove edge from all associated ReebArcs
    /// @param _e Edge of tetrahedralization
    void DeleteMeshEdge(MeshEdge* _e);

    /// Remove 2-nodes from ReebGraph to reduce size of graph
    void Remove2Nodes();

    /// Spatially embed reeb graph based on algorithm presented in class
    ///        description
    ///
    /// Embedding algorithm relates each Reeb Node to its closest tetrahedron
    /// and finds an embedded ReebArc by biasing a path search in the
    /// tetrahedralization's dual graph.
    void Embed(const WorkspaceDecomposition* _tetrahedralization);

    /// Make sure every node is unique.
    void Uniqueify();

    ///@}
    ///@name Internal State
    ///@{

    Parameters m_params; ///< The input parameters for this instance.

    vector<Vector3d> m_vertices; ///< Vertices of tetrahedralization

    /// Edges of Tetrahedralization
    unordered_set<MeshEdge*, MeshEdgeHash, MeshEdgeEq> m_edges;

    /// Triangles of Tetrahedralization and their corresponding tetrahedrons.
    vector<pair<Triangle, unordered_set<size_t>>> m_triangles;

    ReebGraph m_reebGraph;       ///< Reeb Graph

    ///@}
};

#endif
