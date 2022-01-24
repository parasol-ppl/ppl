#ifndef MEAN_CURVATURE_3D_H_
#define MEAN_CURVATURE_3D_H_

#ifndef CGAL_EIGEN3_ENABLED
#define CGAL_EIGEN3_ENABLED
#endif

#include <memory>
#include <map>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <tuple>
#include <sstream>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <containers/sequential/graph/graph.h>
#include <containers/sequential/graph/algorithms/graph_algo_util.h>
#include <boost/functional/hash.hpp>
#include "Utilities/IOUtils.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Workspace/WorkspaceSkeleton.h"
#include "Workspace/WorkspaceDecomposition.h"
#include "Workspace/PropertyMap.h"
#include "Vector.h"

using namespace std;
using namespace mathtool;

class Environment;
class XMLNode;
class WorkspaceDecomposition;


/////////////////////////////////////////////////////////////////////////////
/// 3D Mean Curvature Skeleton implementation from CGAL
/////////////////////////////////////////////////////////////////////////////
class MeanCurvatureSkeleton3D {

  public:

    struct Parameters {
      double m_wM{0.2};
      double m_wH{0.1};
      double m_refine{1};
      size_t m_iterations{1000};
      std::string ioType;
      std::string filebase;
      bool useConvex{false};
      double scaleConvex{1.2};
      size_t space;
    };

    static Parameters m_defaultParams; ///< Parameters

    ///@name Local types
    ///{
    /// Note: Please do not change the kernel to exact computation that will
    /// hamper the computation of mean curvature skeleton internal functions
    typedef CGAL::Simple_cartesian<double> CGALKernel;
    typedef typename CGALKernel::Point_3 Point3;
    typedef CGAL::Surface_mesh<Point3> SurfaceMesh;
    typedef CGAL::Polyhedron_3<CGALKernel, CGAL::Polyhedron_items_with_id_3> Polyhedron;
    typedef CGAL::Mean_curvature_flow_skeletonization<SurfaceMesh> Skeletonization;
    typedef Skeletonization::Skeleton   MCSkeleton;
    typedef MCSkeleton::vertex_descriptor    MCVD;
    typedef MCSkeleton::edge_descriptor      MCED;
    typedef Skeletonization::Meso_skeleton  MESO;
    typedef vector<Point3d> WitnessType;
    typedef PropertyMap<vector<WitnessType>, WitnessType> AnnotationType;

    ///@}
    ///@name Construction
    ///@{

    MeanCurvatureSkeleton3D();

    /// Construct a Mean Curvature Skeleton from an XML node.
    /// @param _node The XML node to parse.
    static void SetDefaultParameters(XMLNode& _node);

    /// Initializes the input mesh as the obstacle space of a given environment
    /// @param _env The environment to use.
    void SetEnvironment(const Environment* _env);

    /// Initialize the input mesh from the input exact polyhedron
    /// @param _poly The polyhedron to use.
    /// @param _d Set to debug mode or not.
    template<typename PolyhedronType>
      MeanCurvatureSkeleton3D(PolyhedronType& _poly, bool _d = false) : m_debug(_d) {
        AddPolyhedron(_poly);
      }

    ~MeanCurvatureSkeleton3D();

    ///@}
    ///@name IO functions
    ///@{

    /// Write MeanCurvatureSkeleton3D to a file
    /// @param _filename The filename to write to.
    void Write(string _filename);

    /// Read MeanCurvatureSkeleton3D from a file
    /// @param _filename The filename to read from.
    void Read(string _filename);

    ///@}
    ///@name Modifiers
    ///@{

    /// Add an input polyhedron in the input mesh
    template<typename PolyhedronType>
    void AddPolyhedron(PolyhedronType& _poly, double _refine = 1.32);

    /// Construct the mean curvature skeleton
    void BuildSkeleton(bool _curve = true);

    /// Construct the mean curvature skeleton from a decomposition
    void BuildSkeleton(const WorkspaceDecomposition* _decomposition, bool _curve = true);

    ///@}
    ///@name Accessors
    ///@{

    /// Get the workspace skeleton from the curved skeleton with surface mesh
    /// points as annotations of the skeleton
    pair<WorkspaceSkeleton, AnnotationType> GetSkeleton();

    /// Get the workspace skeleton from the meso skeleton with surface mesh
    /// points as annotations of the skeleton
    pair<WorkspaceSkeleton, AnnotationType> GetMesoSkeleton();

    /// Set the parameters to use to build and access the skeleton
    /// @param _p The parameters to use.
    void SetParameters(Parameters& _p) { m_params = _p; }

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Initializes the input mesh as the free space of a given environment
    void InitializeFreeSpace(const Environment* _env);

    /// Initializes the input mesh as the obstacle space of a given environment
    void InitializeObstacleSpace(const Environment* _env);

    /// Get the witness for a given skeleton point
    /// @param _v vertex descriptor of the skeleton point
    /// @param _witness Output witness points which are mesh surface points from
    /// which the skeleton is generated by contracting
    void GetWitnesses(MCVD& _v, vector<Point3d>& _witness);

    /// Get the witness for a given meso-skeleton point
    /// @param _v vertex descriptor of the skeleton point
    /// @param _witness Output witness points which are mesh surface points from
    /// which the skeleton is generated by contracting
    void GetWitnesses(MESO::Vertex& _v, vector<Point3d>& _witness);

    /// Get the witness for a given meso-skeleton point
    /// @param _v vertex descriptor of the skeleton point
    /// @param _witness Output witness points which are mesh surface points from
    /// which the skeleton is generated by contracting
    void GetWitnesses(Skeletonization::vertex_descriptor& _v, vector<Point3d>& _witness);

    /// Resets the input
    void Reset();

    /// Refinement of input mesh
    /// @param _f input parameter for Refinement
    void Refine(SurfaceMesh& _m, double _f=1.00);

    /// Changes the skeleton graph into set of polylines
    /// @param _plines Output polylines vector
    void ToPolyLines(vector<vector<MCVD>>& _plines, bool _isSimplify = true);

    /// Get the point for the vertex descriptor in skeleton
    Point3 Point(MCVD& _vd) { return m_mcs[_vd].point; }

    ///@}
    ///@name Internal State
    ///@{

    SurfaceMesh m_input; ///< Input surface mesh
    MCSkeleton m_mcs; ///< CGAL mean curvature skeleton
    MESO m_meso; ///< Meso skeleton
    WorkspaceSkeleton m_skeleton; ///< skeleton compatible with MPLibrary classes
    AnnotationType m_annotation;  ///< annotation graph for the skeleton
    bool m_debug{false};   ///< Toggle debug messages.
    Parameters m_params; ///< Parameters
    
    ///@}
};

/*------------------------Templated Modifiers---------------------------------*/
template<typename PolyhedronType>
void
MeanCurvatureSkeleton3D::
AddPolyhedron(PolyhedronType& _poly, double _refine) {
  stringstream ss;
  ss << _poly;
  ss >> m_input;
  Refine(m_input, _refine); //1.32
}

#endif
