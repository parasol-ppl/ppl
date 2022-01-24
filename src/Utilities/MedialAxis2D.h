#ifndef MEDIAL_AXIS_2D_H_
#define MEDIAL_AXIS_2D_H_

#include <memory>
#include <map>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <tuple>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Segment_Delaunay_graph_filtered_traits_2.h>
#include <CGAL/Segment_Delaunay_graph_2.h>
#include <CGAL/Triangulation_2.h>
#include <containers/sequential/graph/graph.h>
#include <containers/sequential/graph/algorithms/graph_algo_util.h>
#include <boost/functional/hash.hpp>

using namespace std;

#include "Vector.h"
using namespace mathtool;
#include "IOUtils.h"

#include "Geometry/GMSPolyhedron.h"
#include "Geometry/Boundaries/Boundary.h"
#include "SegmentTrees.h"
#include "Workspace/WorkspaceSkeleton.h"



/////////////////////////////////////////////////////////////////////////////
/// 2D Medial Axis construction using Segment Delaunay Graph
/////////////////////////////////////////////////////////////////////////////
class MedialAxis2D {

  public:
    ///@name Local types
    ///{
    typedef CGAL::Simple_cartesian<double> CK;
    typedef CGAL::Segment_Delaunay_graph_filtered_traits_2<CK,
            CGAL::Field_with_sqrt_tag>  Gt;
    typedef CGAL::Segment_Delaunay_graph_2<Gt>   SDG2;
    typedef typename SDG2::Site_2 Site2;
    typedef typename SDG2::Point_2 Point2;
    typedef typename SDG2::Edge Edge2;
    typedef typename SDG2::Data_structure DS;
    typedef typename Gt::Segment_2 Segment2;
    typedef CGAL::Parabola_segment_2<Gt> Parabola2;
    // Type for clearance info - first: clearance value, second: witness
    typedef pair<double,Point3d> ClearanceType;
    // Types for skeleton and its annotations
    typedef WorkspaceSkeleton::GraphType SkeletonGraphType;
    typedef unordered_map<WorkspaceSkeleton::VD, ClearanceType>
      VertexClearanceMapType;

    /// Medial axis edge structure
    struct MedialEdge {
      /// Constructors from line segment and parabola segment
      MedialEdge(Segment2& _s, Site2& _s1, Site2& _s2, double _ss = 0);
      MedialEdge(Parabola2& _p, Point2& _s, Point2& _t, Site2& _s1,
          Site2& _s2);
      MedialEdge(Point2& _s, Point2& _t, Site2& _s1, Site2& _s2);

      /// One line accessors
      const Point2& GetSource() { return m_interpolated.front();  }
      const Point2& GetTarget() { return m_interpolated.back(); }
      bool IsSegment() { return m_isLine; }

      /// To interpolate the line segment into set of points
      void InterpolateSegment(double _step);

      vector<Point2> m_interpolated;  ///< Interpolated points
      /// approximating edge
      Site2 m_site1, m_site2;	///< Witness sites
      bool m_isLine{true};      ///< Flag indicating line segment/parabola
      bool m_isFree{false};	///< Flag indicating edge in free or obstacle space
      friend class MedialAxis2D;
    };

    /// Underlying medial axis graph type
    typedef stapl::sequential::graph<stapl::UNDIRECTED,stapl::MULTIEDGES,
            Point2, MedialEdge>  MedialAxisGraph;

    /// Input polygon segment
    /// Structure defined to expedite point in polygon test
    struct PolygonSegment {
      // Constructor function
      PolygonSegment(Point2& _s, Point2& _t);
      // Check the point lies on the segment
      bool OnLine(const Point2& _p);
      Point2 Source() {	return m_end[0]; }
      Point2 Target() { return m_end[1]; }
      double GetConstant() { return m_constant;	}
      double GetMultiple() { return m_multiple;	}
      Point2 m_end[2];  ///< End points of the segment
      double m_constant;  ///< Constant for the line equation
      double m_multiple;	///< Multiple for the line equation
    };
    typedef vector<PolygonSegment> PolygonSegments;

    struct edgeHash {
      size_t operator()(const WorkspaceSkeleton::ED& _ed) const {
        size_t seed = 0;
        boost::hash_combine(seed,_ed.id());
        boost::hash_combine(seed,boost::hash_value(make_pair(_ed.source(),
                _ed.target())));
        return seed;
        //return hash<typename ED::edge_id_type>()(_ed.id());
      }
    };
    // Clearance map type for skeleton edges
    typedef unordered_map<WorkspaceSkeleton::ED,vector<ClearanceType>,
            edgeHash> EdgeClearanceMapType;
    ///@}
    ///@name Construction
    ///@{

    MedialAxis2D() : m_segTree(2){}
    MedialAxis2D(vector<GMSPolyhedron>& _polys, const Boundary* _b,
        vector<Boundary*> _bndrys=vector<Boundary*>());

    ///@}
    ///@name Modifiers
    ///@{

    /// Add a set of insert segments
    /// @param _s Segments.
    void AddSegments(vector<pair<Point3d,Point3d>>& _s);

    /// Add a set of insert segments
    /// @param _p Points.
    /// @param _s Segments.
    void AddSegments(vector<Point3d>& _p, vector<pair<size_t,size_t>>& _s);

    /// Add a set of points
    /// @param _p Points.
    void AddPoints(vector<Point3d>& _p);

    /// Filter out medial axis edges
    void BuildMedialAxis();

    ///@}
    ///@name Accessors
    ///@{

    /// Get the workspace skeleton from the medial axis
    /// @param _t Pass 0 for entire graph,
    ///     1 for free space and 2 for obstacle space
    tuple<WorkspaceSkeleton,VertexClearanceMapType,EdgeClearanceMapType>
      GetSkeleton(size_t _t=0);

    /// Calculete the clearance annotation of the interpolated points
    vector<ClearanceType> AnnotateSegment(MedialEdge& _e);

    double GetVertexClearance(size_t _i);
    Point3d GetVertexClearanceWitness(size_t _i);

    ///@}

  private:
    ///@name Helpers
    ///@{

    /// Add polygon
    void AddPolygon(const GMSPolyhedron& _p,
        vector<pair<Point3d,Point3d>>& _s, Boundary* _b = nullptr);

    /// Checks whether the edge is a skeleton edge
    bool IsSkeletonEdge(Edge2& _e);

    /// Checks whether the edge is a medial axis edge
    bool IsMedialAxisEdge(Edge2& _e, bool k=false);
    /// Checks whether the edge by end points is a bisector edge
    bool IsReflexBisector(const Point2& _p1, const Point2& _p2,
        Site2& _s1, Site2& _s2);

    /// Get minimum distance of the point from two respective sites
    ClearanceType GetMinDistance(const Point2& _p, Site2& _s1, Site2& _s2);

    /// Get the distance of a point from the site
    ClearanceType DistanceToSite(const Point2& _p, Site2& _s);

    /// Get the minimum distance along the medial axis segment from the sites
    double GetMinDistance(MedialEdge& _e);

    /// Get the maximum distance along the medial axis segment from the sites
    double GetMaxDistance(MedialEdge& _e);

    /// Calculate the clearance maps
    void CalculateClearance();
    /// Check whether the point is in free or obstacle space
    bool IsFree(const Point2& _p);
    /// Function to check whether a point is within boundary
    bool IsOutside(const Point2& _p);

    /// Function to check whether a point in edge is within boundary
    void ValidateEdge(MedialEdge& _e);

    /// Function to find the circumcenter of a triangle
    /// This function is used only when the cgal sdg cannot determine
    /// the center
    Point2 MidPoint(Site2& _o, Site2& _p, Site2& _q, Point2& _fp);

    ///@}
    ///@name Internal State
    ///@{

    SDG2 sdg; ///< CGAL Segment delaunay graph
    MedialAxisGraph m_graph;	///< Graph for the medial axis
    vector<PolygonSegments> m_polygons;	///< Set of segments that make up the polygons - to check for free space medial axis

    SegmentTrees<> m_segTree; ///< Segment Trees for easy query of candidate polygon in point in polygon tests

    unordered_map<size_t, ClearanceType> m_vertexClearance; ///< Clearance of vertices map

    bool m_debug{false};   ///< Toggle debug messages.
    Point3d m_boundary[2]; ///< Bounding box of all the input segments

    ///@}
};

#endif
