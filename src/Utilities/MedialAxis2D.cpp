#include "MedialAxis2D.h"
#include "Workspace/WorkspaceSkeleton.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include <CGAL/squared_distance_2.h>
#include <cmath>
#include <map>
#include <set>
#include <algorithm>

/*--------------------- Associate Structure Functions --------------------*/

MedialAxis2D::MedialEdge::
MedialEdge(Segment2& _s, Site2& _s1, Site2& _s2, double _ss) :
  m_site1(_s1), m_site2(_s2) {
  // Store the end points of the segments
  m_interpolated.emplace_back(_s.source());
  m_interpolated.emplace_back(_s.target());
  // Interpolate the the segment if required
  InterpolateSegment(_ss);
}

MedialAxis2D::MedialEdge::
MedialEdge(Parabola2& _p, Point2& _s, Point2& _t, Site2& _s1, Site2& _s2) :
  m_site1(_s1), m_site2(_s2) {
  // Generate the approximation of the parabola segment as set of points
  _p.generate_points(m_interpolated);
  // Remove duplicates and outside points
  for(size_t i = 1; i < m_interpolated.size(); ++i)
    if(m_interpolated[i-1] == m_interpolated[i]) {
      m_interpolated.erase(m_interpolated.begin()+i-1);
      --i;
    }

  // If the target is closest to the first interpolated point than source,
  // reverse the interpolated
  if(m_interpolated.size() > 1 && squared_distance(_s, m_interpolated[0]) >
      squared_distance(_t, m_interpolated[0]))
    reverse(m_interpolated.begin(), m_interpolated.end()) ;

  // Remove endpoints if it is duplicated in generate_points function
  if(m_interpolated.size() > 1 && m_interpolated[0]!= _s)
    m_interpolated.insert(m_interpolated.begin(), _s);
  if(!m_interpolated.empty() && m_interpolated.back() != _t)
    m_interpolated.emplace_back(_t);
  m_isLine = false;
}

MedialAxis2D::MedialEdge::
MedialEdge(Point2& _s, Point2& _t, Site2& _s1, Site2& _s2) :
  m_site1(_s1), m_site2(_s2) {
  // Store the end points of the segments
  m_interpolated.emplace_back(_s);
  m_interpolated.emplace_back(_t);
}

void
MedialAxis2D::MedialEdge::
InterpolateSegment(double _step) {
  if(_step == 0) return;
  Point2 s = m_interpolated.front();
  Point2 t = m_interpolated.back();
  // pop last value to insert interpolated points in between
  m_interpolated.pop_back();
  size_t i = 1;
  double len = sqrt((s-t).squared_length());
  // linear interpolation of the segment
  while (_step*i < len)	{
    m_interpolated.emplace_back(s + (t - s)*((_step*i)/len));
    ++i;
  }
  // push back the target
  m_interpolated.emplace_back(t);
}

MedialAxis2D::PolygonSegment::
PolygonSegment(Point2& _s, Point2& _t) {
  m_end[0] = _s;
  m_end[1] = _t;
  if(_s.y()== _t.y()) {
    m_constant=_t.x();
    m_multiple=0;
  }
  else {
    m_constant=_t.x()-(_t.y()*(_s.x()- _t.x()))/(_s.y()-_t.y());
    m_multiple=(_s.x()-_t.x())/(_s.y()-_t.y());
  }
}

bool
MedialAxis2D::PolygonSegment::
OnLine(const Point2& _p) {
  Segment2 s(m_end[0],m_end[1]);
  return s.has_on(_p);
}

/*------------------------- Constructors ----------------------------------*/
MedialAxis2D::
MedialAxis2D(vector<GMSPolyhedron>& _polys, const Boundary* _b,
    vector<Boundary*> _bndrys) : m_segTree(2) {
  vector<pair<Point3d,Point3d>> inputSegments;
  size_t i = 0;
  // Add and group all the polygon segments
  for(auto pit = _polys.begin(); pit != _polys.end(); ++pit, ++i)
    if(i<_bndrys.size())
      AddPolygon(*pit,inputSegments,_bndrys[i]);
    else
      AddPolygon(*pit,inputSegments);

  // Add the boundary edges if they exists
  if(_b) {
    // Get the boundary ranges
    auto xrange = _b->GetRange(0);
    auto yrange = _b->GetRange(1);
    // Get the boundary corners
    Point3d a(xrange.min, yrange.min, 0), b(xrange.max, yrange.min, 0),
            c(xrange.max, yrange.max, 0), d(xrange.min, yrange.max, 0);
    // Insert the boundary segments
    inputSegments.emplace_back(make_pair(a,b));
    inputSegments.emplace_back(make_pair(b,c));
    inputSegments.emplace_back(make_pair(c,d));
    inputSegments.emplace_back(make_pair(d,a));
  }
  // Add all the segments for computation of the dealunay graph
  AddSegments(inputSegments);
}


/*--------------------------- Modifiers ------------------------------*/

void
MedialAxis2D::
AddSegments(vector<pair<Point3d,Point3d>>& _s)	{
  vector<pair<Point2,Point2>> input;
  m_boundary[0] = _s[0].first;
  m_boundary[1] = _s[0].first;
  // store each segment as pair of CGAL 2d points
  for(auto it = _s.begin(); it != _s.end(); it++)	{
    input.emplace_back(make_pair(Point2(it->first[0],it->first[1]),
          Point2(it->second[0],it->second[1])));
    m_boundary[0][0] = min(min(it->first[0],it->second[0]),m_boundary[0][0]);
    m_boundary[0][1] = min(min(it->first[1],it->second[1]),m_boundary[0][1]);
    m_boundary[1][0] = max(max(it->first[0],it->second[0]),m_boundary[1][0]);
    m_boundary[1][1] = max(max(it->first[1],it->second[1]),m_boundary[1][1]);
  }
  // Compute the graph through segments
  sdg.insert_segments(input.begin(), input.end());
}

void
MedialAxis2D::
AddPoints(vector<Point3d>& _p)	{
  vector<Point2> input;
  m_boundary[0] = _p[0];
  m_boundary[1] = _p[0];
  // store each point as CGAL points
  for(auto it = _p.begin(); it != _p.end(); it++) {
    input.emplace_back(Point2((*it)[0],(*it)[1]));
    m_boundary[0][0] = min((*it)[0],m_boundary[0][0]);
    m_boundary[0][1] = min((*it)[1],m_boundary[0][1]);
    m_boundary[1][0] = max((*it)[0],m_boundary[1][0]);
    m_boundary[1][1] = max((*it)[1],m_boundary[1][1]);
  }
  // Compute the graph through points
  sdg.insert_points(input.begin(), input.end());
}

void
MedialAxis2D::
AddSegments(vector<Point3d>& _p, vector<pair<size_t,size_t>>& _s) {
  vector<Point2> input;
  m_boundary[0] = _p[0];
  m_boundary[1] = _p[0];
  // store each point as CGAL points
  for(auto it = _p.begin(); it != _p.end(); it++) {
    input.emplace_back(Point2((*it)[0],(*it)[1]));
    m_boundary[0][0] = min((*it)[0],m_boundary[0][0]);
    m_boundary[0][1] = min((*it)[1],m_boundary[0][1]);
    m_boundary[1][0] = max((*it)[0],m_boundary[1][0]);
    m_boundary[1][1] = max((*it)[1],m_boundary[1][1]);
  }
  // Compute the graph through point index
  sdg.insert_segments(input.begin(), input.end(),_s.begin(), _s.end());
}

MedialAxis2D::Point2
MedialAxis2D::
MidPoint(Site2& o, Site2& p, Site2& q, Point2& _fp) {
  vector<Point2> points;
  vector<Segment2> seg;
  double x,y;
  Point2 midPoint;
  if(o.is_segment())
    seg.push_back(o.segment());
  else
    points.push_back(o.point());

  if(p.is_segment())
    seg.push_back(p.segment());
  else
    points.push_back(p.point());

  if(q.is_segment())
    seg.push_back(q.segment());
  else
    points.push_back(q.point());
  if(points.size() == 1) {
    for(auto s : seg)
	points.emplace_back(s.supporting_line().projection(points[0]));
    if(collinear(points[0],points[1],points[2]))
      midPoint = points[0];
    else
      midPoint = circumcenter(points[0],points[1],points[2]);
  }
  else {
    x = 0.25*(points[0].x() + points[1].x() + seg[0].source().x()
        + seg[0].target().x());
    y = 0.25*(points[0].y() + points[1].y() + seg[0].source().y()
        + seg[0].target().y());
    midPoint = Point2(x,y);
  }

  x = _fp[0];
  y = _fp[1];
  if(_fp.x() < m_boundary[0][0] || _fp.x() > m_boundary[1][0])
    x = midPoint.x();
  if(_fp.y() < m_boundary[0][1] || _fp.y() > m_boundary[1][1])
    y = midPoint.y();
  _fp = Point2(x,y);
  return _fp;
}

void
MedialAxis2D::
BuildMedialAxis() {
  typedef typename SDG2::Face_handle Face2;
  struct PointCompare {
    bool operator() (const Point2& _p1, const Point2& _p2) const {
      // This is to check if the points are nearly equal
      if((_p1-_p2).squared_length() < numeric_limits<double>::epsilon())
        return false;
      return _p1 < _p2;
     }
  };
  // Map stores already inserted vertices in the graph
  map<Point2, typename MedialAxisGraph::vertex_descriptor,
    PointCompare> faceToVertexMap;

  // Build the segment tree for fast queries
  m_segTree.BuildSegmentTrees();
  double step = 0.05 * min(fabs(m_boundary[1][0]-m_boundary[0][0]),
      fabs(m_boundary[1][1]-m_boundary[0][1]));

  // Create medial axis edge for each finite edge
  for (auto eit = sdg.finite_edges_begin(); eit != sdg.finite_edges_end();
      ++eit) {
    if(IsMedialAxisEdge(*eit)) {
      Segment2 s;
      Parabola2 ps;
      // The witness sites for the edge
      Site2 p = eit->first->vertex(sdg.cw(eit->second))->site();
      Site2 q = eit->first->vertex(sdg.ccw(eit->second))->site();
      // End points of the segments
      // Get the end points of the segment as circumcenters of the triangles
      // neighboring the dual delaunay graph edge
      Point2 fps[2];
      fps[0] = sdg.circumcenter(eit->first);
      fps[1] = sdg.circumcenter(eit->first->neighbor(eit->second));

      // For outside points, fix their co-ordinates
      bool outside = false;
      if(IsOutside(fps[0])) {
        outside = true;
        Site2 o = eit->first->vertex(eit->second)->site();
        fps[0] = MidPoint(o,p,q,fps[0]);
      }
      if(IsOutside(fps[1])) {
        outside = true;
        Site2 o = sdg.tds().mirror_vertex(eit->first, eit->second)->site();
        fps[1] = MidPoint(o,p,q,fps[1]);
      }

      // Removing self loops
      if((fps[0] - fps[1]).squared_length() <
          numeric_limits<double>::epsilon())
        continue;

      // Get the primary object representing the edge
      auto o = sdg.primal(*eit);
      MedialEdge* me = nullptr;
      // If either of the points was outside
      if(outside) {
        if(!IsReflexBisector(fps[0],fps[1], p, q))
          me = new MedialEdge(fps[0],fps[1],p,q);
      }
      // If the edge is a line segment
      else if(CGAL::assign(s, o))
        me = new MedialEdge(s,p,q,step);
      // If the edge is a parabola segment
      else if(CGAL::assign(ps, o))
        me = new MedialEdge(ps,fps[0],fps[1],p,q);

      // Add the edge in the graph if it is a line segment or
      // a parabola segment
      if(me != nullptr)	{
        MedialAxisGraph::vertex_descriptor eps[2];
        for(size_t i = 0; i < 2; ++i) {
          auto findPt = faceToVertexMap.find(fps[i]);
          // If end point not already in the graph - add vertex
          if(findPt == faceToVertexMap.end()) {
            eps[i] = m_graph.add_vertex(fps[i]);
            faceToVertexMap.insert(make_pair(fps[i],eps[i]));
          }
          else
            eps[i] = findPt->second;
        }

        // Mark edge as belonging to obstacle space or free space
        // if both end points are in obstacle - in obstacle
        me->m_isFree = (IsFree(me->GetSource()) || IsFree(me->GetTarget()));
        ValidateEdge(*me);
        //Add edge
        m_graph.add_edge(eps[0], eps[1], *me);
      }
    }
  }
  CalculateClearance();
}

/*--------------------------- Accessors ------------------------------*/

tuple<WorkspaceSkeleton,MedialAxis2D::VertexClearanceMapType,
MedialAxis2D::EdgeClearanceMapType>
MedialAxis2D::
GetSkeleton(size_t _t) {
  typedef WorkspaceSkeleton::GraphType Graph;
  Graph g;
  VertexClearanceMapType vertexMap;
  EdgeClearanceMapType edgeMap;
  // Copy vertices.
  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit) {
    auto p = vit->property();
    g.add_vertex(vit->descriptor(), Point3d(p.x(),p.y(),0));
  }
  // Copy edges.
  for(auto eit = m_graph.edges_begin(); eit != m_graph.edges_end();
      ++eit) {
    vector<Point3d> edge;
    if(_t == 1 && !eit->property().m_isFree) continue;
    else if(_t == 2 && eit->property().m_isFree) continue;
    for(auto it = eit->property().m_interpolated.begin();
      it != eit->property().m_interpolated.end(); ++it)
      edge.emplace_back(Point3d(it->x(),it->y(),0));
    auto ed = g.add_edge(eit->descriptor(), edge);
    // Put the clearance info as vector of clearance info -
    // clearance and witness
    edgeMap.insert(make_pair(ed,AnnotateSegment(eit->property())));
    // Add the bidirectional edge for the opposite direction
    std::reverse(edge.begin(), edge.end());
    // Need to add it this way so that the edge ID matches.
    add_internal_edge(g, reverse(ed), edge);
    //TODO:: figure out if this needs to be added to the edge map or not.
  }

  // Delete isolated vertices for free space or obstacle space
  // medial axis graph
  if(_t > 0) {
    vector<size_t> deletedVertices;
    for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit) {
      if(g.get_out_degree(vit->descriptor())
          + g.get_in_degree(vit->descriptor()) == 0)
        deletedVertices.emplace_back(vit->descriptor());
    }
    for(auto v : deletedVertices)
      g.delete_vertex(v);
  }

  // Set the vertex clearance map
  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit)
    vertexMap.insert(make_pair(vit->descriptor(),
          m_vertexClearance[vit->descriptor()]));

  WorkspaceSkeleton skeleton;
  skeleton.SetGraph(g);
  return make_tuple(skeleton, vertexMap, edgeMap);
}

vector<MedialAxis2D::ClearanceType>
MedialAxis2D::
AnnotateSegment(MedialEdge& _e)	{
  vector<ClearanceType> clearances;
  // Store the clearance of the edge as the clearance of
  // each interpolated point
  for(auto it = _e.m_interpolated.begin(); it != _e.m_interpolated.end();
      ++it)
    clearances.emplace_back(GetMinDistance(*it,_e.m_site1,_e.m_site2));
  return clearances;
}

double
MedialAxis2D::
GetVertexClearance(size_t _i) {
  auto it = m_vertexClearance.find(_i);
  if(it == m_vertexClearance.end())
    return 0;
  else
    return  it->second.first;
}

Point3d
MedialAxis2D::
GetVertexClearanceWitness(size_t _i)	{
  auto it = m_vertexClearance.find(_i);
  if(it == m_vertexClearance.end())
    return Point3d(0,0,0);
  else
    return it->second.second;
}


/*--------------------------- Helpers ------------------------------*/


bool
MedialAxis2D::
IsOutside(const Point2& _p) {
  return ( _p.x() < m_boundary[0][0] || _p.x() > m_boundary[1][0] ||
      _p.y() < m_boundary[0][1] || _p.y() > m_boundary[1][1] );
}

void
MedialAxis2D::
ValidateEdge(MedialEdge& _e) {
  int num = _e.m_interpolated.size();
  for(int i = 0; i < num; i++)
    if(IsOutside(_e.m_interpolated[i])) {
      _e.m_interpolated.erase(_e.m_interpolated.begin()+i);
      i--;
    }
}

void
MedialAxis2D::
AddPolygon(const GMSPolyhedron& _p,vector<pair<Point3d,Point3d>>& _s,
    Boundary* _b) {
// @TODO: Change the code to call the surface utility of GMSPolyhderon
// after generalizing it
// Create function for determining if a polygon is near the XY surface plane
// or on plane in the positive side of z axis
  static auto NearOrthogonalPlane = [&](const GMSPolygon& _poly,
      size_t _pi = 2) -> bool {
    const double tolerance = 0; // Tolerance for considering points near-plane.
    return _poly.GetPoint(0)[_pi] >= tolerance
      && _poly.GetPoint(1)[_pi] >= tolerance
      && _poly.GetPoint(2)[_pi] >= tolerance;
  };

  // Get all of the edges from every triangle.
  multimap<pair<int, int>,pair<Point3d,Point3d>> lines;
  auto polygonList = _p.GetPolygonList();
  for(const auto& tri : polygonList) {
    if(!NearOrthogonalPlane(tri))
      continue;

    for(unsigned short i = 0; i < 3; ++i) {
      // Always put the lower vertex index first to make finding duplicates
      // easier.
      const int& a = tri[i];
      const int& b = tri[(i + 1) % 3];
      lines.emplace(make_pair(min(a, b), max(a, b)),
          make_pair(tri.GetPoint(i),tri.GetPoint((i+1)%3)));
    }
  }

  // Store the edges that occurred exactly once as the boundary lines.
  for(auto iter = lines.begin(), next = ++lines.begin();
    iter != lines.end() && next != lines.end(); ++iter, ++next) {
    if(next == iter)
      --iter;
    while(next != lines.end() && iter->first == next->first)
      ++next;
    if(distance(iter,next) > 1) {
      iter = lines.erase(iter, next);
      next = iter;
      if(iter != lines.begin())
        --iter;
    }
  }

  // Add the segments
  PolygonSegments polygon;
  for(auto it = lines.begin(); it!= lines.end(); ++it) {
    _s.emplace_back(it->second);
    Point2 src(it->second.first[0],it->second.first[1]);
    Point2 trgt(it->second.second[0],it->second.second[1]);
    polygon.emplace_back(PolygonSegment(src,trgt));
  }
  m_polygons.emplace_back(polygon);

  // Add the boundary in segment tree
  // Find the boundary if not provided
  if(_b)
    m_segTree.AddBoundary(_b);
  else {
    double minx, miny, maxx, maxy;
    minx = maxx = polygon[0].m_end[0][0];
    miny = maxy = polygon[0].m_end[0][1];
    for(auto it = polygon.begin(); it!= polygon.end(); ++it) {
      for(size_t i = 0; i < 2; i++) {
        auto v = it->m_end[i];
	minx = min(minx, v.x());
	maxx = max(maxx, v.x());
	miny = min(miny, v.y());
	maxy = max(maxy, v.y());
      }
    }
    auto b = new WorkspaceBoundingBox(2);
    b->SetRange(0, minx, maxx);
    b->SetRange(1, miny, maxy);
    m_segTree.AddBoundary(b);
  }
}

MedialAxis2D::ClearanceType
MedialAxis2D::
DistanceToSite(const Point2& _p, Site2& _s) {
  Point2 witness;
  // If the witness site is a point, the point site is the witness point
  if(_s.is_point()) {
    witness = _s.point();
  }
  // If the witness site is a segment, the projection of the point on
  // the segment site is the witness point
  else if(_s.is_segment()) {
    auto s = _s.segment();
    witness = s.supporting_line().projection(_p);
    if(!s.has_on(witness)) {
      if((s.source()-_p).squared_length() < (s.target()-_p).squared_length())
        witness = s.source();
      else
        witness = s.target();
    }
  }
  else
    witness = _p;
  return make_pair(sqrt((witness-_p).squared_length()),
      Point3d(witness.x(), witness.y(), 0));
}

MedialAxis2D::ClearanceType
MedialAxis2D::
GetMinDistance(const Point2& _p, Site2& _s1, Site2& _s2) {
  // Return the minimum of the distance to the sites
  auto c1 = DistanceToSite(_p,_s1);
  auto c2 = DistanceToSite(_p,_s2);
  if(c1.first < c2.first)
    return c1;
  else
    return c2;
}

double
MedialAxis2D::
GetMinDistance(MedialEdge& _e) {
  double minD = numeric_limits<double>::max();

  for(auto it = _e.m_interpolated.begin(); it != _e.m_interpolated.end();
      it++)
    minD = min(minD,GetMinDistance(*it,_e.m_site1,_e.m_site2).first);
  return minD;
}

double
MedialAxis2D::
GetMaxDistance(MedialEdge& _e) {
  double maxD = 0;
  for(auto it = _e.m_interpolated.begin(); it != _e.m_interpolated.end(); it++)
    maxD = max(maxD,GetMinDistance(*it,_e.m_site1,_e.m_site2).first);
  return maxD;
}

bool
MedialAxis2D::
IsSkeletonEdge(Edge2& _e) {
  // Function to check whether the point is end point of the segment
  static auto IsSegmentEndPoint =
    [&](const Site2& _p, const Site2& _s) -> bool {
    return (sdg.geom_traits().equal_2_object()(_p, _s.source_site())||
        sdg.geom_traits().equal_2_object()(_p, _s.target_site()));
  };

  Site2 p = _e.first->vertex(sdg.cw(_e.second))->site();
  Site2 q = _e.first->vertex(sdg.ccw(_e.second))->site();

  // if one site is the end point of the other site then it is a bisector
  if(p.is_segment() && q.is_point() && IsSegmentEndPoint(q,p))
    return false;
  else if(q.is_segment() && p.is_point() && IsSegmentEndPoint(p,q))
    return false;
  return true;
}

bool
MedialAxis2D::
IsMedialAxisEdge(Edge2& _e, bool k) {
  // Remove bisector edges
  if(!IsSkeletonEdge(_e)) return false;
  // Get the witness sites
  Site2 p = _e.first->vertex(sdg.cw(_e.second))->site();
  Site2 q = _e.first->vertex(sdg.ccw(_e.second))->site();

  Segment2 s;
  Parabola2 ps;
  Gt::Line_2 l;
  Gt::Ray_2 r;

  auto o = sdg.primal(_e);
  if(CGAL::assign(l, o)) {
    cout<<"Line"<<endl;
    return false;
  }
  // Check whether it is line segment
  else if(CGAL::assign(s, o)) {
    return (!IsReflexBisector(s.source(), s.target(), p, q));
  }
  else if(CGAL::assign(r, o)) {
    cout<<"Ray "<<r.source().x()<<" "<<r.source().y()<<endl;
    return false;
  }
  // Check whether it is parabola segment
  else if(CGAL::assign(ps, o)) {
    return true;
  }
  else {
    cout<<"Other"<<endl;
    return false;
  }
}

bool
MedialAxis2D::
IsReflexBisector(const Point2& _p1, const Point2& _p2,
    Site2& _s1, Site2& _s2) {
  // Function for getting the angle between the line segment site and
  // bisector edge
  static auto GetAngle = [&](Site2& _s, const Point2& _pt1,
      const Point2& _pt2) -> double {
    Vector2d v1(_s.segment().to_vector().x(), _s.segment().to_vector().y());
    Vector2d v2((_p2.x()-_p1.x()), (_p2.y()-_p1.y()));
    v1 = v1.normalize();
    v2 = v2.normalize();
    return acos(abs(v1*v2))*180/PI;
  };

  // Check whether the segment has positive minimum disc radius
  if(GetMinDistance(_p1, _s1, _s2).first > numeric_limits<float>::epsilon()
      && GetMinDistance(_p2, _s1, _s2).first > numeric_limits<float>::epsilon())
    return false;

  double angle1 = (_s1.is_segment())? GetAngle(_s1, _p1, _p2) : -1;
  double angle2 = (_s2.is_segment())? GetAngle(_s2, _p1, _p2) : -1;
  // Check if bisector is reflex edge
  if(angle1 >= 0 && angle2 >= 0	&& (angle1 + angle2) > 175)
    return true;
  else if(angle1 >= 0 && (angle1 > 85 || angle1 <= 5))
    return true;
  else if(angle2 >= 0 && (angle2 > 85 || angle2 <= 5))
    return true;
  else
    return false;
}

void
MedialAxis2D::
CalculateClearance() {
  // Calculate the clearance for each vertex
  for(auto vit = m_graph.begin(); vit != m_graph.end(); ++vit)	{
    auto p = vit->property();
    ClearanceType cl;
    // Find the clearance with respect to each adjacent edge and find the
    // minimum among them
    for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
      auto clrnce = GetMinDistance(p, eit->property().m_site1,
          eit->property().m_site2);
      if(eit == vit->begin() || clrnce.first < cl.first)
        cl = clrnce;
    }
    m_vertexClearance.insert(make_pair(vit->descriptor(),cl));
  }
}

bool
MedialAxis2D::
IsFree(const Point2& _p) {
  // Function for point in a polygon test
  static auto PointInPolygon = [&](const Point2& _p,
      PolygonSegments& _v) -> bool {
    bool  intersecting = false;
    for (size_t i=0; i<_v.size(); i++) {
      if(_v[i].OnLine(_p)) // if on boundary, it is not free
        return true;

      if ((_v[i].Target().y() < _p.y() && _v[i].Source().y() >= _p.y())
          ||( _v[i].Source().y() < _p.y() && _v[i].Target().y() >= _p.y()))
        intersecting^=(_p.y()*_v[i].GetMultiple()+_v[i].GetConstant()<_p.x());
    }
    return intersecting;
  };

  // Find the enclosing intervals
  auto num = m_segTree.FindEnclosingBoundaries(Point3d(_p.x(),_p.y(),0));
  // Cant find enclosing interval - lies in free space
  if(num == 0)
    return true;
  else {
    // If it lies in any of the polygon - lies in obstacle space
    for(size_t i = 0; i<num ; ++i) {
      size_t modelIndex = m_segTree.GetOutputBoundaryIndex(i);
      if(PointInPolygon(_p,m_polygons[modelIndex]))
        return false;
    }
    return true;
  }
}
