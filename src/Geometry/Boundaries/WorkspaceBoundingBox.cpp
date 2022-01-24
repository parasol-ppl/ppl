#include "WorkspaceBoundingBox.h"

#include "Geometry/GMSPolyhedron.h"


/*------------------------------ Construction --------------------------------*/

WorkspaceBoundingBox::
WorkspaceBoundingBox(const size_t _n) : AbstractBoundingBox(_n) { }


WorkspaceBoundingBox::
WorkspaceBoundingBox(const std::vector<double>& _center) :
    AbstractBoundingBox(_center) { }


WorkspaceBoundingBox::
WorkspaceBoundingBox(XMLNode& _node) : AbstractBoundingBox(_node) { }


WorkspaceBoundingBox::
~WorkspaceBoundingBox() noexcept = default;


std::unique_ptr<Boundary>
WorkspaceBoundingBox::
Clone() const {
  return std::unique_ptr<WorkspaceBoundingBox>(new WorkspaceBoundingBox(*this));
}

/*--------------------------- Property Accessors -----------------------------*/

Boundary::Space
WorkspaceBoundingBox::
Type() const noexcept {
  return Boundary::Space::Workspace;
}


std::string
WorkspaceBoundingBox::
Name() const noexcept {
  return "WorkspaceBoundingBox";
}

/*--------------------------- Containment Testing ----------------------------*/

bool
WorkspaceBoundingBox::
InBoundary(const Cfg& _c) const {
  return Boundary::InWorkspace(_c);
}

/*-------------------------- CGAL Representations ----------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// Builder object for making CGAL representations of 2d bounding boxes.
////////////////////////////////////////////////////////////////////////////////
struct builder2d
  : public CGAL::Modifier_base<Boundary::CGALPolyhedron::HalfedgeDS>
{

  typedef Boundary::CGALPolyhedron CGALPolyhedron;

  const std::vector<Range<double>>& m_bbx;

  builder2d(const std::vector<Range<double>>& _bbx) : m_bbx(_bbx) {}

  void operator()(CGALPolyhedron::HalfedgeDS& _h) {
    using Point = CGALPolyhedron::HalfedgeDS::Vertex::Point;
    CGAL::Polyhedron_incremental_builder_3<CGALPolyhedron::HalfedgeDS> b(_h);

    b.begin_surface(4, 2, 8);

    b.add_vertex(Point(m_bbx[0].min, m_bbx[1].min, 0));
    b.add_vertex(Point(m_bbx[0].max, m_bbx[1].min, 0));
    b.add_vertex(Point(m_bbx[0].max, m_bbx[1].max, 0));
    b.add_vertex(Point(m_bbx[0].min, m_bbx[1].max, 0));

    // The bounding box has two facets: one on top, and one below. This is
    // needed to create a closed surface as this representation is three-
    // dimensional.
    b.begin_facet();
    b.add_vertex_to_facet(0);
    b.add_vertex_to_facet(1);
    b.add_vertex_to_facet(2);
    b.add_vertex_to_facet(3);
    b.end_facet();

    b.begin_facet();
    b.add_vertex_to_facet(0);
    b.add_vertex_to_facet(3);
    b.add_vertex_to_facet(2);
    b.add_vertex_to_facet(1);
    b.end_facet();

    b.end_surface();
  }
};


////////////////////////////////////////////////////////////////////////////////
/// Builder object for making CGAL representations of a 3d bounding box.
////////////////////////////////////////////////////////////////////////////////
struct builder3d
  : public CGAL::Modifier_base<Boundary::CGALPolyhedron::HalfedgeDS>
{

  typedef Boundary::CGALPolyhedron CGALPolyhedron;

  const std::vector<Range<double>>& m_bbx;

  builder3d(const std::vector<Range<double>>& _bbx) : m_bbx(_bbx) {}

  void operator()(CGALPolyhedron::HalfedgeDS& _h) {
    using Point = CGALPolyhedron::HalfedgeDS::Vertex::Point;
    CGAL::Polyhedron_incremental_builder_3<CGALPolyhedron::HalfedgeDS> b(_h);

    b.begin_surface(8, 6, 24);

    b.add_vertex(Point(m_bbx[0].min, m_bbx[1].min, m_bbx[2].min));
    b.add_vertex(Point(m_bbx[0].max, m_bbx[1].min, m_bbx[2].min));
    b.add_vertex(Point(m_bbx[0].max, m_bbx[1].max, m_bbx[2].min));
    b.add_vertex(Point(m_bbx[0].min, m_bbx[1].max, m_bbx[2].min));
    b.add_vertex(Point(m_bbx[0].min, m_bbx[1].min, m_bbx[2].max));
    b.add_vertex(Point(m_bbx[0].max, m_bbx[1].min, m_bbx[2].max));
    b.add_vertex(Point(m_bbx[0].max, m_bbx[1].max, m_bbx[2].max));
    b.add_vertex(Point(m_bbx[0].min, m_bbx[1].max, m_bbx[2].max));

    // Front
    b.begin_facet();
    b.add_vertex_to_facet(0);
    b.add_vertex_to_facet(1);
    b.add_vertex_to_facet(2);
    b.add_vertex_to_facet(3);
    b.end_facet();

    // Right
    b.begin_facet();
    b.add_vertex_to_facet(1);
    b.add_vertex_to_facet(5);
    b.add_vertex_to_facet(6);
    b.add_vertex_to_facet(2);
    b.end_facet();

    // Back
    b.begin_facet();
    b.add_vertex_to_facet(5);
    b.add_vertex_to_facet(4);
    b.add_vertex_to_facet(7);
    b.add_vertex_to_facet(6);
    b.end_facet();

    // Left
    b.begin_facet();
    b.add_vertex_to_facet(4);
    b.add_vertex_to_facet(0);
    b.add_vertex_to_facet(3);
    b.add_vertex_to_facet(7);
    b.end_facet();

    // Top
    b.begin_facet();
    b.add_vertex_to_facet(3);
    b.add_vertex_to_facet(2);
    b.add_vertex_to_facet(6);
    b.add_vertex_to_facet(7);
    b.end_facet();

    // Bottom
    b.begin_facet();
    b.add_vertex_to_facet(4);
    b.add_vertex_to_facet(5);
    b.add_vertex_to_facet(1);
    b.add_vertex_to_facet(0);
    b.end_facet();

    b.end_surface();
  }
};


Boundary::CGALPolyhedron
WorkspaceBoundingBox::
CGAL() const {
  CGALPolyhedron cp;

  // Choose builder based on dimension.
  switch(NBox::GetDimension()) {
    case 2:
      {
        builder2d b(NBox::GetRanges());
        cp.delegate(b);
        break;
      }
    case 3:
      {
        builder3d b(NBox::GetRanges());
        cp.delegate(b);
        break;
      }
    default:
      throw RunTimeException(WHERE, "Workspace bounding boxes must have "
          "dimension 2 or 3.");
  }

  // Ensure result is valid.
  if(!cp.is_valid())
    throw RunTimeException(WHERE, "WorkspaceBoundingBox:: Invalid CGAL "
        "polyhedron created!");
  return cp;
}


GMSPolyhedron
WorkspaceBoundingBox::
MakePolyhedron() const {
  // We will use a short 3d box to approximate a 2d box boundary. Define its
  // height here.
  const double height = .02;
  Range<double> x, y, z(-height,height);

  switch(NBox::GetDimension())
  {
    case 3:
      z = GetRange(2);
    case 2:
      x = GetRange(0);
      y = GetRange(1);
      break;
    default:
      throw RunTimeException(WHERE) << "GMSPolyhedron doesn't make sense for "
                                    << NBox::GetDimension() << "-d boundaries.";
  }

  GMSPolyhedron poly = GMSPolyhedron::MakeBox(x, y, z);
  poly.Invert();
  return poly;
}

/*----------------------------------------------------------------------------*/
