#include "TetGenDecomposition.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"
#include "Workspace/WorkspaceDecomposition.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#define TETLIBRARY
#undef PI
#include "tetgen.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/IO/io.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/IO/print_wavefront.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_3/SNC_indexed_items.h>

using CGALKernel    = CGAL::Exact_predicates_exact_constructions_kernel;
using NefPolyhedron = CGAL::Nef_polyhedron_3<CGALKernel>;
using Polyhedron    = CGAL::Polyhedron_3<CGALKernel>;

using namespace std;


/*---------------------------- Local Functions -------------------------------*/
// These are implemented here to avoid including CGAL nef poly stuff in the
// header file. This is needed because the nef poly stuff requires us to
// undefine PI, and there is no way to sand-box that inside of a header.
// Instead, every function that has a CGAL nef poly as part of its object is
// implemented as a 'manual external member' in this section.


/// Get the index of a point in the freespace nef poly.
/// @param _polyhedron The nef polyhedron of interest.
/// @param _p The point to locate.
/// @return The index of _p in _polyhedron's vertex list. If it is not found,
///         an exception will be thrown.
size_t
PointIndex(const NefPolyhedron& _polyhedron,
    const NefPolyhedron::Point_3& _p) {
  size_t index = 0;
  auto it = _polyhedron.vertices_begin();
  for(; it != _polyhedron.vertices_end(); ++it, ++index)
    if(_p == it->point())
      break;
  if(it == _polyhedron.vertices_end())
    throw RunTimeException(WHERE, "Point not found.");
  return index;
}


/// Add the vertices to the free model.
/// @param _freeModel The tetgen model under construction.
/// @param _freespace The free space polyhedra.
/// @param _debug Show debug messages?
void
AddVertices(tetgenio* const _freeModel, const NefPolyhedron& _freespace,
    const bool _debug) {
  if(_debug)
    cout << "Adding " << _freespace.number_of_vertices() << " vertices..."
         << endl;

  _freeModel->numberofpoints = _freespace.number_of_vertices();
  _freeModel->pointlist = new double[_freeModel->numberofpoints * 3];

  size_t n = 0;
  for(auto v = _freespace.vertices_begin(); v != _freespace.vertices_end(); ++v) {
    using CGAL::to_double;
    _freeModel->pointlist[3 * n + 0] = to_double(v->point()[0]);
    _freeModel->pointlist[3 * n + 1] = to_double(v->point()[1]);
    _freeModel->pointlist[3 * n + 2] = to_double(v->point()[2]);
    ++n;
  }
}


/// Extract facet info from the freespace nef poly.
/// @param _freespace The freespace nef poly.
/// @return A set of facet descriptors.
vector<vector<vector<size_t>>>
ExtractFacets(const NefPolyhedron& _freespace) {
  using SHalfloop_const_handle = NefPolyhedron::SHalfloop_const_handle;
  using SHalfedge_const_handle = NefPolyhedron::SHalfedge_const_handle;
  using SHalfedge_around_facet_const_circulator =
      NefPolyhedron::SHalfedge_around_facet_const_circulator;

  typedef vector<size_t> Polygon;
  typedef vector<Polygon> Facet;

  vector<Facet> facets;

  // NefPolyhedron represents each facet as two half-facets. Each has one
  // outward-facing half-facet and one inward-facing twin. Start by iterating
  // over all half-facets.
  for(auto hf = _freespace.halffacets_begin(); hf != _freespace.halffacets_end();
      ++hf) {

    // Assume that our objects are well-formed and skip inward-facing twins.
    if(hf->is_twin())
      continue;

    Facet f;

    // Each facet is composed of one or more polygons. Each polygon is described
    // by a 'half-facet cycle'. Extract the points from these cycles to describe
    // the facet's polygons.
    for(auto it = hf->facet_cycles_begin(); it != hf->facet_cycles_end(); ++it) {
      Polygon poly;

      // Each cycle represents either a single, isolated vertex or a set of
      // vertices connected by edges.
      if(it.is_shalfloop()) {
        // This is an isolated vertex.
        SHalfloop_const_handle sl(it);
        if(sl == 0)
          throw RunTimeException(WHERE, "TetGenDecomposition error: could not "
              "get shalfloop handle while adding facets to freespace model.");

        const auto& point = sl->incident_sface()->center_vertex()->point();
        poly.push_back(PointIndex(_freespace, point));
      }
      else {
        // This is a set of vertices joined by edges.
        SHalfedge_const_handle se(it);
        if(se == 0)
          throw RunTimeException(WHERE, "TetGenDecomposition error: could not "
              "get shalfedge handle while adding facets to freespace model.");

        SHalfedge_around_facet_const_circulator hc_start(se);
        SHalfedge_around_facet_const_circulator hc_end(hc_start);
        CGAL_For_all(hc_start, hc_end) {
          const auto& point = hc_start->source()->center_vertex()->point();
          poly.push_back(PointIndex(_freespace, point));
        }
      }

      // Add the polygon to the current facet.
      f.push_back(poly);
    }

    // Add the facet to the facet list.
    facets.push_back(f);
  }

  return facets;
}


/// Add the facets to the free model.
/// @param _freeModel The tetgen model under construction.
/// @param _freespace The free space polyhedra.
/// @param _debug Show debug messages?
void
AddFacets(tetgenio* const _freeModel, const NefPolyhedron& _freespace,
    const bool _debug) {
  auto facets = ExtractFacets(_freespace);

  if(_debug)
    cout << "Adding " << facets.size() << " facets..." << endl;

  _freeModel->numberoffacets = facets.size();
  _freeModel->facetlist = new tetgenio::facet[_freeModel->numberoffacets];
  _freeModel->facetmarkerlist = new int[_freeModel->numberoffacets];

  for(size_t fn = 0; fn < facets.size(); ++fn) {
    const auto& facetData = facets[fn];
    tetgenio::facet* f = &_freeModel->facetlist[fn];

    // Add this facet to the free model.
    f->numberofpolygons = facetData.size();
    f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
    f->numberofholes = 0;
    f->holelist = nullptr;

    _freeModel->facetmarkerlist[fn] = 1;

    for(size_t pn = 0; pn < facetData.size(); ++pn) {
      const auto& polygonData = facetData[pn];

      // Add this polygon to the facet
      tetgenio::polygon* p = &f->polygonlist[pn];
      p->numberofvertices = polygonData.size();
      p->vertexlist = new int[p->numberofvertices];

      for(size_t vn = 0; vn < polygonData.size(); ++vn) {
        const auto& vertex = polygonData[vn];

        // Add this vertex to the polygon.
        p->vertexlist[vn] = vertex;
      }
    }
  }
}


/// Add holes to the free model for each obstacle.
/// @param _freeModel The tetgen model under construction.
/// @param _freespace The free space polyhedra.
/// @param _env The environment object.
/// @param _debug Show debug messages?
void
AddHoles(tetgenio* const _freeModel, const NefPolyhedron& _freespace,
    const Environment* const _env, const bool _debug) {
  if(_debug)
    cout << "Adding holes..." << endl;

  // Set up to check for obstacle/boundary collisions.
  static PQPSolid pqpSolid;
  CDInfo cdInfo;
  mathtool::Transformation identity;
  const GMSPolyhedron boundaryPoly = _env->GetBoundary()->MakePolyhedron();

  vector<Vector3d> holes;
  for(size_t i = _env->UsingBoundaryObstacle(); i < _env->NumObstacles(); ++i) {
    // Skip internal obstacles.
    MultiBody* obst = _env->GetObstacle(i);
    if(obst->IsInternal())
      continue;

    if(_debug)
      std::cout << "\tAdding " << obst->GetNumBodies() << " holes for obstacle "
                << i << "."
                << std::endl;

    // Create each hole with a point just beneath one of the model's facets.
    for(size_t j = 0; j < obst->GetNumBodies(); ++j) {
      const Body* const body = obst->GetBody(j);
      const GMSPolyhedron& poly = body->GetWorldPolyhedron();
      holes.emplace_back(poly.GetInsidePoint());

      if(_debug) {
        std::cout << "\t\tAdded hole at " << holes.back() << "." << std::endl;

        // Check that the hole point is inside the poly.
        const bool goodHole = pqpSolid.IsInsideObstacle(holes.back(), poly, identity);
        std::cout << "\t\tComputed hole is " << (goodHole ? "" : "not " )
                  << "in the obstacle!"
                  << std::endl;

        /// @todo There seems to be an intermittent problem with adding holes for
        ///       obstacles that touch the environment boundary. Sometimes it
        ///       causes bad decompositions where the free space gets eaten away,
        ///       and sometimes not. Not adding the hole sometimes causes tetgen
        ///       to not remove the tetras within the obstacle. We need to
        ///       characterize the cases where it fails and where it works.
        const bool touching = pqpSolid.IsInCollision(poly, identity,
                                                     boundaryPoly, identity,
                                                     cdInfo);
        if(touching)
          std::cout << "\t\tBody " << j << " touches boundary, might cause "
                    << "problems!"
                    << std::endl;
      }
    }
  }

  // Add a hole to the tetgen structure for each obstacle. Use a point just
  // beneath one of the model's facets.
  _freeModel->numberofholes = holes.size();
  _freeModel->holelist = new double[_freeModel->numberofholes * 3];

  size_t num = 0;
  for(const auto& hole : holes) {
    _freeModel->holelist[3 * num + 0] = hole[0];
    _freeModel->holelist[3 * num + 1] = hole[1];
    _freeModel->holelist[3 * num + 2] = hole[2];
    ++num;
  }
}



void
OutputNefPolyhedron(const NefPolyhedron& _p, const string _filename) {
  ofstream objFile(_filename);
  Polyhedron output;
  _p.convert_to_polyhedron(output);
  CGAL::print_polyhedron_wavefront(objFile, output);
  objFile.close();
}

/*------------------------------- Construction -------------------------------*/

TetGenDecomposition::
TetGenDecomposition() = default;


TetGenDecomposition::
TetGenDecomposition(XMLNode& _node) {
  m_switches = _node.Read("switches", false, m_switches,
      "TetGen input parameters. See TetGen manual. Need 'pn' at a minimum. "
      "Use 'q' for quality (finer-grained) tetrahedra. 'Q' for quiet. "
      "'r' for read input.");

  m_baseFilename = GetPathName(_node.Filename())
      + _node.Read("baseFilename", false, m_baseFilename,
      "Specify a file name to read or write a decomposition. There are two "
      "files for a decomposition with .node and .ele extensions. This is the "
      "base name for those files.");

  m_useConvex = _node.Read("useConvex", false, false,
      "Use obstacle convex hull or environment boundary to get "
      "free space decomposition. Note: This option only supports "
      "one obstacle. In case of multiple obstacles, it will pick "
      "the first in the list.");
  m_convexHullScaleFactor = _node.Read("convexScale", false, 1.2, 1.0, MAX_DBL,
      "How much to scale the convex hull by in order to avoid degenerate "
      "decomposition at the intersection of the convex hull and the "
      "obstacle. Note: This option only supports "
      "one obstacle. In case of multiple obstacles, it will pick "
      "the first in the list.");
  std::string ioType = _node.Read("io", false, "none", "The I/O operation to use "
      "(read, write, none).");

  // Parse the string into an enumerated type and check for errors.
  std::transform(ioType.begin(), ioType.end(), ioType.begin(), ::tolower);
  if(ioType == "read")
    m_ioType = Read;
  else if(ioType == "write")
    m_ioType = Write;
  else if(ioType != "none")
    throw ParseException(_node.Where()) << "Unrecognized IO operation '"
                                        << ioType << "'.";

  m_debug = _node.Read("debug", false, m_debug, "Show debugging messages, "
      "and also write the freespace model if we are writing.");

  ValidateSwitches(m_switches);
}

/*----------------------------- Decomposition --------------------------------*/

const WorkspaceDecomposition*
TetGenDecomposition::
operator()(const Environment* _env) {
  const bool readFile  = !m_baseFilename.empty() and m_ioType == Read,
             writeFile = !m_baseFilename.empty() and m_ioType == Write;

  // Allocate tetgen structures.
  m_freeModel = new tetgenio();
  m_decompModel = new tetgenio();

  if(readFile) {
    LoadDecompModel();

    if(m_debug)
      cout << "\tRunning tetgen with switches 'rnQ'..." << endl;

    tetrahedralize(const_cast<char*>("rnQ"), m_decompModel, m_decompModel);
  }
  else {
    if(m_debug)
      cout << "Decomposing environment with tetgen..."
           << endl;

    if(m_useConvex)
      MakeInternalVoidModel(_env);
    else
      MakeFreeModel(_env);

    // We usually don't want to write the free space - only do this in debug
    // mode.
    if(writeFile and m_debug)
      SaveFreeModel();

    if(m_debug)
      std::cout << "\tRunning tetgen with switches '" << m_switches
                << "'..." << std::endl;

    tetrahedralize(const_cast<char*>(m_switches.c_str()),
        m_freeModel, m_decompModel);

    if(writeFile)
      SaveDecompModel();
  }

  if(m_debug)
    cout << "Decomposition complete." << endl;

  auto decomposition = MakeDecomposition();

  if(writeFile and m_debug) {
    ostringstream os;
    decomposition->WriteObj("decomposition.obj");
  }

  // Release tetgen structures.
  delete m_freeModel;
  delete m_decompModel;
  m_freeModel = nullptr;
  m_decompModel = nullptr;

  return decomposition;
}


const WorkspaceDecomposition*
TetGenDecomposition::
MakeDecomposition() {
  // Assert that tetgen actually produced a tetrahedralization and not something
  // else.
  const size_t numCorners = m_decompModel->numberofcorners;
  if(numCorners != 4)
    throw PMPLException("TetGen error", WHERE, "The decomposition is not "
        "tetrahedral. Expected 4 points per tetraherdon, but got " +
        to_string(numCorners) + ".");

  // Make decomposition object.
  WorkspaceDecomposition* decomposition(new WorkspaceDecomposition());

  // Add points.
  const size_t numPoints = m_decompModel->numberofpoints;
  const double* const points = m_decompModel->pointlist;
  for(size_t i = 0; i < numPoints; ++i)
    decomposition->AddPoint(Point3d(&points[3 * i]));

  // Make region for each tetrahedron.
  const size_t numTetras = m_decompModel->numberoftetrahedra;
  const int* const tetra = m_decompModel->tetrahedronlist;
  for(size_t i = 0; i < numTetras; ++i)
    decomposition->AddTetrahedralRegion(&tetra[i * numCorners]);

  // Make edges between adjacent tetrahedra.
  const int* const neighbors = m_decompModel->neighborlist;
  for(size_t i = 0; i < numTetras; ++i) {
    // Each tetrahedron has up to 4 neighbors. Empty slots are marked with an
    // index of -1.
    for(size_t j = 0; j < 4; ++j) {
      size_t neighborIndex = neighbors[4 * i + j];
      if(neighborIndex != size_t(-1))
        decomposition->AddPortal(i, neighborIndex);
    }
  }

  if(m_debug)
    std::cout << "\tNumber of points: " << numPoints << std::endl
              << "\tNumber of tetras: " << numTetras << std::endl;

  decomposition->Finalize();
  return decomposition;
}

/*------------------------ Freespace Model Creation --------------------------*/

/*----------------------------make internal void model--------------------*/

void
TetGenDecomposition::
MakeInternalVoidModel(const Environment* _env) {
  //this function can only handle one obstacle for now. Throw an error if the
  //environment has more
  if(_env->NumObstacles() > 1)
    throw RunTimeException(WHERE, "This option only supports environments with one obstacle.");
  if(m_debug)
    cout << "Creating cavity model..." << endl
         << "\tAdding convex hull..." << endl;

  // Subtract each obstacle from the convex hull.
  MultiBody* obst = _env->GetObstacle(0);
  auto poly = obst->GetBody(0)->GetWorldPolyhedron().ComputeConvexHull();

  //scale the convex hull to avoid degenerate triangles.
  poly.Scale(m_convexHullScaleFactor);

  if(m_debug) {
    cout << "convex hull com: " << poly.GetCentroid() << endl;
    ofstream out("convex.obj");
    poly.WriteObj(out);
  }

  auto cp = poly.CGAL();

  NefPolyhedron freespace(cp);

  if(!obst->IsInternal()) {

    auto ocp = obst->GetBody(0)->GetWorldPolyhedron().CGAL();

    if(m_debug)
      cout << "\t\tobstacle is " << (ocp.is_closed() ? "" : "not ")
           << "closed" << endl;

    freespace -= NefPolyhedron(ocp);

    if(m_debug)
      OutputNefPolyhedron(freespace, "convex_freespace.obj");

    // Add free model to tetgen structure.
    AddVertices(m_freeModel, freespace, m_debug);
    AddFacets(m_freeModel, freespace, m_debug);
    AddHoles(m_freeModel, freespace, _env, m_debug);
  }
}


void
TetGenDecomposition::
MakeFreeModel(const Environment* _env) {
  if(m_debug)
    cout << "Creating free model..." << endl
         << "\tAdding boundary..." << endl;

  // Initialize the freespace model as an outward-facing boundary.
  auto cp = _env->GetBoundary()->CGAL();
  NefPolyhedron freespace(cp);

  // Subtract each obstacle from the freespace.
  for(size_t i = _env->UsingBoundaryObstacle(); i < _env->NumObstacles(); ++i) {
    if(m_debug)
      cout << "\tAdding obstacle " << i << "..." << endl;

    const MultiBody* const obst = _env->GetObstacle(i);
    if(!obst->IsInternal()) {
      for(size_t j = 0; j < obst->GetNumBodies(); ++j) {
        // Make CGAL representation of this obstacle.
        auto ocp = obst->GetBody(j)->GetWorldPolyhedron().CGAL();

        if(m_debug)
          std::cout << "\t\tbody " << j
                    << " is " << (ocp.is_closed() ? "" : "not ") << "closed"
                    << "\t\t  and " << (ocp.is_valid() ? "" : "not ") << "valid"
                    << std::endl;

        // Subtract it from the freespace.
        freespace -= NefPolyhedron(ocp);
      }
    }
  }

  //display free space
  if(m_debug)
    OutputNefPolyhedron(freespace, "full_freespace.obj");

  // Add free model to tetgen structure.
  AddVertices(m_freeModel, freespace, m_debug);
  AddFacets(m_freeModel, freespace, m_debug);
  AddHoles(m_freeModel, freespace, _env, m_debug);
}


/*------------------------------- IO Helpers ---------------------------------*/

void
TetGenDecomposition::
ValidateSwitches(std::string& _switches) {
  if(_switches.find('p') == string::npos)
    _switches += 'p';
  if(_switches.find('n') == string::npos)
    _switches += 'n';
}


void
TetGenDecomposition::
SaveFreeModel() {
  string basename = m_baseFilename + ".freespace";

  if(m_debug)
    cout << "Saving tetgen free space model with base name '" << basename << "'"
         << endl;

  char* b = const_cast<char*>(basename.c_str());
  m_freeModel->save_nodes(b);
}


void
TetGenDecomposition::
SaveDecompModel() {
  if(m_debug)
    cout << "Saving tetgen decomposition model with base name '"
         << m_baseFilename << "'"
         << endl;

  char* b = const_cast<char*>(m_baseFilename.c_str());
  m_decompModel->save_nodes(b);
  m_decompModel->save_elements(b);
  //m_decompModel->save_neighbors(b);
}


void
TetGenDecomposition::
LoadDecompModel() {
  if(m_debug)
    cout << "Loading tetgen files with base name '" << m_baseFilename << "'"
         << endl;

  char* b = const_cast<char*>(m_baseFilename.c_str());
  m_decompModel->load_node(b);
  m_decompModel->load_tet(b);
  //m_decompModel->load_neighbors(b);
}

/*----------------------------------------------------------------------------*/
