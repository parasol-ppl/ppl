#include "GridOverlay.h"

#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "Geometry/GMSPolyhedron.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "Utilities/MPUtils.h"
#include "Workspace/WorkspaceDecomposition.h"

#include "nonstd/io.h"
#include "nonstd/timer.h"

#include <cmath>


/*------------------------------- Construction -------------------------------*/

GridOverlay::
GridOverlay(const Boundary* const _b, const double _length):
    m_boundary(_b),  m_length(_length) {
  // Compute the number of cells in each dimension.
  for(size_t i = 0; i < 3; ++i)
    m_num[i] = std::ceil(m_boundary->GetRange(i).Length() / m_length);

  if(m_debug)
    std::cout << "Computed grid overlay with " << Size() << " cells "
              << "(" << Size(0) << "x" << Size(1) << "x" << Size(2) << ") "
              << "of length " << m_length << "."
              << std::endl;

  // Build a GMSPolyhedron model of a single grid cell.
  const double halfLength = m_length / 2.;
  const Range<double> range(-halfLength, halfLength);
  m_polyhedron.reset(new GMSPolyhedron(
      GMSPolyhedron::MakeBox(range, range, range)
  ));
}


GridOverlay::
~GridOverlay() noexcept = default;

/*------------------------------- Cell Finding -------------------------------*/

size_t
GridOverlay::
Size() const noexcept {
  return m_num[0] * m_num[1] * m_num[2];
}


size_t
GridOverlay::
Size(const size_t _i) const noexcept {
  return m_num[_i];
}


size_t
GridOverlay::
LocateCell(const Cfg& _cfg) const {
  return LocateCell(_cfg.GetPoint());
}


size_t
GridOverlay::
LocateCell(const Point3d& _p) const {
  return CellIndex(Cell(_p));
}


std::unordered_set<size_t>
GridOverlay::
LocateCells(const Boundary* const _b, const CellSet _type) const {
  GMSPolyhedron polyhedron = _b->MakePolyhedron();
  polyhedron.Invert();
  return this->LocateCells(polyhedron, mathtool::Transformation(), _type);
}


std::unordered_set<size_t>
GridOverlay::
LocateCells(const GMSPolyhedron& _polyhedron, const mathtool::Transformation& _t,
    const CellSet _type) const {
  /// @todo Can make the !_interior version more efficient by computing bbx
  ///       cells for each facet.
  /// @todo Can make both versions more efficient by avoiding re-building the
  ///       _polyhedron's BBX each time. Add min/max pts and transform them with
  ///       the poly, use here.
  static PQP pqp;
  static PQPSolid pqpSolid;

  // Find the BBX cells for this polyhedron.
  std::unordered_set<size_t> bbxCells = LocateBBXCells(
      _polyhedron.ComputeBoundingBox().get());

  CDInfo cdInfo;
  std::unordered_set<size_t> cells;
  cells.reserve(bbxCells.size());

  for(const size_t cell : bbxCells) {
    Point3d center = CellCenter(cell);
    mathtool::Transformation t(center);
    bool touching;
    switch(_type) {
      case CellSet::Boundary:
        touching = pqp.IsInCollision(*m_polyhedron, t,
                                     _polyhedron, _t, cdInfo);
        break;
      case CellSet::Interior:
        touching = pqpSolid.IsInsideObstacle(center, _polyhedron, _t);
        break;
      case CellSet::Closure:
        touching = pqpSolid.IsInCollision(*m_polyhedron, t,
                                          _polyhedron, _t, cdInfo);
        break;
      default:
        throw RunTimeException(WHERE) << "Unrecognized cell set type.";
    }

    if(touching)
      cells.insert(cell);
  }

  return cells;
}


std::unordered_set<size_t>
GridOverlay::
LocateBBXCells(const Boundary* const _b) const {
  Point3d min, max;

  for(size_t i = 0; i < 3; ++i) {
    auto range = _b->GetRange(i);
    min[i] = range.min;
    max[i] = range.max;
  }

  return LocateBBXCells(min, max);
}


std::unordered_set<size_t>
GridOverlay::
LocateBBXCells(const Point3d& _min, const Point3d& _max) const {
  const auto min = Cell(_min);
  const auto max = Cell(_max);

  // Create space for the appropriate number of cells.
  std::unordered_set<size_t> output;
  const size_t expectedSize = (max[0] - min[0] + 1) *
                              (max[1] - min[1] + 1) *
                              (max[2] - min[2] + 1);
  output.reserve(expectedSize);

  // Populate the cell list.
  for(size_t z = min[2]; z <= max[2]; ++z) {
    for(size_t y = min[1]; y <= max[1]; ++y) {
      const size_t first = CellIndex(min[0], y, z);
      const size_t last  = CellIndex(max[0], y, z);
      for(size_t i = first; i <= last; ++i)
        output.insert(i);
    }
  }

#if 1
  // Assert that the maxes are >= the mins.
  if(max[0] < min[0] or max[1] < min[1] or max[2] < min[2])
    throw RunTimeException(WHERE) << "Maxes " << max << " less than mins "
                                  << min;
  // Assert that we got the right number of cells.
  if(output.size() != expectedSize)
    throw RunTimeException(WHERE) << "Wrong number of BBX cells. Expected "
                                  << expectedSize << ", got "
                                  << output.size() << ".";
#endif

  return output;
}


std::unordered_set<size_t>
GridOverlay::
LocateFacetNeighbors(const size_t _index) const {
  // Find the x/y/z indexes for this cell.
  const size_t x = XIndex(_index),
               y = YIndex(_index),
               z = ZIndex(_index);

  // There are six possible facet neighbors.
  // Each neighbor has one x/y/z index incremented or decremented.
  std::array<IndexSet, 6> indexes{IndexSet{x + 1, y    , z    },
                                  IndexSet{x - 1, y    , z    },
                                  IndexSet{x    , y + 1, z    },
                                  IndexSet{x    , y - 1, z    },
                                  IndexSet{x    , y    , z + 1},
                                  IndexSet{x    , y    , z - 1}};

  // Test each possible neighbor to ensure it's in the grid.
  std::unordered_set<size_t> output;
  output.reserve(6);

  for(const auto& i : indexes)
    if(InGrid(i))
      output.insert(CellIndex(i));
  return output;
}


std::unordered_set<size_t>
GridOverlay::
LocateEdgeNeighbors(const size_t _index) const {
  // Find the x/y/z indexes for this cell.
  const size_t x = XIndex(_index),
               y = YIndex(_index),
               z = ZIndex(_index);

  // There are twelve possible facet neighbors.
  // Each neighbor has two x/y/z indexes incremented or decremented.
  std::array<IndexSet, 12> indexes{IndexSet{x    , y + 1, z + 1},
                                   IndexSet{x    , y + 1, z - 1},
                                   IndexSet{x    , y - 1, z + 1},
                                   IndexSet{x    , y - 1, z - 1},
                                   IndexSet{x + 1, y    , z + 1},
                                   IndexSet{x + 1, y    , z - 1},
                                   IndexSet{x - 1, y    , z + 1},
                                   IndexSet{x - 1, y    , z - 1},
                                   IndexSet{x + 1, y + 1, z    },
                                   IndexSet{x + 1, y - 1, z    },
                                   IndexSet{x - 1, y + 1, z    },
                                   IndexSet{x - 1, y - 1, z    }};

  // Test each possible neighbor to ensure it's in the grid.
  std::unordered_set<size_t> output;
  output.reserve(12);

  for(const auto& i : indexes)
    if(InGrid(i))
      output.insert(CellIndex(i));
  return output;
}


std::unordered_set<size_t>
GridOverlay::
LocateVertexNeighbors(const size_t _index) const {
  // Find the x/y/z indexes for this cell.
  const size_t x = XIndex(_index),
               y = YIndex(_index),
               z = ZIndex(_index);

  // There are eight possible vertex neighbors.
  // Each neighbor has all three x/y/z indexes incremented or decremented.
  std::array<IndexSet, 8> indexes{IndexSet{x + 1, y + 1, z + 1},
                                  IndexSet{x + 1, y + 1, z - 1},
                                  IndexSet{x + 1, y - 1, z + 1},
                                  IndexSet{x + 1, y - 1, z - 1},
                                  IndexSet{x - 1, y + 1, z + 1},
                                  IndexSet{x - 1, y + 1, z - 1},
                                  IndexSet{x - 1, y - 1, z + 1},
                                  IndexSet{x - 1, y - 1, z - 1}};

  // Test each possible neighbor to ensure it's in the grid.
  std::unordered_set<size_t> output;
  output.reserve(8);

  for(const auto& i : indexes)
    if(InGrid(i))
      output.insert(CellIndex(i));
  return output;
}


std::unordered_set<size_t>
GridOverlay::
LocateAllNeighbors(const size_t _index) const {
  std::unordered_set<size_t> allNeighbors;
  allNeighbors.reserve(26);

  {
    const auto neighbors = LocateFacetNeighbors(_index);
    allNeighbors.insert(neighbors.begin(), neighbors.end());
  }
  {
    const auto neighbors = LocateEdgeNeighbors(_index);
    allNeighbors.insert(neighbors.begin(), neighbors.end());
  }
  {
    const auto neighbors = LocateVertexNeighbors(_index);
    allNeighbors.insert(neighbors.begin(), neighbors.end());
  }

  return allNeighbors;
}

/*------------------------------- Cell Finders -------------------------------*/

double
GridOverlay::
CellLength() const noexcept {
  return m_length;
}


Point3d
GridOverlay::
CellCenter(const size_t _index) const noexcept {
  // Find the dimensional indexes of each cell. Each should be in the range of 0
  // to m_num - 1.
  const IndexSet index{this->XIndex(_index),
                       this->YIndex(_index),
                       this->ZIndex(_index)};

  Point3d center;
  const double halfLength = m_length / 2.;

  // Find the value of the minimum corner in each dimension and add the
  // half-length to get the center.
  for(size_t i = 0; i < 3; ++i) {
    const auto range = m_boundary->GetRange(i);
    center[i] = index[i] * m_length + range.min + halfLength;
  }

  return center;
}

/*-------------------------- Decomposition Mapping ---------------------------*/

GridOverlay::DecompositionMap
GridOverlay::
ComputeDecompositionMap(const WorkspaceDecomposition* const _decomposition,
    const bool _useCollisionDetection) const
{
  DecompositionMap map(this->Size());

  // For each region, find the grid cells that are associated with it.
  for(auto iter = _decomposition->begin(); iter != _decomposition->end(); ++iter)
  {
    auto region   = &iter->property();
    auto boundary = region->GetBoundary();
    auto cells    = _useCollisionDetection ? LocateCells(boundary, CellSet::Closure)
                                           : LocateBBXCells(boundary);

    for(auto index : cells)
      map[index].push_back(region);
  }

  return map;
}

/*------------------------------- Helpers ------------------------------------*/

GridOverlay::IndexSet
GridOverlay::
Cell(const Point3d& _p) const noexcept {
  IndexSet cell;

  for(size_t i = 0; i < 3; ++i) {
    const auto range = m_boundary->GetRange(i);
    cell[i] = (_p[i] - range.min) / m_length;
    // Catch edge-case for cells on the maximal boundary.
    cell[i] = std::min(cell[i], m_num[i] - 1);
  }

  return cell;
}


size_t
GridOverlay::
CellIndex(const size_t _x, const size_t _y, const size_t _z) const noexcept {
  return (m_num[0] * m_num[1]) * _z + m_num[0] * _y + _x;
}


size_t
GridOverlay::
CellIndex(const IndexSet& _indexes) const noexcept {
  return CellIndex(_indexes[0], _indexes[1], _indexes[2]);
}


size_t
GridOverlay::
ZIndex(const size_t _index) const noexcept {
  // To find the z-index, we see how many x-y slices we can remove.
  return _index / (m_num[0] * m_num[1]);
}


size_t
GridOverlay::
YIndex(const size_t _index) const noexcept {
  // To find the y-index, we first remove the z components and then see how many
  // x rows we can remove.
  return (_index % (m_num[0] * m_num[1])) / m_num[0];
}


size_t
GridOverlay::
XIndex(const size_t _index) const noexcept {
  // To find the x-index, we remove the z and y components.
  return _index % (m_num[0] * m_num[1]) % m_num[0];
}


bool
GridOverlay::
InGrid(const IndexSet& _indexes) const noexcept {
  return 0 <= _indexes[0] and _indexes[0] <= m_num[0] - 1
     and 0 <= _indexes[1] and _indexes[1] <= m_num[1] - 1
     and 0 <= _indexes[2] and _indexes[2] <= m_num[2] - 1;
}

/*-------------------------------- Testing -----------------------------------*/

void
GridOverlay::
Test(const size_t _trials) const {
  // If we asked for 0 trials, do not sample. Check each grid cell in this case.
  const bool sampling = _trials != 0;
  const size_t trials = sampling ? _trials : this->Size();
  const size_t fivePercent = .05 * trials;
  size_t percentComplete = 0;
  nonstd::timer t;
  t.start();

  std::cout << "Testing GridOverlay."
            << "\n\tSampling:  " << (sampling ? "yes" : "no")
            << "\n\tTrials:    " << trials
            << "\n\tNum cells: " << this->Size()
            << "\n\tNum X:     " << m_num[0]
            << "\n\tNum Y:     " << m_num[1]
            << "\n\tNum Z:     " << m_num[2]
            << "\nBoundary Information: " << *m_boundary
            << std::endl;

  for(size_t i = 0; i < 3; ++i)
    std::cout << "\tlength " << i << ": " << m_boundary->GetRange(i).Length()
              << std::endl;


  // Compute a value that is just slightly smaller than the half-length of the
  // cell.
  const double halfLength = .98 * (m_length / 2.);

  // Compute a displacement of one 'halfLength' in each dimension for testing
  // cell centers.
  const Point3d testDisplacement(halfLength, halfLength, halfLength);

  // Generate a ton of cell indexes. For each index, get the x,y,z indexes and
  // confirm that they return the original cell index.
  for(size_t i = 0; i < trials; ++i)
  {
    // Get the next cell index.
    const size_t cell = sampling ? LRand() % Size()
                                 : i;

    // Test that we can get the dimensional indices and recover the original
    // index.
    const size_t x          = XIndex(cell),
                 y          = YIndex(cell),
                 z          = ZIndex(cell),
                 testIndex  = CellIndex(x, y, z);
    const bool indexTranslationOK = x < m_num[0]
                                and y < m_num[1]
                                and z < m_num[2]
                                and testIndex == cell;

    // Test that we can get the cell center and determine it belongs in the
    // original cell.
    const Point3d center = CellCenter(cell);
    const size_t centerIndex = LocateCell(center);
    const bool centerOK = centerIndex == cell;

    // Test that we can apply the ~half length displacement to the cell center
    // in either direction and get back the same cell.
    const size_t addIndex = LocateCell(center + testDisplacement),
                 subIndex = LocateCell(center - testDisplacement);
    const bool addOK = addIndex == cell,
               subOK = subIndex == cell;



    if(!indexTranslationOK or !centerOK or !addOK or !subOK) {
      std::cout << "GridOverlay: test failed on trial " << i << ":"
                << "\n\tcell:         " << cell
                << " (" << x
                << "," << y
                << "," << z
                << ")"
                << "\n\ttestIndex:    " << testIndex
                << "\n\tcenter:       " << center
                << "\n\tCell(center): " << Cell(center)
                << "\n\tcenterIndex:  " << centerIndex
                << " (" << XIndex(centerIndex)
                << "," << YIndex(centerIndex)
                << "," << ZIndex(centerIndex)
                << ")"
                << "\n\tadd:          " << center + testDisplacement
                << "\n\tCell(add):    " << Cell(center + testDisplacement)
                << "\n\taddIndex:     " << addIndex
                << " (" << XIndex(addIndex)
                << "," << YIndex(addIndex)
                << "," << ZIndex(addIndex)
                << ")"
                << "\n\tsub:          " << center - testDisplacement
                << "\n\tCell(sub):    " << Cell(center - testDisplacement)
                << "\n\tsubIndex:     " << subIndex
                << " (" << XIndex(subIndex)
                << "," << YIndex(subIndex)
                << "," << ZIndex(subIndex)
                << ")"
                << std::endl;
      if(!indexTranslationOK)
        std::cout << "Index translation failed." << std::endl;
      if(!centerOK)
        std::cout << "Center test failed." << std::endl;
      if(!addOK)
        std::cout << "Add test failed." << std::endl;
      if(!subOK)
        std::cout << "Subtract test failed." << std::endl;
      throw RunTimeException(WHERE) << "Test failed.";
    }

    if((i + 1) % fivePercent == 0) {
      percentComplete += 5;
      std::cout << percentComplete << "% complete. ("
                << t.elapsed()
                << " seconds)"
                << std::endl;
    }
  }

  std::cout << "GridOverlay::Test passed. ("
            << t.elapsed()
            << " seconds)"
            << std::endl;
}

/*----------------------------------------------------------------------------*/
