#include "GMSPolygon.h"

#include <algorithm>

#include "Utilities/MPUtils.h"

using namespace std;

/*-------------------------------- Construction ------------------------------*/

GMSPolygon::
GMSPolygon() = default;


GMSPolygon::
GMSPolygon(const int _v1, const int _v2, const int _v3,
    const vector<Point3d>& _pts) : m_indexes{_v1, _v2, _v3}, m_pointList(&_pts) {
  AlignIndexes();
  ComputeNormal();
}


GMSPolygon::
GMSPolygon(const_iterator _begin, const_iterator _end, const PointList& _pts)
    : m_indexes(_begin, _end), m_pointList(&_pts) {
  AlignIndexes();
  ComputeNormal();
}

/*-------------------------------- Iterators ---------------------------------*/

GMSPolygon::const_iterator
GMSPolygon::
begin() const noexcept {
  return m_indexes.begin();
}


GMSPolygon::const_iterator
GMSPolygon::
end() const noexcept {
  return m_indexes.end();
}

/*--------------------------------- Accessors --------------------------------*/

const int&
GMSPolygon::
operator[](const size_t _i) const noexcept {
  return m_indexes[_i];
}


size_t
GMSPolygon::
GetNumVertices() const noexcept {
  return m_indexes.size();
}


const Point3d&
GMSPolygon::
GetPoint(const size_t _i) const noexcept {
  return (*m_pointList)[m_indexes[_i]];
}


Vector3d&
GMSPolygon::
GetNormal() noexcept {
  return m_normal;
}


const Vector3d&
GMSPolygon::
GetNormal() const noexcept {
  return m_normal;
}


double
GMSPolygon::
GetArea() const noexcept {
  return m_area;
}

/*--------------------------------- Modifiers --------------------------------*/

void
GMSPolygon::
Reverse() {
  std::reverse(++m_indexes.begin(), m_indexes.end());
  m_normal *= -1;
}


void
GMSPolygon::
ComputeNormal() {
  const Vector3d v1 = GetPoint(1) - GetPoint(0);
  const Vector3d v2 = GetPoint(2) - GetPoint(0);
  m_normal = v1 % v2;
  const double norm = m_normal.norm();
  m_normal /= norm;
  m_area = .5 * norm;
}

/*-------------------------------- Equality ----------------------------------*/

bool
GMSPolygon::
operator==(const GMSPolygon& _p) const noexcept {
  // We don't check m_pointList as it would prohibit checking if two polyhedrons
  // represent the same shape. We leave it to the polyhedron to check if the
  // point lists are value-equal.
  return m_area == _p.m_area
     and m_normal == _p.m_normal
     and m_indexes == _p.m_indexes;
}


bool
GMSPolygon::
operator!=(const GMSPolygon& _p) const noexcept {
  return !(*this == _p);
}

/*--------------------------------- Ordering ---------------------------------*/

bool
GMSPolygon::
operator<(const GMSPolygon& _other) const noexcept {
  return m_area < _other.m_area;
}


bool
GMSPolygon::
operator>(const GMSPolygon& _other) const noexcept {
  return m_area > _other.m_area;
}

/*------------------------------- Queries ------------------------------------*/

const bool
GMSPolygon::
IsTriangle() const noexcept {
  return m_indexes.size() == 3 &&
      m_indexes[0] != m_indexes[1] &&
      m_indexes[1] != m_indexes[2] &&
      m_indexes[0] != m_indexes[2];
}


const Point3d
GMSPolygon::
FindCenter() const noexcept {
  Point3d center;
  for(size_t i = 0; i < m_indexes.size(); ++i)
    center += GetPoint(i);
  return center /= m_indexes.size();
}


const bool
GMSPolygon::
PointIsAbove(const Point3d& _p) const noexcept {
  return (_p - GetPoint(0)) * m_normal > 0;
}


const int
GMSPolygon::
CommonVertex(const GMSPolygon& _p) const noexcept {
  for(size_t i = 0; i < m_indexes.size(); ++i) {
    for(size_t j = 0; j < m_indexes.size(); ++j) {
      if(m_indexes[i] == _p.m_indexes[j]) {
        return m_indexes[i];
      }
    }
  }
  return -1;
}


const std::pair<int, int>
GMSPolygon::
CommonEdge(const GMSPolygon& _p) const noexcept {
  std::pair<int, int> edgeID(-1, -1);
  for(size_t i = 0; i < m_indexes.size(); ++i) {
    for(size_t j = 0; j < m_indexes.size(); ++j) {
      if(m_indexes[i] == _p.m_indexes[j]) {
        if(edgeID.first == -1)
          edgeID.first = m_indexes[i];
        else
          edgeID.second = m_indexes[i];
      }
    }
  }
  return edgeID;
}

/*--------------------------------- Helpers ----------------------------------*/

void
GMSPolygon::
AlignIndexes() noexcept {
  auto iter = std::min_element(m_indexes.begin(), m_indexes.end());
  std::rotate(m_indexes.begin(), iter, m_indexes.end());
}

/*----------------------------------------------------------------------------*/
