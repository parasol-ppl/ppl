#include "CSpaceBoundingSphere.h"


/*------------------------------ Construction --------------------------------*/

CSpaceBoundingSphere::
CSpaceBoundingSphere(const size_t _n, const double _radius)
  : AbstractBoundingSphere(_n, _radius) { }


CSpaceBoundingSphere::
CSpaceBoundingSphere(const std::vector<double>& _center, const double _radius)
  : AbstractBoundingSphere(_center, _radius) { }


CSpaceBoundingSphere::
CSpaceBoundingSphere(XMLNode& _node) : AbstractBoundingSphere(_node) { }


CSpaceBoundingSphere::
~CSpaceBoundingSphere() noexcept = default;


std::unique_ptr<Boundary>
CSpaceBoundingSphere::
Clone() const {
  return std::unique_ptr<CSpaceBoundingSphere>(new CSpaceBoundingSphere(*this));
}

/*---------------------------- Property Accessors ----------------------------*/

Boundary::Space
CSpaceBoundingSphere::
Type() const noexcept {
  return Boundary::Space::CSpace;
}


std::string
CSpaceBoundingSphere::
Name() const noexcept {
  return "CSpaceBoundingSphere";
}

/*--------------------------- Containment Testing ----------------------------*/

bool
CSpaceBoundingSphere::
InBoundary(const Cfg& _cfg) const {
  return InCSpace(_cfg);
}

/*----------------------------------------------------------------------------*/
