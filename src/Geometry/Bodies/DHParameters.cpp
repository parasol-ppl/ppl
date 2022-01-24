#include "DHParameters.h"

#include "Transformation.h"

/*------------------------------ Construction --------------------------------*/

DHParameters::
DHParameters(double _alpha, double _a, double _d, double _theta) :
    m_alpha(_alpha), m_a(_a), m_d(_d), m_theta(_theta) {
}

/*-------------------------------- Conversion --------------------------------*/

Transformation
DHParameters::
GetTransformation() const {
  Matrix3x3 rot;
  /* This is the old implementation that uses 'alternative' DH params. It was
   * used for the ocean one and any problems older than that. I am keeping it
   * around in case we find a problem with the standard notation.
  Vector3d pos(m_a, -sin(m_alpha)*m_d, cos(m_alpha)*m_d);
  getMatrix3x3(rot,
      cos(m_theta), -sin(m_theta), 0.0,
      sin(m_theta)*cos(m_alpha), cos(m_theta)*cos(m_alpha), -sin(m_alpha),
      sin(m_theta)*sin(m_alpha), cos(m_theta)*sin(m_alpha), cos(m_alpha));
  */
  const double ca = std::cos(m_alpha),
               sa = std::sin(m_alpha),
               ct = std::cos(m_theta),
               st = std::sin(m_theta);
  Vector3d pos(m_a * ct, m_a * st, m_d);
  getMatrix3x3(rot,
      ct, -st * ca,  st * sa,
      st,  ct * ca, -ct * sa,
       0,       sa,       ca);
  return Transformation(pos, Orientation(rot));
}

/*---------------------------------- I/O -------------------------------------*/

istream&
operator>>(istream& _is, DHParameters& _d) {
  return _is >> _d.m_alpha >> _d.m_a >> _d.m_d >> _d.m_theta;
}


ostream&
operator<<(ostream& _os, const DHParameters& _d) {
  return _os << _d.m_alpha << " " << _d.m_a << " "
    << _d.m_d << " " << _d.m_theta << " ";
}

/*----------------------------------------------------------------------------*/
