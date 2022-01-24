#ifndef SIMULATOR_CONVERSIONS_H_
#define SIMULATOR_CONVERSIONS_H_

#include <iostream>

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"

#include "Matrix.h"
#include "Quaternion.h"
#include "Transformation.h"
#include "Vector.h"

#include "glutils/gltraits.h"
#include "glutils/color.h"

using namespace mathtool;


/*----------------------------- Bullet Helpers -------------------------------*/

//For changing the friction of an entire btMultiBody (all links + base):
inline
void
SetUniformFrictionOnBulletMultiBody(
    btMultiBody* _body, const double _frictionCoeff) {
  // Note that btMultiBody has links, and each link (including the base) will
  // have its own btMultiBodyLinkCollider, and each one has its own friction
  // setting. This means we must hit each one and update our uniform friction:
  _body->getBaseCollider()->setFriction(_frictionCoeff);
  for(int i = 0; i < _body->getNumLinks(); i++)
    _body->getLink(i).m_collider->setFriction(_frictionCoeff);
}

/*--------------------- Conversion from PMPL to Bullet -----------------------*/

inline
btVector3
ToBullet(const Vector3d& _v) {
  return btVector3(_v[0], _v[1], _v[2]);
}


inline
btQuaternion
ToBullet(const Quaternion& _q) {
  const Vector3d imag = _q.imaginary();
  return btQuaternion(imag[0], imag[1], imag[2], _q.real());
}


inline
btMatrix3x3
ToBullet(const Matrix3x3& _m) {
  return btMatrix3x3(_m[0][0],_m[0][1],_m[0][2],
                     _m[1][0],_m[1][1],_m[1][2],
                     _m[2][0],_m[2][1],_m[2][2]
                    );
}


inline
btTransform
ToBullet(const Transformation& _t) {
  return btTransform(ToBullet(_t.rotation().matrix()),
      ToBullet(_t.translation()));
}

/*--------------------- Conversion from Bullet to PMPL -----------------------*/

inline
Vector3d
ToPMPL(const btVector3& _v) {
  return Vector3d(_v[0], _v[1], _v[2]);
}


inline
Matrix3x3
ToPMPL(const btMatrix3x3& _m) {
  double temp[3][3] = {{_m[0][0], _m[0][1], _m[0][2]},
                       {_m[1][0], _m[1][1], _m[1][2]},
                       {_m[2][0], _m[2][1], _m[2][2]}};
  return Matrix3x3(temp);
}


inline
Transformation
ToPMPL(const btTransform& _t) {
  return Transformation(ToPMPL(_t.getOrigin()), ToPMPL(_t.getBasis()));
}

/*--------------------- Conversion from PMPL to glutils ----------------------*/

inline
glutils::transform
ToGLUtils(const Transformation& _t) {
  const auto& r = _t.rotation().matrix();
  const auto& t = _t.translation();
  // Ignore silly narrowing warnings here.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
  // Note that the glutils transform is in COLUMN MAJOR order to match OpenGL's
  // convention.
  return glutils::transform{r[0][0], r[1][0], r[2][0], 0,
                            r[0][1], r[1][1], r[2][1], 0,
                            r[0][2], r[1][2], r[2][2], 0,
                               t[0],    t[1],    t[2], 1};
#pragma GCC diagnostic pop
}


inline
glutils::vector3f
ToGLUtils(const Vector3d& _v) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
  return {_v[0], _v[1], _v[2]};
#pragma GCC diagnostic pop
}


glutils::color StringToColor(const std::string& _color);

/*------------------------ Output for Bullet Objects -------------------------*/

inline
std::ostream&
operator<<(std::ostream& _out, const btVector3& _v) {
  _out << "(" << _v[0] << ", " << _v[1] << ", " << _v[2] << ")";
  return _out;
}


inline
std::ostream&
operator<<(std::ostream& _out, const btQuaternion& _q) {
  //Printing real part, then x, y, z matches up with PMPL's version.
  _out << "(" << _q.getW() << ", " << _q.getX() << ", " << _q.getY() << ", "
       << _q.getZ() << ")";
  return _out;
}

/*----------------------------------------------------------------------------*/

#endif
