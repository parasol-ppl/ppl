#ifndef COLOR_H_
#define COLOR_H_

#include <Vector.h>

typedef mathtool::Vector<float, 3> Color3; ///< RGB color, range from 0-1
typedef mathtool::Vector<float, 4> Color4; ///< RGBA color, range from 0-1

#ifdef VIZMO

#ifdef __APPLE__
  #include <OpenGL/gl.h>
#else
  #include <gl.h>
#endif

inline void SetGLColor(const Color3& _c) {glColor3fv(_c);} ///< Set color RGB
inline void SetGLColor(const Color4& _c) {glColor4fv(_c);} ///< Set color RGBA

#endif

#endif
