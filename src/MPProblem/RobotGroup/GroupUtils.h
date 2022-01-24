#ifndef GROUP_UTILS_H_
#define GROUP_UTILS_H_

//#include "Transformation.h"
//#include <iostream>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Robot Group Utilities
/// @brief Functions that are general for groups of robots should be placed in
///        here. For now this file is a placeholder until we decide on how
///        robot groups will be fully realized.
////////////////////////////////////////////////////////////////////////////////



///// Given a starting configuration, this function applies a rotation to all the
///// other bodies in the robot that are listed in the body list.
///// @param _bodyList This list of bodies to rotate. First one is leader body.
///// @param _cfg The configuration to rotate from.
///// @param _rotation The change in orientation that should be applied to _cfg.
///// @param _debug A flag to print to cout (no access to an m_debug flag here).
///// @param _dofsPerBody A placeholder to help facilitate different dofs.
///// @return The rotated configuration, with all bodies placed correctly.
//template <typename MPTraits>
//typename MPTraits::CfgType
//RotateCfgAboutBody(const std::vector<unsigned int>& _bodyList,
//                   const typename MPTraits::CfgType& _cfg,
//                   const mathtool::Orientation& _rotation,
//                   const unsigned int _dofsPerBody,
//                   const bool _debug = false) {
//  typedef typename MPTraits::CfgType CfgType;
//
//  // compute position and rotation of part A, BEFORE rotation:
//  _cfg.GetMultiBody()->Configure(_cfg);//Update transform
//  mathtool::Transformation initTA = _cfg.GetMultiBody()->
//              GetBody(_bodyList[0])->GetWorldTransformation();
//
//  const mathtool::Transformation rotation(mathtool::Vector3d(0,0,0), _rotation);
//
//  if(_debug)
//    std::cout << "Rotating bodies " << _bodyList << " with rotation = "
//              << rotation << std::endl;
//
//  // The transform to be applied to all parts (including the first one). We
//  // move the part to its relative world position with A at the world origin,
//  // then the rotation is applied, and we return the part to its relative
//  // position from A.
//  const mathtool::Transformation transform = initTA * rotation;
//
//  //Since we deal with composite CSpace, DOFs not in _bodyList should be
//  // unchanged wrt the input cfg.
//  CfgType rotatedCfg = _cfg;
//
//  //Compute each body's needed transformation and set dofs in cfg.
//  for (const unsigned int body : _bodyList) {
//    // compute position and rotation of part B
//    const mathtool::Transformation initTB = _cfg.GetMultiBody()->
//                               GetBody(body)->GetWorldTransformation();
//
//    const mathtool::Transformation TB = transform * -initTA * initTB;
//
//    //Extract the transformation:
//    std::vector<double> transformed = TB.GetCfg();
//
//    for(size_t i = 0; i < _dofsPerBody; ++i)
//      rotatedCfg[body*_dofsPerBody + i] = transformed[i];
//  }
//
//  return rotatedCfg;
//}
//
//
////template <typename MPTraits>
////size_t
////GetApproximateFormationResolution(const typename MPTraits::CfgType& _start,
////                                  const typename MPTraits::CfgType& _target,
////                                  const std::vector<unsigned int> _bodyList,
////                                  const double _posRes,
////                                  const double _oriRes) {
////  //Returns the number of extra steps per tick to make _start -> _target conform
////  // to _posRes and _oriRes for the bodies in _bodyList.
////  //Assume _start and _target are a single tick away from each other, with
////  // respect to the Cfg::FindIncrement function.
////
////  //TODO: When fixing orientation resolution in PMPL, we need to also properly
////  // handle the case here, for now it's left for the future though.
////
////}
//
///// Given a configuration, add in the same DOF values to each body given.
///// This is a common thing to do in assembly planning/composite C-Spaces.
///// @param _cfg The configuration to add to (done in-place).
///// @param _dofs The values to add in to each body. This function assumes each
/////              body has #dofs = _dofs.size().
///// @param _bodies This list of bodies to update. Order doesn't matter.
//template <typename MPTraits>
//void
//AddDofsForBodies(typename MPTraits::CfgType& _cfg,
//                 const std::vector<double>& _dofs,
//                 const std::vector<unsigned int>& _bodies) {
//  // This function adds all dofs (assumes #dofs per body is correct) to each
//  // body given in _bodies to _cfg.
//  const unsigned int dofsPerBody = _dofs.size();
//  for(const unsigned int body : _bodies)
//    for (unsigned int i = 0; i < dofsPerBody; ++i)
//      _cfg[(body * dofsPerBody) + i] += _dofs[i];
//}
//
//
///// Given 2 configurations, copy the DOF values from one to the DOF values in
///// the other, only for each given body.
///// @param _toCfg The configuration to set (done in-place).
///// @param _fromCfg The configuration to take values from.
///// @param _bodies This list of bodies to update. Order doesn't matter.
//template <typename MPTraits>
//void
//OverwriteDofsFromBodies(typename MPTraits::CfgType& _toCfg,
//                        const typename MPTraits::CfgType& _fromCfg,
//                        const std::vector<unsigned int>& _bodies) {
//  // This function assigns the values from _fromCfg into _toCfg, based on which
//  // bodies are given in _bodies.
//  const unsigned int dofsPerBody = _toCfg.PosDOF() + _toCfg.OriDOF();
//  for(const unsigned int body : _bodies) {
//    const unsigned int bodyOffset = body * dofsPerBody;
//    for (unsigned int i = 0; i < dofsPerBody; ++i) {
//      const unsigned int dofIndex = bodyOffset + i;
//      _toCfg[dofIndex] = _fromCfg[dofIndex];
//    }
//  }
//}

#endif
