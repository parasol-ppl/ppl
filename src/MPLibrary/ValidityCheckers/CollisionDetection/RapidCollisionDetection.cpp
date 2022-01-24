#include "RapidCollisionDetection.h"

#include "CDInfo.h"
#include "Geometry/GMSPolyhedron.h"
#include "Utilities/PMPLExceptions.h"

#include "RAPID.H"


/*------------------------------- Construction -------------------------------*/

std::mutex Rapid::s_lock;

Rapid::
Rapid() : CollisionDetectionMethod("RAPID") { }


Rapid::
~Rapid() = default;

/*---------------------------- Model Construction ----------------------------*/

RAPID_model*
Rapid::
Build(const GMSPolyhedron& _polyhedron) {
  // Even for building models, RAPID isn't thread-safe.
  std::lock_guard<std::mutex> guard(s_lock);

  const auto& facets = _polyhedron.GetPolygonList();

  RAPID_model* rapidModel(new RAPID_model);
  rapidModel->BeginModel();
  for(size_t q = 0; q < facets.size(); q++) {
    double point[3][3];
    for(int i = 0; i < 3; i++) {
      const Vector3d& tmp = facets[q].GetPoint(i);
      for(int j = 0; j < 3; j++)
        point[i][j] = tmp[j];
    }
    rapidModel->AddTri(point[0], point[1], point[2], q);
  }
  rapidModel->EndModel();

  return rapidModel;
}

/*------------------------------- CD Interface -------------------------------*/

bool
Rapid::
IsInCollision(
    const GMSPolyhedron& _polyhedron1,
    const mathtool::Transformation& _transformation1,
    const GMSPolyhedron& _polyhedron2,
    const mathtool::Transformation& _transformation2,
    CDInfo& _cdInfo) {
  auto model1 = _polyhedron1.GetRapidModel();
  auto model2 = _polyhedron2.GetRapidModel();

  /// @TODO See if we can modify RAPID to accept const values to avoid the copy
  ///       here.
  Transformation t1 = _transformation1;
  Transformation t2 = _transformation2;

  const int flag = _cdInfo.m_retAllInfo ? RAPID_ALL_CONTACTS
                                        : RAPID_FIRST_CONTACT;

  std::lock_guard<std::mutex> guard(s_lock);
  if(RAPID_Collide(
        t1.rotation().matrix(), t1.translation(), model1,
        t2.rotation().matrix(), t2.translation(), model2,
        flag))
  {
    throw RunTimeException(WHERE) << "RAPID out of memory.";
  }

  for(int i = 0; i < RAPID_num_contacts; ++i)
    _cdInfo.m_trianglePairs.emplace_back(RAPID_contact[i].id1,
                                         RAPID_contact[i].id2);

  return RAPID_num_contacts;
}

/*----------------------------------------------------------------------------*/
