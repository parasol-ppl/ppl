#ifndef PPL_COLLISION_DETECTION_METHOD_TEST_H_
#define PPL_COLLISION_DETECTION_METHOD_TEST_H_

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/Environment/Environment.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/MPProblem.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CollisionDetectionMethod.h"
#include "Testing/TestBaseObject.h"

class CollisionDetectionMethodTest : virtual public CollisionDetectionMethod,
                                  public TestBaseObject {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult      TestResult;


    ///@}
    ///@name Construction
    ///@{

    CollisionDetectionMethodTest();

    CollisionDetectionMethodTest(MPProblem* _MPProblem);

    ~CollisionDetectionMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  protected:

    ///@name Interface Test Functions
    ///@{

    virtual TestResult IndividualCfgValidityTest() = 0;

    virtual TestResult MultipleCfgValidityTest() = 0;

    ///@}
    ///@name Default Function Calls
    ///@{

    virtual bool IndividualCfgValidity(Cfg _a);

    virtual bool CheckCollision(const MultiBody* const _a, const MultiBody* const _b);

    ///@}

    MPProblem* m_MPProblem {nullptr};

};

/*--------------------------- Construction ---------------------------*/

CollisionDetectionMethodTest::
CollisionDetectionMethodTest() : CollisionDetectionMethod() {}

CollisionDetectionMethodTest::
CollisionDetectionMethodTest(MPProblem* _problem) : CollisionDetectionMethod() {
  m_MPProblem = _problem;
}

CollisionDetectionMethodTest::
~CollisionDetectionMethodTest() {}

/*----------------------------- Interface ----------------------------*/

typename CollisionDetectionMethodTest::TestResult
CollisionDetectionMethodTest::
RunTest() {
  bool passed = true;
  std::string message = "";

  auto result = IndividualCfgValidityTest();
  passed = passed and result.first;
  message = message += result.second;

  result = MultipleCfgValidityTest();
  passed = passed and result.first;
  message = message += result.second;

  return std::make_pair(passed,message);
}

/*----------------------- Default Function Calls ---------------------*/

bool
CollisionDetectionMethodTest::
IndividualCfgValidity(Cfg _a) {
  Environment* env = m_MPProblem->GetEnvironment();

  // check collisions between cfg and environment obstacles
  for (size_t i = 0; i < env->NumObstacles(); i++) {
    auto obstacle = env->GetObstacle(i);
    if(CheckCollision(_a.GetMultiBody(), obstacle))
      return true;
  }

  return false;

}

bool
CollisionDetectionMethodTest::
CheckCollision(const MultiBody* const _a, const MultiBody* const _b){
  CDInfo cdInfo;
  for (size_t i = 0; i < _a->GetNumBodies(); i++) {
    const Body* const b1 = _a->GetBody(i);
      for (size_t j = 0; j < _b->GetNumBodies(); j++) {
        const Body* const b2 = _b->GetBody(j);
        auto collision = this->IsInCollision(b1->GetPolyhedron(),
                                             b1->GetWorldTransformation(),
                                             b2->GetPolyhedron(),
                                             b2->GetWorldTransformation(),
                                             cdInfo);
        if (collision)
          return true;
      }
  }
  return false;
}

#endif
