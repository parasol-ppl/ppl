#ifndef PPL_INSIDE_SPHERES_COLLISION_DETECTION_TEST_H_
#define PPL_INSIDE_SPHERES_COLLISION_DETECTION_TEST_H_

#include "CollisionDetectionMethodTest.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/SpheresCollisionDetection.h"

class InsideSpheresCollisionDetectionTest :  public InsideSpheres,
                               public CollisionDetectionMethodTest {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    InsideSpheresCollisionDetectionTest();

    InsideSpheresCollisionDetectionTest(MPProblem* _problem);

    ~InsideSpheresCollisionDetectionTest();

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult IndividualCfgValidityTest() override;

    virtual TestResult MultipleCfgValidityTest() override;

    ///@}

    ///@name CollisionDetectionMethod Overrides
    ///@{
    virtual bool IsInCollision(
        const GMSPolyhedron& _polyhedron1, const mathtool::Transformation& _t1,
        const GMSPolyhedron& _polyhedron2, const mathtool::Transformation& _t2,
        CDInfo& _cdInfo) override;
    ///@}

};

/*--------------------------- Construction ---------------------------*/

InsideSpheresCollisionDetectionTest::
InsideSpheresCollisionDetectionTest() : InsideSpheres() {}

InsideSpheresCollisionDetectionTest::
InsideSpheresCollisionDetectionTest(MPProblem* _problem) : CollisionDetectionMethod(),
                                                             InsideSpheres(){
  m_MPProblem = _problem;
  }

InsideSpheresCollisionDetectionTest::
~InsideSpheresCollisionDetectionTest() {}



/*--------------------- Test Interface Functions ---------------------*/

typename InsideSpheresCollisionDetectionTest::TestResult
InsideSpheresCollisionDetectionTest::
IndividualCfgValidityTest() {
  auto robot = m_MPProblem->GetRobots()[0].get();

  bool passed = true;
  std::string message = "";

  Cfg cfg(robot);
  // when robot is at center of environment, inside spheres should return that
  // it is in collision with the obstacle
  bool invalid = this->IndividualCfgValidity(cfg);
  if (!invalid){
    passed = false;
    message = message + "\n\tA cfg was incorrectly labeled invalid.\n";
  }

  // place configuration inside an obstacle
  cfg[0] = 0;
  cfg[1] = 2;
  cfg.ConfigureRobot();
  invalid = this->IndividualCfgValidity(cfg);
  if (!invalid) {
    passed = false;
    message = message + "\n\tA cfg was incorrectly labeled valid.\n";
  }

  // place configuration away from any obstacles
  cfg[0] = 16;
  cfg[1] = 14;
  cfg[2] = 16;
  cfg.ConfigureRobot();
  invalid = this->IndividualCfgValidity(cfg);
  if (invalid) {
    passed = false;
    message = message + "\n\tA cfg was incorrectly labeled invalid.\n";
  }

  return std::make_pair(passed,message);
}

typename InsideSpheresCollisionDetectionTest::TestResult
InsideSpheresCollisionDetectionTest::
MultipleCfgValidityTest() {
  bool passed = true;
  std::string message = "";

  auto robot1 = m_MPProblem->GetRobots()[0].get();
  auto robot2 = m_MPProblem->GetRobots()[1].get();

  Cfg cfg1(robot1);
  Cfg cfg2(robot2);

  // place configurations at same position
  cfg1[0] = 15;
  cfg1[1] = 0;
  cfg1[2] = 0;

  cfg2[0] = 15;
  cfg2[1] = 0;
  cfg2[2] = 0;

  cfg1.ConfigureRobot();
  cfg2.ConfigureRobot();
  bool invalid = this->CheckCollision(cfg1.GetMultiBody(), cfg2.GetMultiBody());
  if (!invalid) {
    passed = false;
    message = message + "\n\tA cfg was incorrectly labeled valid.\n";
  }

  // place one configuration away
  cfg1[0] = 0;
  cfg1[1] = -5;
  cfg1[2] = 0;

  cfg2[0] = 15;
  cfg2[1] = -5;
  cfg2[2] = 0;

  cfg1.ConfigureRobot();
  cfg2.ConfigureRobot();
  invalid = this->CheckCollision(cfg1.GetMultiBody(), cfg2.GetMultiBody());
  if (invalid) {
    passed = false;
    message = message + "\n\tA cfg was incorrectly labeled invalid.\n";
  }
  return std::make_pair(passed,message);
}


bool
InsideSpheresCollisionDetectionTest::
IsInCollision(
    const GMSPolyhedron& _polyhedron1, const mathtool::Transformation& _t1,
    const GMSPolyhedron& _polyhedron2, const mathtool::Transformation& _t2,
    CDInfo& _cdInfo) {
  InsideSpheres bs;
  return bs.IsInCollision(_polyhedron1, _t1, _polyhedron2, _t2, _cdInfo);

}

#endif
