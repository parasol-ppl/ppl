#include "BulletEngine.h"

#include "BulletModel.h"
#include "Conversions.h"
#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/GMSPolyhedron.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "ConvexDecomposition/cd_wavefront.h"

#include "nonstd/runtime.h"


/*--------------------------- Static Initialization --------------------------*/

std::map<btDynamicsWorld*, BulletEngine::CallbackSet&>
BulletEngine::s_callbackMap;

/*------------------------------ Construction --------------------------------*/

BulletEngine::
BulletEngine(MPProblem* const _problem) : m_problem(_problem) {
  // Create the bullet objects needed for a dynamic rigid body simulation.
  m_collisionConfiguration = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_broadphase = new btDbvtBroadphase();
  m_solver = new btMultiBodyConstraintSolver();

  m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher,
      m_broadphase, m_solver, m_collisionConfiguration);

  // This is needed to get gimpact shapes to respond to collisions.
  btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

  // Set the gravity in our world, based off of MPProblem:
  m_dynamicsWorld->setGravity(ToBullet(
                              m_problem->GetEnvironment()->GetGravity()));

  // Add this engine's call-back set to the call-back map.
  s_callbackMap.emplace(m_dynamicsWorld, m_callbacks);
  m_dynamicsWorld->setInternalTickCallback(ExecuteCallbacks);
}


BulletEngine::
~BulletEngine() {
  // Acquire the lock for the remainder of object life.
  std::lock_guard<std::mutex> lock(m_lock);

  // Clear the models before the dynamicsWorld as the former need the latter to
  // tear down properly.
  m_models.clear();

  // Delete the rigid bodies in the bullet dynamics world.
  for(int i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
    btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);

    // If the body has a motion state, delete that too.
    if(body && body->getMotionState())
      delete body->getMotionState();

    m_dynamicsWorld->removeCollisionObject(obj);
    delete obj;
  }

  // Remove this engine's dynamics world and call-back set from the call-back
  // map.
  s_callbackMap.erase(m_dynamicsWorld);

  // Delete the remaining bullet objects.
  delete m_dynamicsWorld;
  delete m_solver;
  delete m_broadphase;
  delete m_dispatcher;
  delete m_collisionConfiguration;
}

/*---------------------------- Simulation Interface --------------------------*/

void
BulletEngine::
Step(const btScalar _timestep) {
  std::lock_guard<std::mutex> lock(m_lock);

  // Rebuild any objects which need it.
  RebuildObjects();

  // This doesn't seem to be required in the examples, testing without it for
  // now.
  //m_dynamicsWorld->updateAabbs();
  //m_dynamicsWorld->computeOverlappingPairs();

  // Advance the simulation by '_timestep' units using up to 'maxSubSteps' sub
  // steps of length 'resolution'.
  const btScalar resolution = m_problem->GetEnvironment()->GetTimeRes();
  const int maxSubSteps = std::ceil(_timestep / resolution);

  m_dynamicsWorld->stepSimulation(_timestep, maxSubSteps, resolution);
}

/*----------------------------- Transform Access -----------------------------*/

glutils::transform
BulletEngine::
GetObjectTransform(MultiBody* const _m, const size_t _j) const {
  // Check for out-of-range access.
  if(!m_models.count(_m))
    throw RunTimeException(WHERE) << "Requested model does not exist.";

  btMultiBody* mb = m_models.at(_m)->GetBulletMultiBody();

  std::array<double, 16> buffer;
  if(_j == 0)
    mb->getBaseWorldTransform().getOpenGLMatrix(buffer.data());
  else
    mb->getLink(int(_j-1)).m_cachedWorldTransform.getOpenGLMatrix(buffer.data());

  /// @TODO Fix this to avoid the extra copy.
  glutils::transform t;
  std::copy(buffer.begin(), buffer.end(), t.begin());

  return t;
}

/*----------------------------- Modifiers ------------------------------------*/

BulletModel*
BulletEngine::
AddRobot(Robot* const _robot) {
  return AddObject(_robot->GetMultiBody(), _robot);
}


BulletModel*
BulletEngine::
AddMultiBody(MultiBody* const _m) {
  return AddObject(_m);
}


void
BulletEngine::
SetGravity(const btVector3& _gravity) {
  std::lock_guard<std::mutex> lock(m_lock);

  m_dynamicsWorld->setGravity(_gravity);
}


void
BulletEngine::
RebuildObject(MultiBody* const _m) {
  std::lock_guard<std::mutex> lock(m_lock);

  m_rebuildQueue.push(_m);
}

/*--------------------------------- Helpers ----------------------------------*/

BulletModel*
BulletEngine::
AddObject(MultiBody* const _m, Robot* const _robot) {
  std::lock_guard<std::mutex> lock(m_lock);

  // Check that the robot's multibody matches _m.
  if(_robot and _robot->GetMultiBody() != _m)
    throw RunTimeException(WHERE) << "Mismatch between multibody and robot.";

  // Check that we haven't already added this model.
  if(m_models.count(_m))
    throw RunTimeException(WHERE) << "Cannot add the same MultiBody twice.";

  auto& model = m_models[_m];
  model.reset(new BulletModel(_m, _robot));
  model->AddToDynamicsWorld(m_dynamicsWorld);

  // If this is a car-like robot, create the necessary callbacks.
  if(_robot and _robot->IsCarlike())
    CreateCarlikeCallback(model->GetBulletMultiBody());

  return model.get();
}


void
BulletEngine::
RebuildObjects() {
  while(!m_rebuildQueue.empty()) {
    // Dequeue the next multibody.
    MultiBody* const m = m_rebuildQueue.front();
    m_rebuildQueue.pop();

    // Check that this model exists.
    auto iter = m_models.find(m);
    if(iter == m_models.end())
      throw RunTimeException(WHERE) << "MultiBody not found.";

    // Rebuild the model.
    iter->second->Rebuild();
  }
}

/*--------------------------- Callback Functions -----------------------------*/

void
BulletEngine::
CreateCarlikeCallback(btMultiBody* const _model) {
  // Create a call-back function to make _model appear car-like. For now we will
  // always assume that _model's forward direction is (1, 0, 0) in its local
  // frame.
  CallbackFunction f = [_model]() {
    const btVector3 velocity = _model->getBaseVel();
    const btVector3 heading = _model->localDirToWorld(-1, {1, 0, 0});
    const btScalar sign = velocity * heading < 0 ? -1 : 1;
    _model->setBaseVel(heading * sign * velocity.length());
  };

  // Add this to the set of callbacks for this simulation engine.
  m_callbacks.push_back(f);
}


void
BulletEngine::
ExecuteCallbacks(btDynamicsWorld* _world, btScalar _timeStep) {
  // Get the call-back set associated with this dynamics world.
  CallbackSet& callbacks = s_callbackMap.at(_world);

  // Execute each call-back in the set.
  for(auto& callback : callbacks)
    callback();
}

/*----------------------------------------------------------------------------*/
