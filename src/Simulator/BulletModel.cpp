#include "BulletModel.h"

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/GMSPolyhedron.h"
#include "MPProblem/Robot/Actuator.h"
#include "MPProblem/Robot/Robot.h"
#include "Simulator/Conversions.h"

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "ConvexDecomposition/cd_wavefront.h"

#include "nonstd/io.h"

#include <algorithm>

#define USE_BULLET_COLLIDERS // Comment this out to disable bullet collision.


/*------------------------------- Construction -------------------------------*/

BulletModel::
BulletModel(MultiBody* const _mb, Robot* const _robot)
  : m_pmplModel(_mb),
    m_bulletModel(new btMultiBody(0, 0, btVector3(), false, false)),
    m_robot(_robot)
{
  // Uninitialize first to destruct and clear the dummy m_bulletModel.
#ifdef DEBUG_BULLET_PROBLEMS
  _mb->m_bullet = m_bulletModel;
#endif
  Uninitialize();
  Initialize();
}


BulletModel::
~BulletModel() {
  Uninitialize(true);
}

/*------------------------------ Initialization ------------------------------*/

void
BulletModel::
Initialize() {
  // Store active model's current configuration and zero before rebuilding.
  std::vector<double> zeros(m_pmplModel->DOF(), 0),
                            currentCfg;
  if(m_robot) {
    currentCfg = m_pmplModel->GetCurrentCfg();
    m_pmplModel->Configure(zeros);
  }

  Build();

  if(m_robot) {
    m_pmplModel->Configure(currentCfg);
    Cfg c(m_robot);
    c.SetData(currentCfg);
    SetState(c);
  }

  ZeroVelocities();
}


void
BulletModel::
Uninitialize(const bool _delete) {
  if(m_world)
    RemoveFromDynamicsWorld();

  for(auto collider : m_colliders)
    delete collider;
  for(auto constraint : m_constraints)
    delete constraint;
  for(auto collision : m_collisionShapes)
    delete collision;

  m_colliders.clear();
  m_constraints.clear();
  m_collisionShapes.clear();

  // We must manually destruct the model rather than deleting it to avoid having
  // it move around in memory, which will disrupt the Robot objects. Overwrite
  // the space with zeros to ensure no stale data is left over.
  if(!_delete) {
    m_bulletModel->~btMultiBody();
    std::fill_n((char*)m_bulletModel, sizeof(btMultiBody), (char)0);
  }
  else
    delete m_bulletModel;
}


void
BulletModel::
Rebuild() {
  auto world = m_world;
  Uninitialize();
  Initialize();
  if(world)
    AddToDynamicsWorld(world);
}

/*-------------------------------- Accessors ---------------------------------*/

MultiBody*
BulletModel::
GetPMPLMultiBody() const noexcept {
  return m_pmplModel;
}


btMultiBody*
BulletModel::
GetBulletMultiBody() const noexcept {
  return m_bulletModel;
}

/*--------------------------- Simulation Interface ---------------------------*/

Cfg
BulletModel::
GetState() const noexcept {
  if(!m_robot)
    throw NotImplementedException(WHERE) << "Cannot get the state for a static "
                                         << "bullet model (requires Cfg).";

  Cfg out(m_robot);

  // Get the position. First get the base state.
  /// @todo This should probably move to Cfg.
  switch(m_pmplModel->GetBaseType()) {
    case Body::Type::Fixed:
      break;
    case Body::Type::Planar:
      {
        // Get the base transform.
        auto transform = ToPMPL(m_bulletModel->getBaseWorldTransform());
        auto cfg = transform.GetCfg();

        switch(m_pmplModel->GetBaseMovementType()) {
          case Body::MovementType::Rotational:
            out[2] = Normalize(cfg[5]);
          case Body::MovementType::Translational:
            out[0] = cfg[0];
            out[1] = cfg[1];
            break;
          default:
            throw RunTimeException(WHERE) << "Unrecognized base movement type.";
        }
      }
      break;
    case Body::Type::Volumetric:
      {
        // Get the base transform.
        auto transform = ToPMPL(m_bulletModel->getBaseWorldTransform());
        auto cfg = transform.GetCfg();

        switch(m_pmplModel->GetBaseMovementType()) {
          case Body::MovementType::Rotational:
            for(size_t i = 3; i < 6; ++i)
              out[i] = Normalize(cfg[i]);
          case Body::MovementType::Translational:
            for(size_t i = 0; i < 3; ++i)
              out[i] = cfg[i];
            break;
          default:
            throw RunTimeException(WHERE) << "Unrecognized base movement type.";
        }
      }
      break;
    default:
      throw RunTimeException(WHERE) << "Unrecognized base type.";
  }

  // Get joint positions. Note that bullet uses [ -PI : PI ].
  const size_t firstJointIndex = m_pmplModel->DOF() - m_pmplModel->JointDOF(),
               lastJointIndex  = m_pmplModel->DOF();

  for(size_t i = firstJointIndex, index = 0; i < lastJointIndex; ++i, ++index)
    out[i] = m_bulletModel->getJointPos(index) / PI;

  // Get the velocities.
  const bool getVelocity = m_robot->IsNonholonomic();
  if(getVelocity)
  {
    // Get base velocity.
    out.SetLinearVelocity(ToPMPL(m_bulletModel->getBaseVel()));
    out.SetAngularVelocity(ToPMPL(m_bulletModel->getBaseOmega()));

    // Get joint velocities. Note that bullet uses [ -PI : PI ].
    for(size_t i = firstJointIndex, index = 0; i < lastJointIndex; ++i, ++index)
      out.Velocity(i) = m_bulletModel->getJointVel(index) / PI;
  }

  return out;
}


void
BulletModel::
SetState(const Cfg& _c) noexcept {
  if(!m_robot)
    throw NotImplementedException(WHERE) << "Cannot set the state for a static "
                                         << "bullet model (requires Cfg).";

  /// @TODO Make a more efficient routine for this, that doesn't need to
  ///       configure the PMPL robot. Ideally we should grab the base's world
  ///       transform directly from the _c cfg.
  _c.ConfigureRobot();

  // Set the base transform.
  m_bulletModel->setBaseWorldTransform(
      ToBullet(m_pmplModel->GetBase()->GetWorldTransformation()));

  //auto dofs = m_pmplModel->GetCurrentCfg();

  // Set the joint DOFs. Note that bullet uses [ -PI : PI ].
  const size_t firstJointIndex = m_pmplModel->DOF() - m_pmplModel->JointDOF(),
               lastJointIndex  = m_pmplModel->DOF();
  for(size_t i = firstJointIndex, index = 0; i < lastJointIndex; ++i, ++index)
    m_bulletModel->setJointPos(index, _c[i] * PI);

  InitializeBulletTransforms();

  // Set velocities if needed.
  const bool hasVelocity = _c.GetRobot()->IsNonholonomic();
  if(hasVelocity) {
    // Set the base velocities.
    m_bulletModel->setBaseVel(ToBullet(_c.GetLinearVelocity()));
    m_bulletModel->setBaseOmega(ToBullet(_c.GetAngularVelocity()));

    // Set the joint velocities. Note that bullet uses [ -PI : PI ].
    for(size_t i = firstJointIndex, index = 0; i < lastJointIndex; ++i, ++index)
      m_bulletModel->setJointVel(index, _c.Velocity(i) * PI);
  }
  // If the configurations don't have velocity, we should zero it now.
  else
    ZeroVelocities();
}


void
BulletModel::
Execute(const Control& _c) noexcept {
  // We should only be executing controls on ROBOT models.
  if(!m_robot)
    throw RunTimeException(WHERE) << "Cannot execute controls on multibodies "
                                  << "which are not robots.";

  const std::vector<double> output = LocalDirToWorld(_c.GetOutput());
  auto iter = output.begin();

  btVector3 force(0, 0, 0), torque(0, 0, 0);

  // Set base force.
  const size_t numPos = m_pmplModel->PosDOF();
  for(size_t i = 0; i < numPos; ++i, ++iter)
    force[i] = *iter;

  // Set base torque.
  switch(m_pmplModel->OrientationDOF()) {
    case 1:
      // This is a planar rotational robot. We only want a torque in the Z
      // direction.
      torque[2] = *iter++;
      break;
    case 3:
      // This is a volumetric rotational robot. We need all three torque
      // directions.
      for(size_t i = 0; i < 3; ++i, ++iter)
        torque[i] = *iter;
      break;
    default:
      // This robot does not rotate.
      ;
  }

  const size_t numJoints = m_pmplModel->JointDOF();

  if(s_debug) {
    const std::string translationType =
        _c.actuator->GetDynamicsType() == Actuator::DynamicsType::Force
        ? "force" : "velocity";
    const std::string rotationType =
        _c.actuator->GetDynamicsType() == Actuator::DynamicsType::Force
        ? "torque" : "omega";

    std::cout << "\nBulletModel::Execute"
              << "\nControl: " << _c
              << "\nBase " << translationType << ": " << force
              << "\nBase " << rotationType << ": " << torque;

    auto diter = iter;
    for(size_t i = 0; i < numJoints; ++i, ++diter)
      std::cout << "\nJoint " << i << " " << rotationType << ": " << *diter * PI;

    std::cout << std::endl;
  }

  switch(_c.actuator->GetDynamicsType()) {
    case Actuator::DynamicsType::Force:
      m_bulletModel->addBaseForce(force);
      m_bulletModel->addBaseTorque(torque);

      // Add forces to the joints. Bullet uses [ -PI : PI ].
      for(size_t i = 0; i < numJoints; ++i, ++iter)
        m_bulletModel->addJointTorque(i, *iter * PI);
      break;
    case Actuator::DynamicsType::Velocity:
      m_bulletModel->setBaseVel(force);
      m_bulletModel->setBaseOmega(torque);

      // Add velocities to the joints. Bullet uses [ -PI : PI ].
      for(size_t i = 0; i < numJoints; ++i, ++iter)
        m_bulletModel->setJointVel(i, *iter * PI);
      break;
    default:;
  }
}


void
BulletModel::
Execute(const ControlSet& _c) noexcept {
  for(const auto& control : _c)
    Execute(control);
}


void
BulletModel::
ZeroVelocities() noexcept {
  // Zero the base velocity.
  m_bulletModel->setBaseVel({0,0,0});
  m_bulletModel->setBaseOmega({0,0,0});

  // Zero the joint velocities.
  for(int i = 0; i < m_bulletModel->getNumLinks(); i++) {
    const auto& jointType = m_bulletModel->getLink(i).m_jointType;

    // If it's a spherical (2 dof) joint, then we must use the other version of
    // setting the link velocity dofs for each value of desired velocity.
    if(jointType == btMultibodyLink::eFeatherstoneJointType::eSpherical) {
      static btScalar temp[] = {0,0,0};
      m_bulletModel->setJointVelMultiDof(i, temp);
    }
    // Do nothing if the joint was a non-actuated joint.
    else if(jointType != btMultibodyLink::eFeatherstoneJointType::eFixed) {
      m_bulletModel->setJointVel(i, 0);
    }
  }
}

/*------------------------------ Dynamics World ------------------------------*/

void
BulletModel::
AddToDynamicsWorld(btMultiBodyDynamicsWorld* const _world) {
  if(m_world)
    throw RunTimeException(WHERE) << "Cannot attach a model to more than one "
                                  << "dynamics world.";
  m_world = _world;

  // Determine the collision filter group and mask.
  // See BulletCollision/BroadphaseCollision/btBroadphaseProxy.h
  // and BulletCollision/CollisionDispatch/btCollisionWorld.h
  // for more info on these.
  auto group = m_pmplModel->IsActive()
      ? btBroadphaseProxy::DefaultFilter
      : btBroadphaseProxy::StaticFilter;
  auto mask = m_pmplModel->IsActive()
      ? btBroadphaseProxy::AllFilter
      : btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter;
        // This is logically equivalent to "All BUT StaticFilter":

  // Add the multibody.
  m_world->addMultiBody(m_bulletModel);

  // Add the colliders.
  for(auto* collider : m_colliders)
    m_world->addCollisionObject(collider, group, mask);

  // Add the constraints.
  for(auto* constraint : m_constraints)
    m_world->addMultiBodyConstraint(constraint);

  /// @TODO Need to set friction properly.
  //col->setFriction(m_problem->GetEnvironment()->GetFrictionCoefficient());
}


void
BulletModel::
RemoveFromDynamicsWorld() {
  if(!m_world)
    throw RunTimeException(WHERE) << "Cannot remove a model which is not in a "
                                  << "dynamics world.";

  // Remove the multibody.
  m_world->removeMultiBody(m_bulletModel);

  // Remove the colliders.
  for(auto* collider : m_colliders)
    m_world->removeCollisionObject(collider);

  // Remove the constraints.
  for(auto* constraint : m_constraints)
    m_world->removeMultiBodyConstraint(constraint);

  m_world = nullptr;
}

/*------------------------------ Helpers -------------------------------------*/

void
BulletModel::
Build() {
  // Create collision shapes for each body in this multibody.
  m_collisionShapes = BuildCollisionShapes(m_pmplModel);

  const bool  isDynamic = m_pmplModel->IsActive();
  const auto& joints    = m_pmplModel->GetJoints();

  if(s_debug)
    std::cout << "BulletModel::AddObject: Creating multibody"
              << "\n\t" << m_collisionShapes.size() << " bodies"
              << "\n\t" << joints.size() << " joints"
              << "\n\t" << (isDynamic ? "" : "not ") << "dynamic"
              << std::endl;

  // First check that number of elements of each vector match. We need one less
  // joint as the base is not considered a joint.
  if(m_collisionShapes.size() - 1 != joints.size())
    throw RunTimeException(WHERE) << "Expected the number of shapes ("
                                  << m_collisionShapes.size()
                                  << ") to be equal to the number of joints "
                                  << "plus 1 (" << joints.size() + 1 << ").";

  // Create the base and btMultiBody object.
  {
    // Compute inertia for base.
    const double baseMass = m_pmplModel->IsActive()
                          ? m_pmplModel->GetBase()->GetMass() : 0;
    btVector3 baseInertia(0, 0, 0);
    if(isDynamic)
      m_collisionShapes[0]->calculateLocalInertia(baseMass, baseInertia);

    // Compute other properties that apply to the whole multibody object.
    const int  numLinks  = m_collisionShapes.size() - 1;
    const bool fixedBase = !isDynamic;
    const bool canSleep  = false; /// @TODO Not sure what this means.

    // Make the btMultiBody in the exact same address with placement new.
    new(m_bulletModel) btMultiBody(numLinks, baseMass, baseInertia, fixedBase,
        canSleep);
    btTransform baseTransform = ToBullet(
        m_pmplModel->GetBase()->GetWorldTransformation());
    m_bulletModel->setBaseWorldTransform(baseTransform);

#ifdef USE_BULLET_COLLIDERS
    // The multibody on its own doesn't process collisions. Create a collider
    // object for each component to handle this. The base has id -1.
    btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(m_bulletModel, -1);
    col->setCollisionShape(m_collisionShapes[0]);
    col->setWorldTransform(baseTransform);
    m_bulletModel->setBaseCollider(col);

    // Attach the collider to this model.
    m_colliders.push_back(col);
#endif

    if(s_debug)
      std::cout << "Added base with mass " << baseMass << "."
                << std::endl;
  }

  // Create the links.
  for(size_t i = 0; i < joints.size(); ++i) {
    // Get the indices for this link and its parent in both PMPL and bullet
    // representation (bullet uses [0:n-1] where PMPL uses [1:n]).
    const int parentPmplIndex = joints[i]->GetPreviousBodyIndex(),
              linkPmplIndex   = joints[i]->GetNextBodyIndex(),
              parentIndex     = parentPmplIndex - 1,
              linkIndex       = linkPmplIndex - 1;

    if(s_debug)
      std::cout << "\nAdding joint " << i << " connecting bodies "
                << parentPmplIndex << " and " << linkPmplIndex << "."
                << std::endl;

    // Get the mass and moment of inertia for the link:
    /// @TODO Respect PMPL's moment instead of recomputing it here.
    /// @TODO Specify center of mass?
    const btScalar linkMass = joints[i]->GetNextBody()->GetMass();
    btVector3 linkInertia(0, 0, 0);
    if(isDynamic)
      m_collisionShapes[linkPmplIndex]->calculateLocalInertia(linkMass,
                                                              linkInertia);

    // Set up the connection between this link and its parent.

    // Get the joint transformations.
    Transformation parentToActuation = joints[i]->GetTransformationToDHFrame(),
                   actuation = joints[i]->GetDHParameters().GetTransformation(),
                   actuationToLink = joints[i]->GetTransformationToBody2(),
                   parentToLink = parentToActuation * actuation * actuationToLink;

    if(s_debug)
      std::cout << "PMPL Transforms:"
                << "\nparentToActuation: " << parentToActuation
                << "\nactuation: " << actuation
                << "\nactuationToLink: " << actuationToLink
                << "\nparentToLink: " << parentToLink
                << std::endl;

    // Get the parent to link rotation for the 0 configuration, in the PARENT
    // frame.
    btQuaternion parentToLinkRotationInParentFrame;
    {
      mathtool::Quaternion temp;
      mathtool::Matrix3x3 check;
      mathtool::convertFromMatrix(temp, parentToLink.rotation().matrix());

      parentToLinkRotationInParentFrame = ToBullet(temp);

      if(s_debug) {
        mathtool::convertFromQuaternion(check, temp);
        const bool reconverted = check == parentToLink.rotation().matrix(),
                   unitQuat    = temp.norm() == 1;
        if(!reconverted)
          std::cout << "Mat <-> Quat conversion is bad."
                    << "\nOriginal mat: " << parentToLink.rotation().matrix()
                    << "\nReconverted:  " << check
                    << std::endl;
        if(!unitQuat)
          std::cout << "Quat is not unit length (" << temp.norm() << ")"
                    << "\nQ: " << temp
                    << std::endl;

        std::cout << "\n\nparentToLinkRotationInParentFrame: "
                  << "\nPMPL mat: " << parentToLink.rotation().matrix()
                  << "\nPMPL quat: " << temp
                  << "\nBullet quat: " << parentToLinkRotationInParentFrame
                  << std::endl;
      }
    }

    // Get the parent to actuation frame translation, in the PARENT frame.
    btVector3 parentToActuationTranslationInParentFrame = ToBullet(
        parentToActuation.translation());

    // Get the actuation to link frame translation, in the LINK frame.
    btVector3 actuationToLinkTranslationInLinkFrame = ToBullet(
        -actuationToLink.rotation() * actuationToLink.translation());

    switch(joints[i]->GetConnectionType())
    {
      case Connection::JointType::Revolute:
      {
        // For a revolute link, the last piece now is to set the jointAxis in
        // the link's frame. For revolute joints this is the Z-direction in the
        // actuation frame, so we just need to rotate it to the link frame.
        const btVector3 jointAxisInLinkFrame = ToBullet(
            actuationToLink.rotation() * Vector3d(0, 0, -1));

        m_bulletModel->setupRevolute(linkIndex, linkMass, linkInertia,
            parentIndex,
            parentToLinkRotationInParentFrame,
            jointAxisInLinkFrame,
            parentToActuationTranslationInParentFrame,
            actuationToLinkTranslationInLinkFrame,
            s_disableParentCollision);

        // Add joint limits as a bullet constraint.
        const Range<double> range = joints[i]->GetJointRange(0);
        btMultiBodyConstraint* con = new btMultiBodyJointLimitConstraint(
            m_bulletModel, linkIndex, range.min * PI, range.max * PI);
        m_constraints.push_back(con);

        if(s_debug)
          std::cout << "\t\tAdded revolute joint " << linkIndex
                    << "with joint range ["
                    << std::setprecision(3) << range.min * PI << " : "
                    << std::setprecision(3) << range.max * PI << "]."
                    << std::endl;
        break;
      }
      case Connection::JointType::Spherical:
      {
        throw NotImplementedException(WHERE) << "Bullet uses three-dof spherical "
                                             << "joints, and we do not have "
                                             << "support for handling that ("
                                             << "original impl assumed 2 dof)."
                                             << std::endl;

        /// @TODO support spherical constraints. As per answers here:
        /// http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=10780
        /// Until then, warn the user that their constraints won't be respected.
        std::cerr << "Bullet supports spherical joints, but not constraints for "
                  << "them. Any constraints on this joint will be ignored by the "
                  << "physics engine!"
                  << std::endl;

        m_bulletModel->setupSpherical(linkIndex, linkMass, linkInertia,
            parentIndex,
            parentToLinkRotationInParentFrame,
            parentToActuationTranslationInParentFrame,
            actuationToLinkTranslationInLinkFrame,
            s_disableParentCollision);

        if(s_debug)
          std::cout << "\t\tAdded spherical joint " << linkIndex << "."
                    << std::endl;
        break;
      }
      case Connection::JointType::NonActuated:
      {
        // Since this is a fixed joint, there is no need for joint constraints.
        m_bulletModel->setupFixed(linkIndex, linkMass, linkInertia, parentIndex,
            parentToLinkRotationInParentFrame,
            parentToActuationTranslationInParentFrame,
            actuationToLinkTranslationInLinkFrame,
            s_disableParentCollision);

        if(s_debug)
          std::cout << "\t\tAdded fixed joint " << linkIndex << "."
                    << std::endl;
        break;
      }
      default:
        throw RunTimeException(WHERE) << "Unsupported joint type.";
    }

#ifdef USE_BULLET_COLLIDERS
    // Create a collider for the link.
    // See examples/MultiBody/Pendulum.cpp for related example code.
    btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(m_bulletModel,
        linkIndex);
    col->setCollisionShape(m_collisionShapes[linkIndex + 1]);
    m_bulletModel->getLink(linkIndex).m_collider = col;

    m_colliders.push_back(col);
#endif
  }

  // Finalize the multibody.
  m_bulletModel->finalizeMultiDof();

  InitializeBulletTransforms();
}


std::vector<btCollisionShape*>
BulletModel::
BuildCollisionShapes(const MultiBody* const _multibody) {
  std::vector<btCollisionShape*> shapes;

  // Make collision shapes for each body.
  for(size_t i = 0; i < _multibody->GetNumBodies(); i++)
    shapes.push_back(BuildCollisionShape(_multibody->GetBody(i)));

  return shapes;
}


btCollisionShape*
BulletModel::
BuildCollisionShape(const Body* const _body) {
  // Get the body's polyhedron, vertices, and facets.
  const GMSPolyhedron& poly = _body->GetPolyhedron();
  const auto& vertices = poly.GetVertexList();
  const auto& facets = poly.GetPolygonList();

  // Initialize a btTriangleMesh with enough space for our model.
  btTriangleMesh* mesh = new btTriangleMesh();
  mesh->preallocateVertices(vertices.size());
  mesh->preallocateIndices(facets.size());

  // Add vertices, don't remove duplicates (because PMPL doesn't, and this will
  // mess up the facet indexes).
  for(const auto& v : vertices)
    mesh->findOrAddVertex(btVector3(v[0], v[1], v[2]), false);

  // Add facets.
  for(const auto& f : facets)
    mesh->addTriangleIndices(f[0], f[1], f[2]);

  // Make the collision shape from the mesh.
  auto shape = new btGImpactMeshShape(mesh);
  shape->updateBound();
  shape->setMargin(0); // Do not use any collision 'margin' around this obstacle.

  return shape;
}


void
BulletModel::
InitializeBulletTransforms() {
  // From bullet example Pendulum.cpp.

  // Initialize the internal link transforms.
  btAlignedObjectArray<btQuaternion> scratch_q;
  btAlignedObjectArray<btVector3> scratch_v;
  m_bulletModel->forwardKinematics(scratch_q, scratch_v);

  // Initialize the internal collider transforms.
  btAlignedObjectArray<btQuaternion> world_to_local;
  btAlignedObjectArray<btVector3> local_origin;
  m_bulletModel->updateCollisionObjectWorldTransforms(world_to_local,
      local_origin);
}


std::vector<double>
BulletModel::
WorldDirToLocal(std::vector<double>&& _force) const {
  Transform(_force,
      [this](btVector3& _v) {_v = this->m_bulletModel->worldDirToLocal(-1, _v);});
  return _force;
}


std::vector<double>
BulletModel::
LocalDirToWorld(std::vector<double>&& _force) const {
  Transform(_force,
      [this](btVector3& _v) {_v = this->m_bulletModel->localDirToWorld(-1, _v);});
  return _force;
}


void
BulletModel::
Transform(std::vector<double>& _force, std::function<void(btVector3&)>&& _f)
    const {
  auto read = _force.begin(),
       write = _force.begin();

  btVector3 scratch(0, 0, 0);

  // Convert base force.
  const size_t numPos = m_pmplModel->PosDOF();
  for(size_t i = 0; i < numPos; ++i, ++read)
    scratch[i] = *read;

  _f(scratch);

  for(size_t i = 0; i < numPos; ++i, ++write)
    *write = scratch[i];


  // Convert base torque.
  scratch.setValue(0, 0, 0);

  switch(m_pmplModel->OrientationDOF()) {
    case 3:
      // This is a volumetric rotational robot. We need all three torque
      // directions.
      for(int i = 0; i < 2; ++i, ++read)
        scratch[i] = *read;

      _f(scratch);

      for(int i = 0; i < 2; ++i, ++write)
        *write = scratch[i];

      break;
    case 1:
      // This is a planar rotational robot. We only want a torque in the Z
      // direction.
      /// @TODO In simulation, it is possible that a planar rotational robot
      /// will collide with something and be knocked off its plane. If this
      /// happens, the transformations between our 3-dof Cfgs and the bullet
      /// world will be thrown off with no way to recover. The only resolution is
      /// to treat these as 6-dof robots in simulation.
      scratch[2] = *read;
      _f(scratch);
      *write = scratch[2];
      break;
    default:
      // This robot does not rotate.
      ;
  }
}

/*----------------------------------------------------------------------------*/
