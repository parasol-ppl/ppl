#ifndef PMPL_BULLET_ENGINE_H_
#define PMPL_BULLET_ENGINE_H_

#include "btBulletDynamicsCommon.h"

#include "glutils/gltraits.h"

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

// Bullet forward-declarations.
class btMultiBodyDynamicsWorld;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btMultiBodyConstraintSolver;
class btMultiBody;

// PMPL forward-declarations.
class Body;
class BulletModel;
class Connection;
class MultiBody;
class MPProblem;
class Robot;


////////////////////////////////////////////////////////////////////////////////
/// Encapsulates the details of creating a bullet simulation.
////////////////////////////////////////////////////////////////////////////////
class BulletEngine final {

  ///@name Bullet Components
  ///@{

  /// The bullet world.
  btMultiBodyDynamicsWorld*        m_dynamicsWorld{nullptr};

  // The collision detection components.
  btDefaultCollisionConfiguration* m_collisionConfiguration{nullptr};
  btCollisionDispatcher*           m_dispatcher{nullptr};
  btBroadphaseInterface*           m_broadphase{nullptr};
  btMultiBodyConstraintSolver*     m_solver{nullptr};

  ///@}
  ///@name Call-back Functions
  ///@}

  /// A call-back function can be used to modify the physics engine behavior for
  /// a dynamic object. It will be executed by the bullet engine after each
  /// internal timestep.
  typedef std::function<void(void)> CallbackFunction;

  /// A set of call-back functions that are used for this engine object.
  typedef std::vector<CallbackFunction> CallbackSet;

  /// The set of all call-back functions for this simulation.
  CallbackSet m_callbacks;

  /// A map from all dynamics worlds (one per engine object) to their call-back
  /// sets. This is needed to get bullet to call the appropriate call-back set
  /// from a single, universal function.
  static std::map<btDynamicsWorld*, CallbackSet&> s_callbackMap;

  ///@}
  ///@name Other Internal State
  ///@{

  MPProblem* const m_problem; ///< A pointer to the problem being simulated.

  /// A map from PMPL to bullet models that exist in this simulation.
  std::map<MultiBody*, std::unique_ptr<BulletModel>> m_models;

  /// A queue of objects which are needing rebuild.
  std::queue<MultiBody*> m_rebuildQueue;

  std::mutex m_lock;    ///< Lock the engine while objects are edited.

  bool m_debug{false};  ///< Show debug messages?

  ///@}

  public:

    ///@name Construction
    ///@{

    BulletEngine(MPProblem* const _problem);

    ~BulletEngine();

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Step the simulation forward.
    /// @param _timestep The total length of time to advance the simulation.
    void Step(const btScalar _timestep);

    ///@}
    ///@name Transform Access
    ///@{

    /// Get the current transform for a given object.
    /// @param _m The object pointer.
    /// @param _j The component index (0 for base by default).
    /// @return An OpenGL transform matrix describing object _i's current
    ///         position and orientation.
    glutils::transform GetObjectTransform(MultiBody* const _m,
        const size_t _j = 0) const;

    ///@}
    ///@name Modifiers
    ///@{

    /// Add a robot to the simulation.
    /// @param _robot The robot to add.
    /// @return The bullet representation of _robot.
    /// @todo We need a way to set initial velocities for the robot.
    BulletModel* AddRobot(Robot* const _robot);

    /// Add a non-robot multibody to the simulation.
    /// @param _m The multibody to add.
    /// @return The bullet representation of _m.
    BulletModel* AddMultiBody(MultiBody* const _m);

    /// Set the gravity in the world (this will also set it for all bodies)
    /// @param _gravityVec Is simply the 3-vector representing (x,y,z) gravity
    void SetGravity(const btVector3& _gravityVec);

    /// Request that the physics model of an object be rebuilt. The rebuild will
    /// occur on the next call to Step.
    /// @param _m The object to rebuild.
    void RebuildObject(MultiBody* const _m);

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Add a multibody object to the world.
    /// @param _m The multibody to add.
    /// @param _robot The robot associated with _m, if any (should only be used
    ///               by AddRobot).
    /// @return The bullet representation of _m.
    BulletModel* AddObject(MultiBody* const _m, Robot* const _robot = nullptr);

    /// Rebuild the bullet models for which a rebuild was requested.
    /// @WARNING The simulation must be halted while this is going on. Also, if
    ///          you do this for a robot then its internal MicroSimulator will
    ///          not receive any updates that may have been made to the global
    ///          bullet model. This function does not lock as it should be
    ///          called only within Step().
    void RebuildObjects();

    ///@}
    ///@name Call-back Function Interface
    ///@{

    /// Create a call-back to make a multibody behave like a car with perfect
    /// friction.
    /// @param _model The multibody to affect.
    void CreateCarlikeCallback(btMultiBody* const _model);

    /// Execute all call-backs for a given dynamics world.
    /// @param _world The dynamics world.
    /// @param _timeStep The length of the last time step.
    static void ExecuteCallbacks(btDynamicsWorld* _world, btScalar _timeStep);

    ///@}
    ///@name Deleted Functions
    ///@{

    BulletEngine(const BulletEngine&) = delete;
    BulletEngine& operator=(const BulletEngine&) = delete;

    BulletEngine(BulletEngine&&) = delete;
    BulletEngine& operator=(BulletEngine&&) = delete;

    ///@}

};

#endif
