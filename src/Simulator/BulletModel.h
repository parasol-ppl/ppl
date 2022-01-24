#ifndef PMPL_BULLET_MODEL_H_
#define PMPL_BULLET_MODEL_H_

#include "MPProblem/Robot/Control.h"

#include <functional>
#include <vector>

class btCollisionShape;
class btMultiBody;
class btMultiBodyConstraint;
class btMultiBodyDynamicsWorld;
class btMultiBodyLinkCollider;
class btVector3;

class Body;
class Cfg;
class MultiBody;
class Robot;


////////////////////////////////////////////////////////////////////////////////
/// A model of a multibody within a bullet simulation.
///
/// This structure contains all of the components which make up a bullet
/// multibody model, as well as functions for interfacing with it from PMPL.
///
/// @todo This model can currently only support robots with revolute joints.
///       Nonactuated and spherical joints don't work right. One problem is that
///       bullet indexes the number of joints while pmpl uses the number of dof,
///       so some of our joint loops don't line up. Another is that bullet only
///       supports 3-dof spherical joints with no way to constrain them.
////////////////////////////////////////////////////////////////////////////////
class BulletModel final {

  private:

    ///@name Internal State
    ///@{

    MultiBody* const m_pmplModel;      ///< The PMPL multibody.
    btMultiBody* const m_bulletModel;  ///< The bullet multibody.
    Robot* const m_robot;              ///< The robot, if any.

    /// The dynamics world to which this is attached.
    btMultiBodyDynamicsWorld* m_world{nullptr};

    // The extra junk which should have been created and managed within the
    // bullet multibody.
    std::vector<btMultiBodyLinkCollider*> m_colliders;
    std::vector<btMultiBodyConstraint*>   m_constraints;
    std::vector<btCollisionShape*>        m_collisionShapes;

    static constexpr bool s_debug{false};  ///< Show debug messages?

    ///@}
    ///@name Class Constants
    ///@{

    /// We will disable adjacent-link collision tests in the simulation.
    /// This is because we cannot perfectly convert angles between PMPL and
    /// Bullet, so it's possible that joints on tight-fitting parts would
    /// incur suprious collisions as small errors add up.
    static constexpr bool s_disableParentCollision{true};

    ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a bullet model of a PMPL multibody.
    /// @param _mb The PMPL multibody.
    /// @param _robot The robot, if any. Static multibodies will not have this.
    BulletModel(MultiBody* const _mb, Robot* const _robot = nullptr);

    ~BulletModel();

    /// Destroy and rebuild all internal bullet structures. This is intended for
    /// edit tools which need to update bullet models to match changes to the
    /// PMPL structures. Requires the model to already be part of a dynamics
    /// world, but must be done while the simulation isn't stepping.
    void Rebuild();

    ///@}
    ///@name Accessors
    ///@{

    /// Get the PMPL multibody.
    MultiBody* GetPMPLMultiBody() const noexcept;

    /// Get the bullet multibody.
    btMultiBody* GetBulletMultiBody() const noexcept;

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Get the current configuration of the bullet model. Does not work for
    /// static multibodies.
    Cfg GetState() const noexcept;

    /// Set the current configuration of the bullet model. Does not work for
    /// static multibodies.
    /// @param _c The configuration to set.
    void SetState(const Cfg& _c) noexcept;

    /// Execute a control on the bullet model for the next time step.
    /// @param _c The control to apply.
    void Execute(const Control& _c) noexcept;

    /// Execute a set of controls on the bullet model for the next time step.
    /// @param _c The controls to apply.
    void Execute(const ControlSet& _c) noexcept;

    /// Set all velocities on the bullet model to zero. This does NOT respect
    /// the mechanics models and should only be used for initialization and
    /// debugging.
    void ZeroVelocities() noexcept;

    ///@}
    ///@name Dynamics World Helpers
    ///@{
    /// These functions handle adding and removing this model from a bullet
    /// dynamics world.

    void AddToDynamicsWorld(btMultiBodyDynamicsWorld* const _world);

    void RemoveFromDynamicsWorld();

    ///@}

  private:

    ///@name Initialization
    ///@{
    /// These functions create and destroy bullet components. There are no
    /// guarantees on what will happen if you call them on a model attached to a
    /// dynamics world. If the simulation is running, race conditions will
    /// ensue. If it is paused, there may be strange effects or seg faults upon
    /// resume. The best practice is to only call these when the model is not
    /// attached to a simulation.

    /// Create the bullet structures from the PMPL multibody.
    void Initialize();

    /// Release all bullet structures.
    /// @param _delete Delete the bullet model? Should only be true when called
    ///                by the destructor.
    void Uninitialize(const bool _delete = false);

    ///@}
    ///@name Helpers
    ///@{

    /// Build the bullet structures from the PMPL multibody.
    /// @TODO This is a very large function. Break it up into smaller pieces for
    ///       adding the base and links.
    void Build();

    /// Build a set of bullet collision shapes for a pmpl MultiBody.
    /// @param _body The pmpl MultiBody.
    /// @return A set of bullet collision shapes.
    std::vector<btCollisionShape*> BuildCollisionShapes(
        const MultiBody* const _body);

    /// Build a bullet collision shape for a pmpl Body.
    /// @param _body The pmpl Body to use.
    /// @return A bullet collision shape.
    btCollisionShape* BuildCollisionShape(const Body* const _body);

    /// Initialze the bullet model transforms.
    void InitializeBulletTransforms();

    /// Convert a generalized force/velocity vector from the world frame to the
    /// bullet model's current local frame.
    /// @param _force The generalized force/velocity vector to convert.
    /// @return The representation of _force in the local frame.
    std::vector<double> WorldDirToLocal(std::vector<double>&& _force) const;

    /// Convert a generalized force/velocity vector from the bullet model's
    /// current local frame to the world frame.
    /// @param _force The generalized force/velocity vector to convert.
    /// @return The representation of _force in the world frame.
    std::vector<double> LocalDirToWorld(std::vector<double>&& _force) const;

    /// Helper for coordinate transforms.
    /// @param _force The generalized force/velocity vector to convert.
    /// @param _f The transformation to apply to the positional and rotational
    ///           components.
    void Transform(std::vector<double>& _force, std::function<void(btVector3&)>&& _f)
        const;

    ///@}

};

#endif
