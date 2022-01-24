#ifndef PMPL_ROBOT_H_
#define PMPL_ROBOT_H_

#include "Control.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

class Actuator;
class Agent;
class Battery;
class Body;
class Boundary;
class BulletModel;
class Cfg;
class ControllerMethod;
class CSpaceBoundingBox;
class MatlabMicroSimulator;
class MicroSimulator;
class MPProblem;
class MultiBody;
class RobotCommandQueue;
class XMLNode;
class StateEstimator;


////////////////////////////////////////////////////////////////////////////////
/// Complete representation of a robot.
///
/// @details A robot has many components, including:
///   @arg MultiBody: The robot's physical geometry.
///   @arg Agent: The robot's high-level decision-making algorithm. Determines
///               what actions the robot should take to complete its task. Used
///               only in simulations.
///   @arg Actuators: The robot's motors/effectors. Translates control commands
///                   into generalized forces.
///   @arg Controller: The robot's low-level controller, which determines what
///                    control should be applied to move from point to point.
///   @arg BulletModel: Simulation model of the robot. Represents the robot in
///                     the bullet world.
///   @arg MicroSimulator: Isolated simulation of this robot for testing control
///                        applications.
///   @arg CommandQueue: Controls a set of hardware for this robot.
///   @arg StateEstimator: Object for integrating sensor data with commands to
///                        localize a hardware robot.
///
/// @todo Come up with a nice way to support both real and emulated hardware.
////////////////////////////////////////////////////////////////////////////////
class Robot final {

  ///@name Internal State
  ///@{

  MPProblem* m_problem{nullptr};              ///< The owning problem object.

  std::unique_ptr<MultiBody> m_multibody;  ///< Robot's geometric representation.

  /// Actuators, mapped by label.
  std::unordered_map<std::string, std::unique_ptr<Actuator>> m_actuators;
  std::unique_ptr<CSpaceBoundingBox> m_cspace;    ///< The robot's c-space.
  std::unique_ptr<CSpaceBoundingBox> m_vspace;    ///< The robot's velocity space.
  std::unique_ptr<Agent> m_agent;                 ///< High-level decision agent.
  std::unique_ptr<ControllerMethod> m_controller; ///< Low-level controller.
  std::unique_ptr<MicroSimulator> m_simulator;    ///< Internal simulator.
#ifdef PMPL_USE_MATLAB
  std::unique_ptr<MatlabMicroSimulator> m_matlabSimulator; ///< Matlab internal simulator.
#endif
  BulletModel* m_bulletModel{nullptr};            ///< The bullet simulation model.

  std::unique_ptr<RobotCommandQueue> m_hardware;    ///< Hardware command queue.
  std::unique_ptr<StateEstimator> m_stateEstimator; ///< The localization object.
  std::unique_ptr<Battery> m_battery;               ///< An emulated battery.

  std::string m_label;             ///< The robot's unique label.
  bool m_virtual{false};           ///< Is this an imaginary robot?
  bool m_nonholonomic{false};      ///< Is the robot nonholonomic?
  bool m_carlike{false};           ///< Is the robot car-like?
  double m_maxLinearVelocity{10};  ///< Max linear velocity.
  double m_maxAngularVelocity{1};  ///< Max angular velocity.
  std::string m_capability;        ///< The terrain label that the robot can use.
  bool m_fixed{false};             ///< Does the robot have a fixed base
  bool m_manipulator{false};       ///< Is the robot a manipulator?
  std::string m_defaultStrategyLabel; ///< The robot's default MP Strategy.

  //////////////////////////////////////////////////////////////////////////////
  /// Description of a Robot's end-effector for grasping problems.
  /// @todo Move to its own file in Geometry/Bodies. Have the MultiBody parse
  ///       and own these objects (possibly many per robot). Devise some
  ///       'default' way to compute the contact point if it is not specified
  ///       (like ray shoot from bbx center) so that we can easily parse
  ///       problems without grasping using the same code.
  /// @todo Generalize this abstraction to support multiple types of grasp.
  ///       Implement this (point-grasping) as a concrete instantiation.
  //////////////////////////////////////////////////////////////////////////////
  struct EndEffector {
    EndEffector() {}
    EndEffector(XMLNode& _node, MultiBody* const _mb);

    /// The body pointer in the robot's multibody of this end effector.
    Body* effectorBody{nullptr};

    /// The vertex of the end effector that establishes point contact.
    mathtool::Vector3d contactPoint;

    /// The direction from the center of the effector's bounding box to the
    /// contact point. NOT normalized to a unit magnitude.
    mathtool::Vector3d centerToContactDir;
  };

  EndEffector m_endEffector;  ///< End-effector information.

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a robot from an XML node.
    /// @param _p The owning MPProblem.
    /// @param _node The XML node to parse.
    Robot(MPProblem* const _p, XMLNode& _node);

    /// Construct a robot from a multibody.
    /// @param _p The owning MPProblem.
    /// @param _label The unique label for this robot.
    Robot(MPProblem* const _p, std::unique_ptr<MultiBody>&& _mb,
          const std::string& _label);

    /// Copy a robot to another MPProblem.
    /// @param _p The destination MPProblem.
    /// @param _r The source robot to copy.
    Robot(MPProblem* const _p, const Robot& _r);

    ~Robot() noexcept;

    ///@}
    ///@name Disabled Functions
    ///@{
    /// Regular copy/move is not allowed because we require a permanent
    /// MPProblem for the robot to belong to.
    /// Assignment is disabled because we should never need to re-assign entire
    /// robots. Destruct the old one and create a new one instead.

    Robot(const Robot&) = delete;
    Robot(Robot&&) = delete;

    Robot& operator=(const Robot&) = delete;
    Robot& operator=(Robot&&) = delete;

    ///@}

  protected:

    ///@name I/O
    ///@{

    /// Parse an XML robot file.
    /// @param _filename The file name.
    void ReadXMLFile(const std::string& _filename);

    /// Parse an XML robot node.
    /// @param _node The SML node.
    void ReadXMLNode(XMLNode& _node);

    /// Parse multibody information from robot's XML file.
    /// @param _node The XML node to parse
    void ReadMultiBodyXML(XMLNode& _node);

    /// Parse a multibody file describing this robot.
    /// @param _filename The file name.
    void ReadMultibodyFile(const std::string& _filename);

    ///@}

  public:

    ///@name Planning Interface
    ///@{

    /// Compute the configuration and velocity spaces for this robot.
    void InitializePlanningSpaces();

    /// Get the configuration space boundary for this robot.
    const CSpaceBoundingBox* GetCSpace() const noexcept;

    /// Get the velocity space boundary for this robot.
    const CSpaceBoundingBox* GetVSpace() const noexcept;

    /// Get the owning MPProblem.
    MPProblem* GetMPProblem() const noexcept;

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Execute a simulation step: update the percept model, have the agent make
    /// a decision, and send the resulting controls to the actuators.
    /// @param _dt The timestep length.
    void Step(const double _dt);

    /// Align the multibody model to the robot's current simulated state.
    void SynchronizeModels() noexcept;

    /// Access the robot's simulation model. This is owned by the simulation
    /// engine.
    BulletModel* GetSimulationModel() noexcept;

    /// Set the robot's simulation model (should only happen in the main bullet
    /// engine).
    void SetSimulationModel(BulletModel* const _m);

    ///@}
    ///@name Geometry Accessors
    ///@{
    /// Access the robot's geometric representation. The robot will take
    /// ownership of its MultiBody and delete it when necessary.

    MultiBody* GetMultiBody() noexcept;
    const MultiBody* GetMultiBody() const noexcept;

    /// Access the robot manipulator's end effector, if it exists.
    const EndEffector& GetEndEffector() const noexcept;

    ///@}
    ///@name Agent Accessors
    ///@{
    /// Access the robot's agent. The robot will take ownership of its agent and
    /// delete it when necessary.

    Agent* GetAgent() noexcept;
    void SetAgent(std::unique_ptr<Agent>&& _a) noexcept;

    ///@}
    ///@name Control Accessors
    ///@{
    /// Access the robot's control structures. The robot will take ownership of
    /// these and delete them when necessary.

    ControllerMethod* GetController() noexcept;
    void SetController(std::unique_ptr<ControllerMethod>&& _c) noexcept;

    ///@}
    ///@name Actuator Accessors
    ///@{
    /// Access the robot's actuators. These are set during input file parsing
    /// and cannot be changed otherwise.

    /// Get actuator using label
    /// @param _label Label of actuator to retrieve
    Actuator* GetActuator(const std::string& _label) noexcept;
    /// Get set of all actuators mapped by label
    const std::unordered_map<std::string, std::unique_ptr<Actuator>>&
        GetActuators() const noexcept;

    ///@}
    ///@name Dynamics Accessors
    ///@{

    /// Get the robot's micro-simulator to test out controls.
    MicroSimulator* GetMicroSimulator() noexcept;

#ifdef PMPL_USE_MATLAB
    /// Get the robot's matlab micro-simulator. Currently this is only for the
    /// matlab-based steerable needle. Always returns null when compiled without
    /// matlab support.
    MatlabMicroSimulator* GetMatlabMicroSimulator() noexcept;
#endif

    ///@}
    ///@name Hardware Interface
    ///@{
    /// Access the interface to the hardware robot (if any).

    /// Get hardware command queue
    RobotCommandQueue* GetHardwareQueue() const noexcept;

    /// Get the emualted battery
    Battery* GetBattery() const noexcept;

    /// Get the state estimator
    StateEstimator* GetStateEstimator() const noexcept;

    /// Set the state estimator
    void SetStateEstimator(std::unique_ptr<StateEstimator>&& _stateEstimator)
        noexcept;

    ///@}
    ///@name Other Properties
    ///@{

    /// Check if this is a 'virtual' robot. These do not represent physical
    /// robots, and will be ignored by other robots in collision detection.
    /// Virtual robots will not appear in simulations.
    bool IsVirtual() const noexcept;

    /// Set the robot's virtual flag.
    /// @param _v The new value for the virtual flag.
    void SetVirtual(const bool _v) noexcept;

    /// Check if the robot is nonholonomic.
    bool IsNonholonomic() const noexcept;

    /// Check if the robot is car-like.
    bool IsCarlike() const noexcept;

    /// Get the maximum translational velocity for this robot.
    double GetMaxLinearVelocity() const noexcept;

    /// Get the maximum angular velocity for this robot.
    double GetMaxAngularVelocity() const noexcept;

    /// Get the unique label for this robot.
    const std::string& GetLabel() const noexcept;

    /// Get the default strategy label for this robot.
    const std::string& GetDefaultStrategyLabel() const noexcept;

    /// Get the capability for this robot.
    const std::string& GetCapability() const noexcept;

    /// Set the capability for this robot.
    void SetCapability(const std::string& _capability);

    /// Check if the robot is a manipulator
    bool IsManipulator() const noexcept;

    /// Check if the robot has a fixed base
    bool IsFixed() const noexcept;

    /// Get initial configuration for the robot (not implemented)
		Cfg GetInitialCfg();
    /// Set initial configuration for the robot (not implemented)
		void SetInitialCfg(Cfg _cfg);
    ///@}


};

/*----------------------------------- Debug ----------------------------------*/

std::ostream& operator<<(std::ostream&, const Robot&);

/*----------------------------------------------------------------------------*/

#endif
