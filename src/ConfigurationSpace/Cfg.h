#ifndef PMPL_CFG_H_
#define PMPL_CFG_H_

#include <cstddef>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include "Geometry/Boundaries/Range.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/MPUtils.h"

#include "Vector.h"

class MultiBody;
class Boundary;
class Environment;
class Robot;

enum class DofType;

namespace mathtool {
  class EulerAngle;
  class Transformation;
}


////////////////////////////////////////////////////////////////////////////////
/// A point in configuration space.
///
/// @details Each instance holds an array of values representing all the degrees
///          of freedom of a robot. Translational DOFs represent the position of
///          a reference point on the robot relative to the world origin. Angular
///          and joint DOFs are normalized to the range [-1, 1), so this object
///          can only reliably represent points in @cspace and not directions.
////////////////////////////////////////////////////////////////////////////////
class Cfg {

  public:

    ///@name Construction
    ///@{

    /// Construct a configuration for Cfg::inputRobot.
    /// @details This constructor is provided for two cases: allocating storage
    ///          for Cfgs without knowing the robot in advance, and reading in
    ///          roadmaps. The robot pointer will be Cfg::inputRobot, which can
    ///          be thought of as a variable default for the Cfg(Robot* const)
    ///          constructor.
    explicit Cfg();

    /// Construct a configuration for a given robot.
    /// @param _robot The robot this represents.
    explicit Cfg(Robot* const _robot);

    /// Construct a configuration for a given robot at a specified point in
    /// workspace.
    /// @param _v The workspace location of the robot's reference point.
    /// @param _robot The robot to represent.
    explicit Cfg(const mathtool::Vector3d& _v, Robot* const _robot = nullptr);
    
    /// Construct a copy of a configuration.
    /// @param _other The configuration to copy.
    Cfg(const Cfg& _other);

    /// Construct a copy of a configuration.
    /// @param _other The configuration to copy.
    Cfg(Cfg&& _other);

    virtual ~Cfg();

    ///@}
    ///@name Assignment
    ///@{

    /// Set a configuration equal to another.
    /// @param _cfg The configuration to be set to.
    Cfg& operator=(const Cfg& _cfg);

    /// Set a configuration equal to another.
    /// @param _cfg The configuration to be set to.
    Cfg& operator=(Cfg&& _cfg);

    /// Add a given configuration to the current, by each degree of freedom.
    /// @param _cfg The configuration to be added.
    Cfg& operator+=(const Cfg& _cfg);
    /// Subtract a given configuration from the current, by each degree of freedom.
    /// @param _cfg The configuration to be subtracted.
    Cfg& operator-=(const Cfg& _cfg);
    /// Multiply the current configuration by a given configuration, by each degree of freedom
    /// @param _cfg The configuration to multiply the current.
    Cfg& operator*=(const Cfg& _cfg);
    /// Divide the current configuration by a given configuration, by each degree of freedom.
    /// @param _cfg The configuration to divide the current.
    Cfg& operator/=(const Cfg& _cfg);
    /// Multiply the current configuration by a scalar.
    /// @param _d The scalar used to multiply the current.
    Cfg& operator*=(const double _d);
    /// Divide the current configuration by a scalar.
    /// @param _d The scalar used to divide the current.
    Cfg& operator/=(const double _d);

    ///@}
    ///@name Arithmetic
    ///@{

    /// Find the negative of the configuration.
    Cfg operator-() const;

    /// Find the sum of the current and a given configuration, 
    /// by each degree of freedom.
    /// @param _cfg The configuration to be added. 
    /// @return The sum of the configurations.
    Cfg operator+(const Cfg& _cfg) const;

    /// Find the difference of the current and a given configuration, 
    /// by each degree of freedom.
    /// @param _cfg The configuration to be subtracted. 
    /// @return The difference of the configurations.
    Cfg operator-(const Cfg& _cfg) const;

    /// Find the product of the current and a given configuration, 
    /// by each degree of freedom.
    /// @param _cfg The configuration used to multiply the current. 
    /// @return The product of the configurations.
    Cfg operator*(const Cfg& _cfg) const;

    /// Find the quotient of the current and a given configuration, 
    /// by each degree of freedom.
    /// @param _cfg The configuration used to divide the current. 
    /// @return The quotient of the configurations.
    Cfg operator/(const Cfg& _cfg) const;

    /// Find the configuration obtained by multiplying the current
    /// by a scalar.
    /// @param _d The scalar used to multiply the current.
    /// @return The multiplied configuration.
    Cfg operator*(const double _d) const;

    /// Find the configuration obtained by dividing the current
    /// by a scalar.
    /// @param _d The scalar used to divide the current.
    /// @return The divided configuration.
    Cfg operator/(const double _d) const;

    ///@}
    ///@name Equality
    ///@{

    /// Check if the current and given configurations are equal.
    /// @param _cfg The given configuration.
    /// @return True is equal, false otherwise.
    bool operator==(const Cfg& _cfg) const;

    /// Check if the current and given configurations are unequal.
    /// @param _cfg The given configuration.
    /// @return True is unequal, false otherwise.
    bool operator!=(const Cfg& _cfg) const;

    /// Check if the current and given configurations are equal.
    /// within a given set of resolutions.
    /// @param _cfg The given configuration.
    /// @param _posRes The resolution for positional degrees of freedom.
    /// @param _oriRes The resolution for orientational degrees of freedom.
    /// @return True if both are within the given resolutions, false otherwise.
    bool WithinResolution(const Cfg& _cfg, const double _posRes,
                          const double _oriRes) const;

    ///@}
    ///@name Comparison
    ///@{

    /// Check if the current configuration is less that the given.
    /// @param _cfg The given configuration.
    /// @return True if less than the given, false otherwise.
    /// @todo The function currently seems to check each dof in order,
    ///       returning T/F immediately unless exactly equal; is this intended?
    bool operator<(const Cfg& _cfg) const;

    ///@}
    ///@name Robot Info
    ///@{

    /// Get the robot corresponding to this configuration.
    /// @return The corresponding robot.
    Robot* GetRobot() const noexcept;

    /// Set the robot corresponding to this configuration.
    /// @param _r The desired new robot.
    void SetRobot(Robot* const _r) noexcept;

    /// Get the robot's multibody.
    /// @return The robot's multibody.
    MultiBody* GetMultiBody() const noexcept;

    /// Get the robot's DOF count.
    /// @return The robot's DOF count.
    /// @warning For composite C-Space this returns a total DOF count.
    size_t DOF() const noexcept;

    /// Get the robot's positional DOF count.
    /// @return The robot's positional DOF count.
    /// @warning For composite C-Space this function return "per body" counts.
    size_t PosDOF() const noexcept;

    /// Get the robot's orientational DOF count.
    /// @return The robot's orientational DOF count.
    /// @warning For composite C-Space this function return "per body" counts.
    size_t OriDOF() const noexcept;

    /// Get the robot's joint DOF count.
    /// @return The robot's joint DOF count.
    /// @warning For composite C-Space this function return "per body" counts.
    size_t JointDOF() const noexcept;

    /// Is the robot nonholonomic?
    bool IsNonholonomic() const noexcept;

    ///@}
    ///@name DOF Accessors
    ///@{

    /// Access the data for a given DOF.
    /// @param _dof The index of the desired DOF.
    /// @return The desired DOF.
    double& operator[](const size_t _dof) noexcept;
    double operator[](const size_t _dof) const noexcept;

    /// Get the data for all DOFs.
    /// @return A vector of all DOFs.
    const std::vector<double>& GetData() const noexcept;

    /// Access the velocity data for a given DOF.
    /// @param _dof The index of the desired DOF.
    /// @return The desired DOF's velocity data.
    double& Velocity(const size_t _dof) noexcept;
    double Velocity(const size_t _dof) const noexcept;

    /// Get the velocity data for all DOFs.
    /// @return A vector of all DOF velocity data.
    const std::vector<double>& GetVelocity() const noexcept;

    /// Set the DOF data.
    /// @param _data The vector of new DOF data.
    virtual void SetData(const std::vector<double>& _data);
    virtual void SetData(std::vector<double>&& _data);

    /// Set the joint DOF data. Other DOFs will remain unchanged.
    /// @param _data The vector of new DOF joint data.
    void SetJointData(const std::vector<double>& _data);

    /// Get the robot's reference point.
    Point3d GetPoint() const noexcept;

    /// Get a vector of the robot's positional DOFs.
    virtual std::vector<double> GetPosition() const;
    /// Get a vector of the robot's rotational DOFs.
    virtual std::vector<double> GetRotation() const;
    /// Get a vector of the robot's joint DOFs.
    virtual std::vector<double> GetJoints() const;
    /// Get a vector of the robot's non-joint DOFs.
    virtual std::vector<double> GetNonJoints() const;
    /// Get a vector of the robot's orientational DOFs.
    virtual std::vector<double> GetOrientation() const;

    /// Get the magnitude of the robot's DOF vector.
    virtual double Magnitude() const;
    /// Get the magnitude of the robot's positional DOF vector.
    virtual double PositionMagnitude() const;
    /// Get the magnitude of the robot's orientational DOF vector.
    virtual double OrientationMagnitude() const;

    /// Get the position in R^3.
    mathtool::Vector3d GetLinearPosition() const;
    /// Get the euler vector rotation.
    mathtool::Vector3d GetAngularPosition() const;
    /// Get the euler angle rotation.
    mathtool::EulerAngle GetEulerAngle() const;
    /// Get the position velocity in R^3.
    mathtool::Vector3d GetLinearVelocity() const;
    /// Get the rotation velocity in R^3.
    mathtool::Vector3d GetAngularVelocity() const;
    /// Get the world transformation of the robot's base.
    mathtool::Transformation GetBaseTransformation() const;

    /// Sets the robot's linear position.
    void SetLinearPosition(const mathtool::Vector3d&);
    /// Sets the robot's angular position.
    void SetAngularPosition(const mathtool::Vector3d&);
    /// Sets the robot's euler angle.
    void SetEulerAngle(const mathtool::EulerAngle&);
    /// Sets the robot's linear velocity.
    void SetLinearVelocity(const mathtool::Vector3d&);
    /// Sets the robot's angular velocity.
    void SetAngularVelocity(const mathtool::Vector3d&);
    /// Sets the robot's base tranformation.
    void SetBaseTransformation(const mathtool::Transformation&);

    /// Transforms the cfg about a new frame of reference
    void TransformCfg(const mathtool::Transformation&);

    ///@}
    ///@name Labels and Stats
    ///@{
    /// Each Cfg has a set of labels and stats. Label are boolean attributes,
    /// while stats are real-valued.

    /// Get the robot's label for the given string identifier.
    bool GetLabel(const std::string& _label) const;
    /// Does the robot have a given string label?
    bool IsLabel(const std::string& _label) const noexcept;
    /// Set the robot's string label given the string identifier.
    void SetLabel(const std::string& _label, const bool _value) noexcept;

    /// Get a statistic given a string identifier.
    double GetStat(const std::string& _stat) const;
    /// Does the robot have a statistic given a string identifier?
    bool IsStat(const std::string& _stat) const noexcept;
    /// Set  a statistic given a string identifier.
    void SetStat(const std::string& _stat, const double _value = 0) noexcept;
    /// Increment a statistic given a string identifier.
    void IncrementStat(const std::string& _stat, const double _value = 1)
        noexcept;

    ///@}
    ///@name Generation Methods
    ///@{

    /// Set all DOFs to zero.
    void Zero() noexcept;

    /// Test if a configuration lies within a boundary and also within the
    /// robot's c-space limits.
    /// @param _boundary The boundary to check.
    /// @return True if the configuration places the robot inside both the
    ///         boundary and its DOF limits.
    bool InBounds(const Boundary* const _b) const noexcept;
    /// @overload
    bool InBounds(const Environment* const _env) const noexcept;

    /// Create a configuration where workspace robot's EVERY VERTEX
    /// is guaranteed to lie within the specified boundary. If
    /// a cfg can't be found, the program will abort.
    /// The function will try a predefined number of times.
    /// @param _b The given boundary.
    virtual void GetRandomCfg(const Boundary* const _b);
    /// Create a configuration within the given environment.
    /// @param _env The given environment.
    virtual void GetRandomCfg(Environment* _env);

    /// Randomly sample the velocity for this configuration.
    virtual void GetRandomVelocity();

    /// Generate a random configuration with a set length.
    /// @param _length The desired length.
    /// @param _dm The distance metric for checking length.
    /// @param _norm Normalize the orientation DOFs?
    template <typename DistanceMetricPointer>
    void GetRandomRay(const double _length, DistanceMetricPointer _dm,
        const bool _norm = true);

    /// Configure the robot with the DOF values of this configuration.
    virtual void ConfigureRobot() const;

    /// Move this configuration towards a goal by adding a fixed increment.
    /// @param _goal The desired goal configuration.
    /// @param _increment The fixed increment to add to this, moving towards
    ///                   goal.
    virtual void IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment);

    /// Find the c-space increment that moves from a start to a goal in a fixed
    /// number of steps.
    /// @param _start The start configuration.
    /// @param _goal The goal configuration.
    /// @param _nTicks The number of steps to take.
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal,
        const int _nTicks);

    /// Find the c-space increment and number of steps needed to move from a
    /// start to a goal, taking steps no larger than the designated resolutions.
    /// @param _start The start configuration.
    /// @param _goal The goal configuration.
    /// @param _nTicks The number of steps to take (computed by this method).
    /// @param _positionRes The position resolution to use.
    /// @param _orientationRes The orientation resolution to use.
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks,
        const double _positionRes, const double _orientationRes);

    /// Create a configuration from the weighted sum of two other cfgs.
    /// @param _c1 The first configuration.
    /// @param _c2 The second configuration.
    /// @param _weight The weight for the second configuration. The first will
    ///                have (1 - _weight).
    virtual void WeightedSum(const Cfg& _c1, const Cfg& _c2,
        const double _weight = .5);

    /// Extract the position and orientation for this configuration from two
    /// other configurations.
    /// @param _pos Copy the position from this configuration.
    /// @param _ori Copy the orientation from this configuration.
    virtual void GetPositionOrientationFrom2Cfg(const Cfg& _pos, const Cfg& _ori);

    ///@}
    ///@name C-Space Directions
    ///@{

    /// Find a c-space direction to another configuration in the local (base body)
    /// frame of this Cfg. The rotational component will be in Euler Vector
    /// representation since Euler Angles do not really live in the local or
    /// global frame.
    /// @param _target The target configuration.
    /// @return The c-space direction from this to _target in the local frame of
    ///         this, using Euler Vector representation for the orientation.
    /// @WARNING This function is still wrong, it does not handle the rotational
    ///          components correctly.
    std::vector<double> DirectionInLocalFrame(const Cfg& _target) const;

    ///@}
    ///@name I/O
    ///@{

    // Static pointer for reading roadmaps. It should be set to the relevant
    // robot before reading in the map file, and nullptr otherwise.
    /// @TODO This is needed because we use stapl's graph reading function,
    ///       which does not allow us to set the robot pointers on construction.
    ///       Devise a better scheme for doing this that does not involve static
    ///       data.
    static Robot* inputRobot;

    /// Read a configuration from an input stream.
    /// @param _is The input stream to read from.
    virtual void Read(std::istream& _is);

    /// Write a configuration to an output stream.
    /// @param _os The output stream to write to.
    virtual void Write(std::ostream& _os) const;

    /// Print the Cfg's dofs and velocities with limited precision for terminal
    /// debugging.
    /// @param _precision The display precision.
    /// @return A string of [dof1, dof2, ..., dofn] for holonomic robots, or
    ///         {[dof1, ..., dofn], <vel1, ..., veln>} for nonholonomic robots.
    std::string PrettyPrint(const size_t _precision = 4) const;

    ///@}
    ///@name Internal State with poor encapsulation
    ///@{
    /// @TODO Fix encapsulation issues.
    /// @TODO Witness should not be a shared_ptr.

    CDInfo m_clearanceInfo;
    std::shared_ptr<Cfg> m_witnessCfg;

    ///@}
    ///@name Helpers
    ///@{

    /// Enable the normalization of orientation DOFs.
    void EnableNormalization() const;

    /// Disable the normalization of orientation DOFs.
    void DisableNormalization() const;

    /// Normalize an orientation DOF to the range [-1, 1).
    /// @param _index The index of the DOF to normalize. If it is -1, all
    ///               orientation DOFs will be normalized.
    virtual void NormalizeOrientation(const int _index = -1) noexcept;

    /// Ensure that this Cfg respects the robot's velocity limits.
    void EnforceVelocityLimits() noexcept;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::vector<double> m_dofs;    ///< The DOF values.
    std::vector<double> m_vel;     ///< The velocities, if any.
    Robot* m_robot{nullptr};       ///< The robot this cfg refers to.

    std::map<std::string, bool> m_labelMap;  ///< A map of labels for this cfg.
    std::map<std::string, double> m_statMap; ///< A map of stats for this cfg.

    /// The function to use for normalizing orientation DOFs.
    mutable double (*m_normalizer)(const double&){Normalize};

    ///@}

};

/*--------------------------- Generation Methods -----------------------------*/

template <class DistanceMetricPointer>
void
Cfg::
GetRandomRay(const double _length, DistanceMetricPointer _dm, const bool _norm) {
  // Randomly sample DOFs.
  for(size_t i = 0; i < DOF(); ++i)
    m_dofs[i] = 2. * DRand() - 1.;

  // Scale to appropriate length.
  _dm->ScaleCfg(_length, *this);

  // Normalize if requested.
  if(_norm)
    NormalizeOrientation();
}

/*----------------------------------------------------------------------------*/

std::ostream& operator<<(std::ostream& _os, const Cfg& _cfg);
std::istream& operator>>(std::istream& _is, Cfg& _cfg);

#endif
