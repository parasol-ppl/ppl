#ifndef PMPL_GROUP_CFG_H_
#define PMPL_GROUP_CFG_H_

#include <cstddef>
#include <iostream>
#include <vector>

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/GroupLocalPlan.h"

#include "Transformation.h"
#include "Vector.h"

class Robot;
class RobotGroup;


////////////////////////////////////////////////////////////////////////////////
/// An aggregate configuration which represents a configuration for each robot
/// in a robot group.
///
/// The main point of group cfg is to take advantage of everything implemented
/// for individual robots. This means that we use VIDs from individual roadmaps
/// to keep track of the robot cfgs referred to in a group cfg. In the case that
/// a group cfg is modified or new in some way, there is a temporary local
/// storage (m_localCfgs) which stores individual cfgs not yet in a roadmap.
/// When adding a group cfg to a group roadmap, the VID is used in place after
/// adding the individual cfg to the individual roadmap.
///
/// @note Do not use 'SetRobotCfg' on a roadmap configuration with a non-VID:
///       The 'set' configuration will be lost since it is not in the individual
///       roadmap.
///
/// @todo Rework so that we only need a robot group to construct this, in which
///       case it will have all local cfgs. It should only get tied to a roadmap
///       with SetGroupRoadmap after it has been added to one.
////////////////////////////////////////////////////////////////////////////////
class GroupCfg final {

  public:

    ///@name Local Types
    ///@{

    typedef size_t           VID;      ///< A VID in an individual Robot roadmap.
    typedef std::vector<VID> VIDSet;   ///< A set of VIDs from indiv. Robot roadmaps.

    typedef Cfg              IndividualCfg;
    typedef GroupRoadmap<GroupCfg, GroupLocalPlan<IndividualCfg>> GroupRoadmapType;

    /// A formation represents a group of robots which are maintaining their
    /// configurations relative to a leader, such as maintaining a square or
    /// V-shape while moving. The values are robot indexes (within the group,
    /// not problem) with the first index denoting the leader robot. These are
    /// not stored in configurations but may be required for edges.
    typedef std::vector<size_t> Formation;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a group configuration.
    /// @param _groupMap The group roadmap to which this configuration belongs,
    ///                  or null if it is not in a map.
    /// @param _init Default-initialize local configurations?
    /// @todo This object does not work at all without a group map. We should
    ///       throw relevant exceptions if needed.
    explicit GroupCfg(GroupRoadmapType* const _groupMap = nullptr,
        const bool _init = false);

    ///@}
    ///@name Equality
    ///@{

    /// Check if the current and given group configurations are equal.
    /// @param _other The given group configuration.
    /// @return True if equal, false otherwise.
    bool operator==(const GroupCfg& _other) const noexcept;
    /// Check if the current and given group configurations are unequal.
    /// @param _other The given group configuration.
    /// @return True if unequal, false otherwise.
    bool operator!=(const GroupCfg& _other) const noexcept;

    ///@}
    ///@name Arithmetic
    ///@{

    /// Find the sum of the current and a given group configuration, 
    /// by each degree of freedom.
    /// @param _other The group configuration to be added. 
    /// @return The sum of the group configurations.
    GroupCfg operator+(const GroupCfg& _other) const;

    /// Find the difference of the current and a 
    /// given group configuration, by each degree of freedom.
    /// @param _other The group configuration to be subtracted. 
    /// @return The difference of the group configurations.
    GroupCfg operator-(const GroupCfg& _other) const;

    /// Find the product of the current and a given group configuration, 
    /// by each degree of freedom.
    /// @param _other The group configuration used to multiply the current. 
    /// @return The product of the group configurations.
    GroupCfg operator*(const double& _other) const;

    /// Add a given group configuration to the current, by each degree of freedom.
    /// @param _other The group configuration to be added.
    GroupCfg& operator+=(const GroupCfg& _other);

    /// Subtract a given group configuration from the current, by each degree of freedom.
    /// @param _other The group configuration to be subtracted.
    GroupCfg& operator-=(const GroupCfg& _other);

    /// Multiply the current group configuration by a scalar.
    /// @param _val The scalar used to multiply the current.
    GroupCfg& operator*=(const double& _val);

    ///@}
    ///@name Robots
    ///@{
    /// Access the robots within this group configuration.

    /// Get the number of robots.
    size_t GetNumRobots() const noexcept;

    /// Get the full vector of robot pointers.
    const std::vector<Robot*>& GetRobots() const noexcept;

    /// Get the robot pointer for a group member by index.
    /// @param _index The desired index.
    Robot* GetRobot(const size_t _index) const;

    ///@}
    ///@name Roadmap Accessors
    ///@{
    /// These functions provide access to the related group map (if any) and
    /// descriptors for non-local individual configurations.

    /// Get the group roadmap this group cfg is with respect to.
    GroupRoadmapType* GetGroupRoadmap() const noexcept;

    /// Change the roadmap that this group is using/in reference to. Also
    /// performs compatibility/verification tests to see if it's possible.
    /// Note: Does NOT add this new cfg to any roadmap, but makes all cfg info
    ///       local (everything will be in m_localCfgs) so that there are no
    ///       issues when adding to the roadmap later.
    /// @todo Fix this, it looks like it was meant to be a copy-constructor with
    ///       a different roadmap. We will still need a roadmap setter in case
    ///       the roadmap moves.
    GroupCfg SetGroupRoadmap(GroupRoadmapType* const _newRoadmap) const;

    /// Get the VID for a particular robot.
    /// @param _index The index (within the group) of the robot.
    /// @return The VID of the robot's individual configuration, or INVALID_VID
    ///         if it is a local configuration.
    VID GetVID(const size_t _index) const noexcept;

    ///@}
    ///@name Individual Configurations
    ///@{
    /// These functions manage the individual configurations that comprise this
    /// group configuration.

    /// Set the individual cfg for a robot to a local copy of an cfg.
    /// @param _robot The robot which the cfg refers to.
    /// @param _cfg The cfg.
    void SetRobotCfg(Robot* const _robot, IndividualCfg&& _cfg);

    /// Set the individual cfg for a robot to a local copy of an cfg.
    /// @param _index The robot's group index which the cfg refers to.
    /// @param _cfg The cfg.
    void SetRobotCfg(const size_t _index, IndividualCfg&& _cfg);

    /// Set the individual cfg for a robot to a roadmap copy of an cfg.
    /// @param _robot The robot which the cfg refers to.
    /// @param _vid The cfg descriptor.
    void SetRobotCfg(Robot* const _robot, const VID _vid);

    /// Set the individual cfg for a robot to a roadmap copy of an cfg.
    /// @param _index The robot's group index which the cfg refers to.
    /// @param _vid The cfg descriptor.
    void SetRobotCfg(const size_t _index, const VID _vid);

    /// Get the individual Cfg for a robot in the group.
    /// @param _robot The robot which the cfg refers to.
    /// @return The individual configuration for the indexed robot.
    IndividualCfg& GetRobotCfg(Robot* const _robot);

    /// Get the individual Cfg for a robot in the group.
    /// @param _index The index of the robot.
    /// @return The individual configuration for the indexed robot.
    IndividualCfg& GetRobotCfg(const size_t _index);

    /// Get the individual Cfg for a robot in the group.
    /// @param _robot The robot which the cfg refers to.
    /// @return The individual configuration for the indexed robot.
    const IndividualCfg& GetRobotCfg(Robot* const _robot) const;

    /// Get the individual Cfg for a robot in the group.
    /// @param _index The index of the robot.
    /// @return The individual configuration for the indexed robot.
    const IndividualCfg& GetRobotCfg(const size_t _index) const;

    /// Clear the Local Cfg information in the cfg (for after adding to roadmap)
    void ClearLocalCfgs();

    ///@}
    ///@name DOF Accessors
    ///@{
    /// @todo These should not return any single-robot values, which can always
    ///       be obtained by accessing the relevant robot.

    /// We assume homogeneous robots right now, so the default argument
    /// finds the values for the first one.
    size_t PosDOF(const size_t _index = 0) const;
    size_t OriDOF(const size_t _index = 0) const;
    size_t DOF(const size_t _index = 0) const;

    /// Check if there is a nonholonomic robot in the group.
    bool IsNonholonomic() const noexcept;

    /// Compute the total DOF for this robot group.
    size_t CompositeDOF() const;

    /// Compute the composite magnitude.
    double Magnitude() const;

    /// Compute the composite manitude of the positional components.
    double PositionMagnitude() const;

    /// Compute the composite magnitude of the orientation components.
    double OrientationMagnitude() const;

    ///@}
    ///@name Configuration Helpers
    ///@{

    /// Configure each individual cfg that this group cfg represents.
    /// This is for template cooperation and is also a needed function before
    /// dealing with CD calls and such.
    /// Note this will throw an exception if no cfg is present for any robot.
    void ConfigureRobot() const;

    /// Check that another GroupCfg is within a resolution as specified.
    /// @param _cfg The test configuration.
    /// @param _posRes The position resolution.
    /// @param _oriRes The orientation resolution.
    /// @return True if each individual configuration in this is within a
    ///         resolution distance of _cfg.
    bool WithinResolution(const GroupCfg& _cfg, const double _posRes,
        const double _oriRes) const;

    ///@}
    ///@name DOF Modifiers
    ///@{

    // Note: Using these functions will make this configuration utilize the
    // local cfgs, which won't be in group/individual roadmaps until added.


    /// Given this GroupCfg as the starting configuration, this function applies
    /// a rotation to all the robots that are listed, assuming the first one is
    /// the formation's leader.
    /// Note: Currently assumes all robots just have ONE body. The case of multi-
    /// bodied robots would need to be specially handled (right now it should just
    /// be split into multiple robots if a group is needed).
    /// @param _robotList This list of bodies to rotate. First one is leader body.
    /// @param _rotation The change in orientation that should be applied to _cfg.
    /// @param _debug A flag to print to cout (no access to an m_debug flag here).
    void RotateFormationAboutLeader(const Formation& _robotList,
                                    const mathtool::Orientation& _rotation,
                                    const bool _debug = false);

    /// Given this GroupCfg as the starting configuration, this function applies
    /// a transformation uniformly over all robots listed.
    /// Note: Currently assumes all robots just have ONE body. The case of multi-
    /// bodied robots would need to be specially handled (right now it should just
    /// be split into multiple robots if a group is needed).
    /// Note: This is assuming 6 DOFs!
    /// @param _robotList This list of bodies to rotate. First one is leader body.
    /// @param _transform The change in orientation that should be applied to _cfg.
    /// @param _relativeTransform (Optional) The transformation to "undo" before
    ///        applying _transform. If default, it will be a simple transform
    ///        application. See RotateFormationAboutLeader for usage example.
    void ApplyTransformationForRobots(const Formation& _robotList,
                             const mathtool::Transformation& _transform,
                             const mathtool::Transformation& _relativeTransform
                                                  = mathtool::Transformation());


    /// Given this configuration, add in the same DOF values to each body given.
    /// It assumes that all robots in the formation given have the same DOFs.
    /// This is a common thing to do in assembly planning/composite C-Spaces.
    /// @param _dofs The values to add in to each body. This function assumes each
    ///              body has #dofs = _dofs.size().
    /// @param _robots This list of bodies to update. Order doesn't matter.
    void AddDofsForRobots(const std::vector<double>& _dofs,
                          const Formation& _robots);


    /// This function adds all positional dofs in _dofs. It will handle 1-3 dofs
    /// based on each IndividualCfg's PosDof value. Does not add in orientation.
    /// Note: This function is slightly more efficient than the std::vector
    ///       version, as we do not need to check the size of _dofs.
    /// @param _dofs The positional values to add in to each body.
    /// @param _robots This list of bodies to update. Order doesn't matter.
    void AddDofsForRobots(const mathtool::Vector3d& _dofs,
                          const Formation& _robots);

    /// Given new DOF values, overwrite the existing values for each individual
    /// cfg in this group cfg that is listed in _robots. Note that _dofs needs
    /// to be the same number of DOFs as each individual cfg in the group.
    /// @param _fromCfg The configuration to take values from.
    /// @param _robots This list of bodies to update. Order doesn't matter.
    void OverwriteDofsForRobots(const std::vector<double>& _dofs,
                                const Formation& _robots);


    /// Given new DOF values, overwrite the existing values for each individual
    /// cfg in this group cfg that is listed in _robots. Note that _dofs needs
    /// to be the same number of DOFs as each individual cfg in the group.
    /// @param _fromCfg The configuration to take values from.
    /// @param _robots This list of bodies to update. Order doesn't matter.
    void OverwriteDofsForRobots(const mathtool::Vector3d& _dofs,
                                const Formation& _robots);


    /// Given this and another configuration, copy the DOF values from the other
    /// to the DOF values in this one, but only for each given robot indexed.
    /// @param _fromCfg The configuration to take values from.
    /// @param _robots This list of bodies to update. Order doesn't matter.
    void OverwriteDofsForRobots(const GroupCfg& _fromCfg,
                                const Formation& _robots);

    /// @overload to handle robot pointers.
    void OverwriteDofsForRobots(const GroupCfg& _fromCfg,
                                const std::vector<Robot*>& _robots);


    /// Overwrites all data in this cfg, assumes the length of _dofs is the same
    /// as CompositeDOF(). Basically just converts a composite C-Space vector
    /// into its individual pieces for a Group Cfg.
    void SetData(const std::vector<double>& _dofs);

    /// Find the c-space increment and number of steps needed to move from a
    /// start to a goal, taking steps no larger than the designated resolutions.
    /// @param _start The start configuration.
    /// @param _goal The goal configuration.
    /// @param _nTicks The number of steps to take (NOT computed by this method)
    void FindIncrement(const GroupCfg& _start, const GroupCfg& _goal,
        const int _nTicks);

    /// Find the c-space increment and number of steps needed to move from a
    /// start to a goal, taking steps no larger than the designated resolutions.
    /// @param _start The start configuration.
    /// @param _goal The goal configuration.
    /// @param _nTicks The number of steps to take (computed by this method).
    /// @param _positionRes The position resolution to use.
    /// @param _orientationRes The orientation resolution to use.
    void FindIncrement(const GroupCfg& _start, const GroupCfg& _goal,
        int* const _nTicks, const double _positionRes,
        const double _orientationRes);

    /// Test if a group configuration lies within a boundary and also within the
    /// robot's c-space limits.
    /// @param _boundary The boundary to check.
    /// @return True if the configuration places the robot inside both the
    ///         boundary and its DOF limits.
    bool InBounds(const Boundary* const _b) const noexcept;
    /// @overload
    bool InBounds(const Environment* const _env) const noexcept;

    /// Normalize Orientation DOFs for a Group Cfg
    virtual void NormalizeOrientation(const Formation& _robots = Formation())
        noexcept;

    ///@}
    ///@name Output Utilities
    ///@{

    std::string PrettyPrint(const size_t _precision = 4) const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Return whether the cfg for the robot is local to the group cfg, or if
    /// it's in an individual roadmap already.
    bool IsLocalCfg(const size_t _robotIndex) const noexcept;

    /// Initialize the set of local configurations if not already done.
    void InitializeLocalCfgs() noexcept;

    /// Verify that an index is valid. Throw an exception if not.
    /// @param _robotIndex The (group) index to verify.
    void VerifyIndex(const size_t _robotIndex) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    GroupRoadmapType* m_groupMap{nullptr};  ///< The robot group.

    VIDSet m_vids;   ///< The individual VIDs in this aggregate configuration.
    std::vector<IndividualCfg> m_localCfgs; ///< Individual cfgs not in a map.

    ///@}

};

std::ostream& operator<<(std::ostream&, const GroupCfg&);

#endif
