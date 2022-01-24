#ifndef PMPL_CD_INFO_H_
#define PMPL_CD_INFO_H_

#include "Vector.h"

#include <limits>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

class Body;
class MultiBody;



////////////////////////////////////////////////////////////////////////////////
/// Map for tracking the detected clearance between bodies during collision
/// detection.
////////////////////////////////////////////////////////////////////////////////
class CDClearanceMap {

  public:

    ///@name Local Types
    ///@{

    typedef std::pair<const Body*, const Body*>            BodyKey;
    typedef std::pair<const MultiBody*, const MultiBody*>  MultiBodyKey;

    typedef std::map<BodyKey, double>                      BodyMap;
    typedef std::map<MultiBodyKey, double>                 MultiBodyMap;

    typedef typename BodyMap::const_iterator               iterator;

    ///@}
    ///@name Modifiers
    ///@{

    /// Merge another clearance map into this one. If a key is present in both
    /// maps, the lowest clearance will be maintained.
    /// @param _other The map to merge into this one.
    void Merge(const CDClearanceMap& _other);

    /// Set the observed clearance between two bodies.
    /// @param _a The first body.
    /// @param _b The second body.
    /// @param _clearance The clearance observed between _a and _b.
    void SetClearance(const Body* const _a, const Body* const _b,
        const double _clearance);

    /// Reset the structure.
    void Clear();

    ///@}
    ///@name Queries
    ///@{

    /// Get the observed clearance between two bodies.
    /// @param _a The first body.
    /// @param _b The second body.
    /// @return The clearance observed between _a and _b, or infinity if it was
    ///         not observed.
    /// @note Set one of the bodies to null to get the minimum clearance for the
    ///       non-null body.
    double GetClearance(const Body* const _a, const Body* const _b)
        const noexcept;

    /// Get the observed clearance between two multibodies.
    /// @param _a The first multibody.
    /// @param _b The second multibody.
    /// @return The clearance observed between _a and _b, or infinity if it was
    ///         not observed.
    /// @note Set one of the multibodies to null to get the minimum clearance
    ///       for the non-null multibody.
    double GetClearance(const MultiBody* const _a, const MultiBody* const _b)
        const noexcept;

    /// Get the lowest reported clearance.
    double GetMinClearance() const noexcept;

    /// Get the body pair which had the lowest clearance value.
    BodyKey GetClosestBodies() const noexcept;

    /// Get the multibody pair which had the lowest clearance value.
    MultiBodyKey GetClosestMultiBodies() const noexcept;

    iterator begin() const noexcept;
    iterator end()   const noexcept;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Create a clearance key for a pair of bodies.
    /// @param _a The first body.
    /// @param _b The second body.
    /// @return A clearance key with the lower address first.
    BodyKey MakeKey(const Body* const _a, const Body* const _b)
        const noexcept;

    /// Create a clearance key for a pair of multibodies.
    /// @param _a The first multibody.
    /// @param _b The second multibody.
    /// @return A clearance key with the lower address first.
    MultiBodyKey MakeKey(const MultiBody* const _a, const MultiBody* const _b)
        const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    BodyMap m_bodyMap;           ///< The mapped clearances.
    MultiBodyMap m_multibodyMap; ///< The minimum mb clearances.

    /// The lowest reported clearance value.
    double m_minClearance{std::numeric_limits<double>::infinity()};

    BodyKey m_closestPair{nullptr, nullptr}; ///<The closest reported body pair.

    ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// Information returned by validity checkers, e.g., distance from obstacles.
///
/// @todo Generalize this object to store collisions with obstacles, boundaries,
///       and other robots in a uniform way. It should contain a vector of
///       'Collision' structures which describe the object type, indexes, and
///       distance for each detected collision.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
struct CDInfo {

  ///@name Local Types
  ///@{

  /// A pair of triangle indexes on the robot (first) and obstacle (second) in a
  /// discovered collision.
  typedef std::pair<int, int> CollisionPair;

  ///@}
  ///@name Construction
  ///@{

  /// @param _retAllInfo Compute distance information if possible
  CDInfo(const bool _retAllInfo = false);

  /// Reset object to default state.
  /// @param _retAllInfo Compute distance information if possible
  void ResetVars(const bool _retAllInfo = false);

  ///@}
  ///@name Ordering
  ///@{

  /// Order these objects according to the minimum distance from closest
  /// colliding obstacle.
  /// @param _cdInfo Other CDInfo
  /// @return True if minimum distance is less than other's minimum distance.
  bool operator<(const CDInfo& _cdInfo) const noexcept;

  ///@}
  ///@name Internal State
  ///@{

  bool m_retAllInfo;              ///< Consider all collisions or only the first?
  int m_collidingObstIndex;       ///< Index for first discovered obstacle collision.
  int m_nearestObstIndex;         ///< Index for closest obstacle.
  double m_minDist;               ///< Distance between Robot and closest obstacle.

  mathtool::Vector3d m_robotPoint;  ///< Closest point on Robot to closest obstacle.
  mathtool::Vector3d m_objectPoint; ///< Closest point on closest obstacle to Robot.

  std::vector<CollisionPair> m_trianglePairs; ///< All colliding triangle pairs.

  CDClearanceMap m_clearanceMap; ///< Map of detected clearances.

  ///@}

};

#endif
