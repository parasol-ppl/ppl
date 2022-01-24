#ifndef PMPL_NEIGHBORS_H_
#define PMPL_NEIGHBORS_H_

#include <cstddef>
#include <functional>
#include <limits>


////////////////////////////////////////////////////////////////////////////////
/// Describes a distance relation between two cfgs as determined by a
/// neighborhood finder. At least one of the cfgs should be in the roadmap.
////////////////////////////////////////////////////////////////////////////////
struct Neighbor final {

  ///@name Local Types
  ///@{

  typedef size_t VID;

  /// A function for comparing two neighbors for ordering purposes. Returns true
  /// if _a is ordered before _b.
  typedef std::function<bool(const Neighbor& _a, const Neighbor& _b)> Comparator;

  ///@}
  ///@name Internal State
  ///@{

  VID source{VID(-1)};  ///< The source VID.
  VID target{VID(-1)};  ///< The target VID.

  /// The distance between source and target.
  double distance{std::numeric_limits<double>::infinity()};

  ///@}
  ///@name Construction
  ///@{

  Neighbor();

  Neighbor(const VID _target, const double _distance);

  Neighbor(const VID _source, const VID _target, const double _distance);

  ///@}
  ///@name Equality
  ///@{

  /// Checks equality with another Neighbor
  bool operator==(const Neighbor& _other) const noexcept;

  /// Checks inequality with another Neighbor
  bool operator!=(const Neighbor& _other) const noexcept;

  ///@}
  ///@name Ordering
  ///@{
  /// There are several ways to order a set of neighbors. The default is to
  /// order only by distance. The static functions here give alternate
  /// comparators that can be used to select the desired ordering.

  /// Orders by distance only.
  bool operator<(const Neighbor& _other) const noexcept;

  /// Orders by distance, then target VID.
  static Comparator OrderByDistanceThenTarget() noexcept;

  /// Orders by distance, then source VID.
  static Comparator OrderByDistanceThenSource() noexcept;

  /// Orders by distance, then source, then target.
  static Comparator OrderByAll() noexcept;

  ///@}

};

/*------------------------------- Construction -------------------------------*/

inline
Neighbor::
Neighbor() = default;


inline
Neighbor::
Neighbor(const VID _target, const double _distance)
  : target(_target), distance(_distance)
{ }


inline
Neighbor::
Neighbor(const VID _source, const VID _target, const double _distance)
  : source(_source), target(_target), distance(_distance)
{ }

/*--------------------------------- Equality ---------------------------------*/

inline
bool
Neighbor::
operator==(const Neighbor& _other) const noexcept {
  return source == _other.source
     and target == _other.target
     and distance == _other.distance;
}


inline
bool
Neighbor::
operator!=(const Neighbor& _other) const noexcept {
  return !(*this == _other);
}


/*--------------------------------- Ordering ---------------------------------*/

inline
bool
Neighbor::
operator<(const Neighbor& _other) const noexcept {
  return distance < _other.distance;
}


inline
Neighbor::Comparator
Neighbor::
OrderByDistanceThenTarget() noexcept {
  return [](const Neighbor& _a, const Neighbor& _b) -> bool {
    if(_a.distance < _b.distance)
      return true;
    else if(_a.distance > _b.distance)
      return false;
    else if(_a.target < _b.target)
      return true;
    else if(_a.target > _b.target)
      return false;
    return false;
  };
}


inline
Neighbor::Comparator
Neighbor::
OrderByDistanceThenSource() noexcept {
  return [](const Neighbor& _a, const Neighbor& _b) -> bool {
    if(_a.distance < _b.distance)
      return true;
    else if(_a.distance > _b.distance)
      return false;
    else if(_a.source < _b.source)
      return true;
    else if(_a.source > _b.source)
      return false;
    return false;
  };
}


inline
Neighbor::Comparator
Neighbor::
OrderByAll() noexcept {
  return [](const Neighbor& _a, const Neighbor& _b) -> bool {
    if(_a.distance < _b.distance)
      return true;
    else if(_a.distance > _b.distance)
      return false;
    else if(_a.source < _b.source)
      return true;
    else if(_a.source > _b.source)
      return false;
    else if(_a.target < _b.target)
      return true;
    else if(_a.target > _b.target)
      return false;
    return false;
  };
}

/*----------------------------------------------------------------------------*/

#endif
