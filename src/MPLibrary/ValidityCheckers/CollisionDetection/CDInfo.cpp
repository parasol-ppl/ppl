#include "CDInfo.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"

#include <limits>


/// Insert a value into a map if it didn't already exist, or update it if it is
/// less than the existing value.
/// @param _map The map container.
/// @param _key The key.
/// @param _value The value. It will be set if _key was not already in _map, or
///               if it is lower than the value currently mapped to _key.
template <template <typename...> class MapType, typename KeyType,
          typename ValueType>
void
UpdateMinimum(MapType<KeyType, ValueType>& _map, const KeyType _key,
    const ValueType _value) {
  auto iter = _map.find(_key);
  if(iter == _map.end())
    _map[_key] = _value;
  else
    iter->second = std::min(iter->second, _value);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CDClearanceMap ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*-------------------------------- Modifiers ---------------------------------*/

void
CDClearanceMap::
Merge(const CDClearanceMap& _other) {
  // Copy each entry from _other into this if it has lower clearance.
  for(const auto& keyValue : _other) {
    const auto& key   = keyValue.first;
    const auto& value = keyValue.second;

    if(key.first and key.second and value < GetClearance(key.first, key.second))
      SetClearance(key.first, key.second, value);
  }
}


void
CDClearanceMap::
SetClearance(const Body* const _a, const Body* const _b,
    const double _clearance) {
  if(!_a or !_b)
    throw RunTimeException(WHERE) << "Neither body can be null; this meaning is "
                                  << "reserved for internal use.";

  // Track body clearance.
  BodyKey key = MakeKey(_a, _b);
  m_bodyMap[key] = _clearance;
  UpdateMinimum(m_bodyMap, MakeKey(_a, nullptr), _clearance);
  UpdateMinimum(m_bodyMap, MakeKey(_b, nullptr), _clearance);

  // Track multibody clearance.
  UpdateMinimum(m_multibodyMap, MakeKey(_a->GetMultiBody(), _b->GetMultiBody()),
      _clearance);
  UpdateMinimum(m_multibodyMap, MakeKey(_a->GetMultiBody(), nullptr), _clearance);
  UpdateMinimum(m_multibodyMap, MakeKey(_b->GetMultiBody(), nullptr), _clearance);
}


void
CDClearanceMap::
Clear() {
  m_bodyMap.clear();
  m_multibodyMap.clear();
  m_minClearance = std::numeric_limits<double>::infinity();
  m_closestPair = {nullptr, nullptr};
}

/*---------------------------------- Queries ---------------------------------*/

double
CDClearanceMap::
GetClearance(const Body* const _a, const Body* const _b) const noexcept {
  auto iter = m_bodyMap.find(MakeKey(_a, _b));
  return iter == m_bodyMap.end() ? std::numeric_limits<double>::infinity()
                                 : iter->second;
}


double
CDClearanceMap::
GetClearance(const MultiBody* const _a, const MultiBody* const _b) const
    noexcept {
  auto iter = m_multibodyMap.find(MakeKey(_a, _b));
  return iter == m_multibodyMap.end() ? std::numeric_limits<double>::infinity()
                                      : iter->second;
}


double
CDClearanceMap::
GetMinClearance() const noexcept {
  return m_minClearance;
}


CDClearanceMap::BodyKey
CDClearanceMap::
GetClosestBodies() const noexcept {
  return m_closestPair;
}


CDClearanceMap::MultiBodyKey
CDClearanceMap::
GetClosestMultiBodies() const noexcept {
  if(m_closestPair.first and m_closestPair.second)
    return MakeKey(m_closestPair.first->GetMultiBody(),
                   m_closestPair.second->GetMultiBody());
  return {nullptr, nullptr};
}


CDClearanceMap::iterator
CDClearanceMap::
begin() const noexcept {
  return m_bodyMap.begin();
}


CDClearanceMap::iterator
CDClearanceMap::
end() const noexcept {
  return m_bodyMap.end();
}

/*--------------------------------- Helpers ----------------------------------*/

inline
CDClearanceMap::BodyKey
CDClearanceMap::
MakeKey(const Body* const _a, const Body* const _b) const noexcept {
  return {std::min(_a, _b), std::max(_a, _b)};
}


inline
CDClearanceMap::MultiBodyKey
CDClearanceMap::
MakeKey(const MultiBody* const _a, const MultiBody* const _b) const noexcept {
  return {std::min(_a, _b), std::max(_a, _b)};
}

/*----------------------------------------------------------------------------*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CDInfo ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

CDInfo::
CDInfo(bool _retAllInfo) {
  ResetVars(_retAllInfo);
}


void
CDInfo::
ResetVars(bool _retAllInfo) {
  m_retAllInfo = _retAllInfo;
  m_collidingObstIndex = -1;
  m_nearestObstIndex = -1;
  m_minDist = std::numeric_limits<double>::max();
  m_robotPoint(0, 0, 0);
  m_objectPoint(0, 0, 0);
  m_trianglePairs.clear();
  m_clearanceMap.Clear();
}

/*--------------------------------- Ordering ---------------------------------*/

bool
CDInfo::
operator<(const CDInfo& _cdInfo) const noexcept {
  return m_minDist < _cdInfo.m_minDist;
}

/*----------------------------------------------------------------------------*/
