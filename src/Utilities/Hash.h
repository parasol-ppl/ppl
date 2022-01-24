#ifndef PMPL_HASH_H_
#define PMPL_HASH_H_

#include <functional>


namespace std {

  //////////////////////////////////////////////////////////////////////////////
  /// Define a hasher for a std::pair of elements.
  /// @note This implementation was borrowed from older versions of boost. I
  ///       will not use the newer version because it isn't guaranteed to give
  ///       the same behavior across multiple runs of a program, which is
  ///       detrimental randomness that we don't need (it obfuscates our control
  ///       of randomness in our sampling processes).
  /// @note Tested with 100M random pairs of same type and no collisions.
  /// @todo Double-check that we get similar performance with differing types.
  //////////////////////////////////////////////////////////////////////////////
  template <typename T, typename U>
  struct hash<std::pair<T, U>> {

    typedef std::pair<T, U> KeyPair;

    static constexpr size_t magicOffset = 0x9e3779b99e3779b9;

    size_t operator()(const KeyPair& _key) const noexcept {
      static constexpr std::hash<T> hasher1;
      static constexpr std::hash<U> hasher2;
      auto h1 = hasher1(_key.first),
           h2 = hasher2(_key.second);
      return h2 + magicOffset + (h1 << 6) + (h1 >> 2);
    }

  };


}

#endif
