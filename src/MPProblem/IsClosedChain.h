#ifndef IsClosedChain_h
#define IsClosedChain_h

class Cfg_reach_cc {
};

#include "boost/type_traits/is_same.hpp"
#include "boost/type_traits/is_base_of.hpp"
#include "boost/mpl/or.hpp"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <typename T>
struct IsClosedChain : boost::mpl::or_<
                                       boost::is_same<Cfg_reach_cc, T>,
                                       boost::is_base_of<Cfg_reach_cc, T>
                                      >::type {
};

#endif
