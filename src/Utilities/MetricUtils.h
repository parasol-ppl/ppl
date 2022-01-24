#ifndef PMPL_METRIC_UTILS_H_
#define PMPL_METRIC_UTILS_H_

#include "Utilities/Average.h"
#include "Utilities/ClockClass.h"

#ifndef _PARALLEL
#include <containers/sequential/graph/algorithms/connected_components.h>
#endif

#include <cstddef>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// The StatClass is a storage hub of all statistics to be tracked in PMPL,
/// including but not limited to timing, success/fail attempts,
/// collision detection calls.
///
/// @ingroup MetricUtils
////////////////////////////////////////////////////////////////////////////////
class StatClass final {

  public:

    StatClass();

    void ClearStats();

    int IncNumCollDetCalls(const std::string& _cdName,
        const std::string& _callName);
    size_t GetIsCollTotal();
    void IncCfgIsColl(const std::string& _callName);

    int IncLPConnections(const std::string& _lpName , int _incr = 1);
    int IncLPAttempts(const std::string& _lpName, int _incr = 1);
    int IncLPCollDetCalls(const std::string& _lpName, int _incr = 1);

    void IncNodesGenerated(const std::string& _samplerName, size_t _incr = 1);
    void IncNodesAttempted(const std::string& _samplerName, size_t _incr = 1);

    //Clock Accessors
    void ClearClock(const std::string& _name);
    void StartClock(const std::string& _name);
    void StopClock(const std::string& _name);
    void StopPrintClock(const std::string& _name, std::ostream& _os);
    void PrintClock(const std::string& _name, std::ostream& _os);
    double GetSeconds(const std::string& _name);
    int GetUSeconds(const std::string& _name);

    // Statistics Accessors/Modifiers
    double GetStat(const std::string& _s);
    void SetStat(const std::string& _s, const double _v);
    void IncStat(const std::string& _s, const double _v = 1);

    // Average Accessors
    Average<double>& GetAverage(const std::string& _s);

    // Histories
    std::vector<double>& GetHistory(const std::string& _s);
    void AddToHistory(const std::string& _s, double _v);
    void WriteHistory(const std::string& _s);

    void SetAuxDest(const std::string& _s);

    void PrintAllStats(std::ostream& _os);

    template <typename RoadmapType>
    void PrintAllStats(std::ostream& _os, RoadmapType* _rmap = nullptr);

    template<class GraphType>
    void DisplayCCStats(std::ostream& _os, GraphType&);

    // m_lpInfo represents information about the Local Planners, referenced by
    // name
    // m_lpInfo.first is the name of the Local Planner
    // m_lpInfo.second.get<0>() is the # of LP attempts
    // m_lpInfo.second.get<1>() is the # of LP connections (successes)
    // m_lpInfo.second.get<2>() is the # of LP collision detection calls
    std::map<std::string, std::tuple<size_t, size_t, size_t> > m_lpInfo;
    std::map<std::string, size_t> m_collDetCountByName;

    std::map<std::string, ClockClass> m_clockMap;

    ///IsColl simply counts the number of times a Cfg is tested for Collision.
    ///\see Cfg::isCollision
    std::map<std::string, size_t> m_isCollByName;
    size_t m_isCollTotal;


    //m_samplerInfo represents sampler nodes attempted and generated
    //  map<string, pair<int, int> > represents a mapping between the sampler name
    //  and first the number of attempted samples, then the number of generated samples
    //  that is, pair.first = attempts, pair.second = generated (successful attempts)
    std::map<std::string, std::pair<size_t, size_t>> m_samplerInfo;

  private:

    ///@name Internal State
    ///@{

    std::map<std::string, size_t> m_numCollDetCalls;
    std::map<std::string, double> m_stats;
    std::map<std::string, Average<double>> m_averages;
    std::map<std::string, std::vector<double>> m_histories;
    std::string m_auxFileDest;

    ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// A timer object for functions that starts a clock on construction and stops
/// it on destruction.
////////////////////////////////////////////////////////////////////////////////
class MethodTimer final {

  ///@name Internal State
  ///@{

  StatClass* const m_stats;    ///< The stat class which owns the clock.
  const std::string m_label;   ///< The clock label.

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a method timer, which starts the named clock.
    /// @param _stats The stat class which owns the clock.
    /// @param _label The clock label.
    MethodTimer(StatClass* const _stats, const std::string& _label);

    /// Destruction stops the named clock.
    ~MethodTimer();

    ///@}

};


template <typename RoadmapType>
void
StatClass::
PrintAllStats(std::ostream& _os, RoadmapType* _rmap) {
  if(_rmap) {
    // Output roadmap statistics.
    _os << "Roadmap Statistics:\n"
        << "\n  Number of Nodes: " << _rmap->get_num_vertices()
        << "\n  Number of Edges: " << _rmap->get_num_edges()
        << std::endl;
    DisplayCCStats(_os, *_rmap);
  }

  PrintAllStats(_os);
}


template <class GraphType>
void
StatClass::
DisplayCCStats(std::ostream& _os, GraphType& _g)  {
  #ifndef _PARALLEL
  // Compute the CC information.
  typedef typename GraphType::vertex_descriptor VID;

  stapl::sequential::vector_property_map<GraphType, size_t> cMap;
  std::vector<std::pair<size_t, VID>> ccs;

  stapl::sequential::get_cc_stats(_g, cMap, ccs);

  size_t ccnum = 0;
  _os << "\n  There are " << ccs.size() << " connected components:";
  for(auto cc : ccs)
    _os << "\n    CC[" << ccnum++ << "]: " << cc.first
        << " (vid " << cc.second << ")";
  _os << std::endl;

  #else
    _os << "WARNING: CCs not computed, called sequential implementation of CC stats \n" ;
  #endif
}

#endif
