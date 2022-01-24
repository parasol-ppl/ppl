#include "MetricUtils.h"

#include "Utilities/PMPLExceptions.h"


/*-------------------------------- StatClass ---------------------------------*/

StatClass::
StatClass() {
  ClearStats();
}


void
StatClass::
ClearStats() {
  m_numCollDetCalls.clear();
  m_lpInfo.clear();
  m_samplerInfo.clear();
  m_collDetCountByName.clear();
  m_isCollByName.clear();
  m_isCollTotal = 0;
}


int
StatClass::
IncNumCollDetCalls(const std::string& _cdName, const std::string& _callName) {
  m_numCollDetCalls[_cdName]++;
  // If a caller's name was provided
  // then increment the verification counter
  // with that name.
  m_collDetCountByName[_callName]++;
  return m_numCollDetCalls[_cdName];
}


size_t
StatClass::
GetIsCollTotal() {
  return m_isCollTotal;
}


void
StatClass::
IncCfgIsColl(const std::string& _callName) {
  m_isCollByName[_callName]++;
  m_isCollTotal++;
}


int
StatClass::
IncLPConnections(const std::string& _lpName, int _incr) {
  return std::get<1>(m_lpInfo[_lpName]) += _incr;
}


int
StatClass::
IncLPAttempts(const std::string& _lpName, int _incr) {
  return std::get<0>(m_lpInfo[_lpName]) += _incr;
}


int
StatClass::
IncLPCollDetCalls(const std::string& _lpName, int _incr) {
  return std::get<2>(m_lpInfo[_lpName]) += _incr;
}


void
StatClass::
IncNodesGenerated(const std::string& _samplerName, size_t _incr) {
  m_samplerInfo[_samplerName].second += _incr;
}


void
StatClass::
IncNodesAttempted(const std::string& _samplerName, size_t _incr) {
  m_samplerInfo[_samplerName].first += _incr;
}


void
StatClass::
StartClock(const std::string& _name) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  if(m_clockMap.find(_name) == m_clockMap.end())
    m_clockMap[_name].SetName(_name);
  m_clockMap[_name].StartClock();
}


void
StatClass::
StopClock(const std::string& _name) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  m_clockMap[_name].StopClock();
}


void
StatClass::
StopPrintClock(const std::string& _name, std::ostream& _os) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  m_clockMap[_name].StopPrintClock(_os);
}


void
StatClass::
PrintClock(const std::string& _name, std::ostream& _os) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  m_clockMap[_name].PrintClock(_os);
}


void
StatClass::
ClearClock(const std::string& _name) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  m_clockMap[_name].ClearClock();
}


double
StatClass::
GetSeconds(const std::string& _name) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  return m_clockMap[_name].GetSeconds();
}


int
StatClass::
GetUSeconds(const std::string& _name) {
  if(_name == "")
    throw RunTimeException(WHERE, "Empty clock name requested.");

  return m_clockMap[_name].GetUSeconds();
}


double
StatClass::
GetStat(const std::string& _s) {
  return m_stats[_s];
}


void
StatClass::
SetStat(const std::string& _s, const double _v) {
  m_stats[_s] = _v;
}


void
StatClass::
IncStat(const std::string& _s, const double _v) {
  m_stats[_s] += _v;
}


Average<double>&
StatClass::
GetAverage(const std::string& _s) {
  return m_averages[_s];
}


std::vector<double>&
StatClass::
GetHistory(const std::string& _s) {
  return m_histories[_s];
}


void
StatClass::
AddToHistory(const std::string& _s, double _v) {
  m_histories[_s].push_back(_v);
}


void
StatClass::
WriteHistory(const std::string& _s) {
  std::ofstream ofs(m_auxFileDest + "." + _s + ".hist");
  for(const auto& item : m_histories[_s])
    ofs << std::scientific << std::setprecision(16) << item << std::endl;
}


void
StatClass::
SetAuxDest(const std::string& _s) {
  m_auxFileDest = _s;
}


void
StatClass::
PrintAllStats(std::ostream& _os) {
  // Output sampler statistics.
  if(!m_samplerInfo.empty()) {
    size_t totalAttempts = 0, totalGenerated = 0;
    _os << "\n\n"
        << std::setw(60) << std::left  << "Sampler Statistics"
        << std::setw(10) << std::right << "Attempts"
        << std::setw(10) << std::right << "Successes"
        << "\n\n";

    for(const auto& info : m_samplerInfo) {
      _os << "  "
          << std::setw(58) << std::left  << info.first
          << std::setw(10) << std::right << info.second.first
          << std::setw(10) << std::right << info.second.second
          << std::endl;
      totalAttempts += info.second.first;
      totalGenerated += info.second.second;
    }

    if(totalAttempts > 0)
      _os << "  "
          << std::setw(58) << std::left << "All Samplers"
          << std::setw(10) << std::right << totalAttempts
          << std::setw(10) << std::right << totalGenerated
          << "\n  Success Rate: "
          << std::setprecision(3) << totalGenerated * 100.0 / totalAttempts << "%"
          << std::endl;
  }

  // Output local planner statistics.
  if(!m_lpInfo.empty()) {
    _os << "\n\n"
        << std::setw(44) << std::left  << "Local Planner Statistics"
        << std::setw(12) << std::right << "Attempts"
        << std::setw(12) << std::right << "Connections"
        << std::setw(12) << std::right << "CD Calls"
        << "\n\n";

    for(const auto& info : m_lpInfo)
      _os << "  "
          << std::setw(42) << std::left  << info.first
          << std::setw(12) << std::right << std::get<0>(info.second)
          << std::setw(12) << std::right << std::get<1>(info.second)
          << std::setw(12) << std::right << std::get<2>(info.second)
          << std::endl;
  }

  // Ouput CD statistics.
  if(!m_isCollByName.empty()) {
    _os << "\n\n"
        << std::setw(70) << std::left  << "Collision Detection Calls"
        << std::setw(10) << std::right << "Count"
        << "\n\n";
    for(const auto& info : m_isCollByName)
      _os << "  "
          << std::setw(68) << std::left  << info.first
          << std::setw(10) << std::right << info.second
          << std::endl;
    _os << "  "
        << std::setw(68) << std::left  << "All Collision Detectors"
        << std::setw(10) << std::right << m_isCollTotal
        << std::endl;
  }

  // Output other statistics.
  if(!m_stats.empty()) {
    _os << "\n\n"
        << std::setw(70) << std::left  << "Other Statistics"
        << std::setw(10) << std::right << "Value"
        << "\n\n";
    for(const auto& stat : m_stats)
      _os << "  "
          << std::setw(68) << std::left  << stat.first
          << std::setw(10) << std::right << stat.second
          << std::endl;
  }

  // Output averages.
  if(!m_averages.empty()) {
    _os << "\n\n"
        << std::setw(60) << std::left  << "Averages"
        << std::setw(10) << std::right << "Value"
        << std::setw(10) << std::right << "Count"
        << "\n\n";
    for(const auto& stat : m_averages)
      _os << "  "
          << std::setw(58) << std::left  << stat.first
          << std::setw(10) << std::right << stat.second.Get()
          << std::setw(10) << std::right << stat.second.Count()
          << std::endl;
  }

  // Output clocks.
  _os << "\n\n"
      << std::setw(66) << std::left  << "Clock Name"
      << std::setw(14) << std::right << "Time (Seconds)"
      << "\n\n";
  for(auto& clock : m_clockMap)
    _os << "  "
        << std::setw(68) << std::left << clock.first
        << std::setw(10) << std::right << clock.second.GetSeconds()
        << std::endl;

  // Write out history files.
  for(const auto& hist : m_histories)
    WriteHistory(hist.first);
}

/*------------------------------- MethodTimer --------------------------------*/

MethodTimer::
MethodTimer(StatClass* const _stats, const std::string& _label)
  : m_stats(_stats), m_label(_label) {
  m_stats->StartClock(m_label);
}


MethodTimer::
~MethodTimer() {
  m_stats->StopClock(m_label);
}

/*----------------------------------------------------------------------------*/
