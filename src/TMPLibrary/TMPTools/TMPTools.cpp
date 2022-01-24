#include "TMPTools.h"

TMPTools::
TMPTools(TMPLibrary* const _tmpLibrary) : m_tmpLibrary(_tmpLibrary) {}

void
TMPTools::
ParseXML(XMLNode& _node) {
  /*for(auto& child : _node){
  }*/
}

template <typename Utility>
void
TMPTools::
SetUtility(const std::string& _label, Utility* _utility,
           LabelMap<Utility>& _map) {
  // Set the library pointer.
  _utility->SetTMPLibrary(m_tmpLibrary);

  // Check if this label is already in use.
  auto iter = _map.find(_label);
  const bool alreadyExists = iter != _map.end();

  // If the label alrady exists, we need to release the previous utility first.
  if(alreadyExists) {
    delete iter->second;
    iter->second = _utility;
  }
  else {
    _map.insert({_label, _utility});
  }
}

template<typename Utility>
Utility*
TMPTools::
GetUtility(const std::string& _label, const LabelMap<Utility>& _map) {
  try {
    return _map.at(_label);
  }
  catch(const std::out_of_range&) {
    Utility dummy;
    throw RunTimeException(WHERE) << "Requested " << dummy.GetName() <<
                            " '" << _label << "' does not exist.";
  }
  catch(const std::exception& _e) {
    Utility dummy;
    throw RunTimeException(WHERE) << "Error when fetching " << dummy.GetName() <<
                            " '" << _label << "': " << _e.what();
  }
  catch(...) {
    Utility dummy;
    throw RunTimeException(WHERE) << "Error when fetching " << dummy.GetName() <<
                            " '" << _label << "': (unkown)";
  }
}
