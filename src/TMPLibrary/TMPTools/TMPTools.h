#ifndef _PPL_TMP_TOOLS_H_
#define _PPL_TMP_TOOLS_H_

#include <iostream>
#include <unordered_map>

#include "TMPLibrary/TMPLibrary.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

class TMPTools {
  public:

    ///@name Local Types
    ///@{

    template <typename Utility>
    using LabelMap = std::unordered_map<std::string, Utility*>;

    ///@}
    ///@name Construction
    ///@{

    TMPTools() = default;

    TMPTools(TMPLibrary* _tmpLibrary);

    virtual ~TMPTools() = default;

    void ParseXML(XMLNode& _node);

    ///@}

  private:

    ///@name Helpers
    ///@{

    template <typename Utility>
    Utility* GetUtility(const std::string& _label,
        const LabelMap<Utility>& _map);

    template <typename Utility>
    void SetUtility(const std::string& _label, Utility* _utility,
        LabelMap<Utility>& _map);

    ///@}
    ///@name Internal State
    ///@{

    TMPLibrary* const m_tmpLibrary; ///< The owning library.

    ///@}
};
/*----------------------------------------------------------------------------*/

#endif
