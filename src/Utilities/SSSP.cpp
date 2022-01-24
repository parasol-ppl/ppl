#include "SSSP.h"

#include "Utilities/PMPLExceptions.h"


/*----------------------------------- Debug ----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const SSSPTermination& _t) {
  switch(_t) {
    case SSSPTermination::Continue:
      return _os << "Continue";
    case SSSPTermination::EndBranch:
      return _os << "EndBranch";
    case SSSPTermination::EndSearch:
      return _os << "EndSearch";
    default:
      throw RunTimeException(WHERE) << "Unknown termination criterion '"
                                    << int(_t) << "'."
                                    << std::endl;
  }
}

/*----------------------------------------------------------------------------*/
