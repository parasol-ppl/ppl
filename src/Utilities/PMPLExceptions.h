#ifndef PMPL_EXCEPTIONS_H_
#define PMPL_EXCEPTIONS_H_

#include "Utilities/RuntimeUtils.h"

#include "nonstd/exception.h"

#include <string>
#include <sstream>


////////////////////////////////////////////////////////////////////////////////
/// Base exception class for PMPL.
///
/// PMPLExceptions are objects which methods can throw for their various errors.
/// These aid users in identifying where and how errors occur. The base class
/// provides an output stream operator which allows one to treat the object like
/// an ostringstream.
/// @ingroup Exceptions
////////////////////////////////////////////////////////////////////////////////
struct PMPLException : public nonstd::exception {

  PMPLException(const std::string& _type, const std::string& _where,
      const std::string& _message) {
    *this << "\nError:\n\t" << _type
          << "\nWhere:\n\t" << _where
          << "\nWhy:\n\t" << _message
          << "\n";
  }

};


////////////////////////////////////////////////////////////////////////////////
/// Exception for parsing errors.
/// @ingroup Exceptions
////////////////////////////////////////////////////////////////////////////////
struct ParseException : public PMPLException {

  ParseException(const std::string& _where, const std::string& _message = "") :
      PMPLException("Parse Exception", _where, _message) {}

};


////////////////////////////////////////////////////////////////////////////////
/// Exception for file output errors.
/// @ingroup Exceptions
////////////////////////////////////////////////////////////////////////////////
struct WriteException : public PMPLException {

  WriteException(const std::string& _where, const std::string& _message = "") :
      PMPLException("Write Exception", _where, _message) {}

};


////////////////////////////////////////////////////////////////////////////////
/// Exception for runtime errors.
/// @ingroup Exceptions
////////////////////////////////////////////////////////////////////////////////
struct RunTimeException : public PMPLException {

  RunTimeException(const std::string& _where, const std::string& _message = "") :
      PMPLException("Runtime Exception", _where, _message) {}

};


////////////////////////////////////////////////////////////////////////////////
/// Exception for features which are not yet implemented.
/// @ingroup Exceptions
////////////////////////////////////////////////////////////////////////////////
struct NotImplementedException : public PMPLException {

  NotImplementedException(const std::string& _where) :
      PMPLException("Not Implemented Exception", _where,
                    "This feature is not implemented. ") {}

};

#endif
