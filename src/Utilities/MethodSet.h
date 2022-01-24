#ifndef PMPL_METHOD_SET_H_
#define PMPL_METHOD_SET_H_

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <boost/mpl/list.hpp>
#include <boost/mpl/next_prior.hpp>

#include "IOUtils.h"
#include "XMLNode.h"


////////////////////////////////////////////////////////////////////////////////
/// Creates new method instances from an XML node.
////////////////////////////////////////////////////////////////////////////////
template <typename Method>
struct MethodFactory final {

  ///@name Local Types
  ///@{

  typedef std::shared_ptr<Method> OwningPointer;

  ///@}
  ///@name Operator
  ///@{

  OwningPointer operator()(XMLNode& _node) const {
    return OwningPointer(new Method(_node));
  }

  ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// Defines basic method container class to hold methods (for classes like
/// DistanceMetricMethod, LocalPlannerMethod, etc).
///
/// MethodTypeList must be defined within templated class of MPTraits
///   e.g., Method = NeighborhoodFinderMethod
///         MethodTypeList = boost::mpl::list<BruteForceNF, BandsNF, ...>
///   e.g., Method = LocalPlannerMethod
///         MethodTypeList = boost::mpl::list<Straightline, RotateAtS, ...>
///
/// MethodSet first parses the MethodTypeList and adds all enumerated methods to
/// its universe of method types. Then, specific instantiations of those types
/// can be added to its map of available methods by calling 'AddMethod'.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits, typename Method>
class MethodSet final {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::MPLibrary                  MPLibrary;

    typedef Method*                                       MethodPointer;
    typedef std::shared_ptr<Method>                       OwningPointer;
    typedef typename std::map<std::string, OwningPointer> MethodMap;

    typedef std::function<OwningPointer(XMLNode&)>        FactoryType;
    typedef typename std::map<std::string, FactoryType>   FactoryMap;

    typedef typename MethodMap::iterator                  iterator;
    typedef typename MethodMap::const_iterator            const_iterator;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a MethodSet from a MethodTypeList defined in the MPTraits class.
    /// @param _mtl An instance of the method type list.
    /// @param _name The name of this method set.
    template <typename MethodTypeList>
    MethodSet(MPLibrary* const _p, const MethodTypeList& _mtl,
        const std::string& _name);

    ///@}
    ///@name Method Accessors
    ///@{

    /// Add the appropriate methods from an XML node.
    void ParseXML(XMLNode& _node);

    void AddMethod(XMLNode& _node);

    void AddMethod(MethodPointer _e, const std::string& _label);

    void AddMethod(OwningPointer _e, const std::string& _label);

    /// Get a method by label.
    /// @param _label The method label.
    /// @return The corresponding method pointer.
    MethodPointer GetMethod(const std::string& _label);

    /// Prepare all methods in this set for execution on the owning MPLibrary's
    /// current MPProblem.
    void Initialize();

    /// Display the instantiated methods.
    void Print(std::ostream& _os) const;

    ///@}
    ///@name Iteration
    ///@{
    /// Iterate over the instantiated methods.

    iterator begin() noexcept;
    iterator end() noexcept;
    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;

    ///@}

  private:

    ///@name Initialization Helpers
    ///@{
    /// Add a set of methods to the factory map recursively using template
    /// metaprogramming.

    /// Add the range [first, last) to the universe.
    template <typename First, typename Last>
    void AddToUniverse(First, Last);

    /// Base case for terminating recursion.
    template <typename Last>
    void AddToUniverse(Last, Last);

    ///@}
    ///@name Internal State
    ///@{

    MPLibrary* const m_library; ///< The owning planning library.

    std::string m_name;     ///< The name of this set of methods.

    FactoryMap m_universe;  ///< The set of allowed methods.
    MethodMap m_elements;   ///< The set of instantiated methods.

    ///@}

};

/*------------------------------- Method Set ---------------------------------*/

template <typename MPTraits, typename Method>
template <typename MethodTypeList>
MethodSet<MPTraits, Method>::
MethodSet(MPLibrary* const _p, const MethodTypeList& _mtl,
    const std::string& _name) : m_library(_p), m_name(_name) {
  AddToUniverse(typename boost::mpl::begin<MethodTypeList>::type(),
                typename boost::mpl::end<MethodTypeList>::type());
}


template <typename MPTraits, typename Method>
void
MethodSet<MPTraits, Method>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node)
    AddMethod(child);
}


template <typename MPTraits, typename Method>
void
MethodSet<MPTraits, Method>::
AddMethod(XMLNode& _node) {
  auto iter = m_universe.find(_node.Name());

  // Skip if method isn't in universe.
  if(iter == m_universe.end())
    return;

  OwningPointer e = iter->second(_node);
  AddMethod(e, e->m_label);
}


template <typename MPTraits, typename Method>
void
MethodSet<MPTraits, Method>::
AddMethod(MethodPointer _e, const std::string& _label) {
  AddMethod(OwningPointer(_e), _label);
}


template <typename MPTraits, typename Method>
void
MethodSet<MPTraits, Method>::
AddMethod(OwningPointer _e, const std::string& _label) {
  auto iter = m_universe.find(_e->m_name);

  // Throw exception if method isn't in universe.
  if(iter == m_universe.end())
    throw ParseException(WHERE) << "Method set '" << m_name << "' has no "
                                << "method type '" << _e->m_name << "'.";
  // Also throw if the label already exists.
  if(m_elements.count(_label))
    throw ParseException(WHERE) << "Method set '" << m_name << "' already has "
                                << "label '" << _label << "'.";
  // And also for empty labels.
  if(_label.empty())
    throw ParseException(WHERE) << "Method label cannot be empty.";

  _e->SetMPLibrary(m_library);
  _e->SetLabel(_label);
  m_elements[_label] = _e;
}


template <typename MPTraits, typename Method>
typename MethodSet<MPTraits, Method>::MethodPointer
MethodSet<MPTraits, Method>::
GetMethod(const std::string& _label) {
  // Find the method and ensure it exists.
  auto iter = m_elements.find(_label);

  if(iter == m_elements.end()) {
    std::ostringstream choices;
    Print(choices);
    throw RunTimeException(WHERE) << "Method '" << _label << "' does not exist "
                                  << "in set " << m_name << "."
                                  << choices.str();
  }

  return iter->second.get();
}


template <typename MPTraits, typename Method>
void
MethodSet<MPTraits, Method>::
Initialize() {
  for(auto& elem : m_elements)
    elem.second->Initialize();
}


template <typename MPTraits, typename Method>
void
MethodSet<MPTraits, Method>::
Print(std::ostream& _os) const {
  size_t count = 0;

  _os << "\n" << m_name << " has these methods available:";
  for(const auto& elem : m_elements)
    _os << "\n\t"
        << ++count << ") '" << elem.first << "' (" << elem.second->m_name << ")";
  _os << std::endl;
}

/*--------------------------------- Iteration --------------------------------*/

template <typename MPTraits, typename Method>
inline
typename MethodSet<MPTraits, Method>::iterator
MethodSet<MPTraits, Method>::
begin() noexcept {
  return m_elements.begin();
}


template <typename MPTraits, typename Method>
inline
typename MethodSet<MPTraits, Method>::iterator
MethodSet<MPTraits, Method>::
end() noexcept {
  return m_elements.end();
}


template <typename MPTraits, typename Method>
inline
typename MethodSet<MPTraits, Method>::const_iterator
MethodSet<MPTraits, Method>::
begin() const noexcept {
  return m_elements.begin();
}


template <typename MPTraits, typename Method>
inline
typename MethodSet<MPTraits, Method>::const_iterator
MethodSet<MPTraits, Method>::
end() const noexcept {
  return m_elements.end();
}

/*-------------------------- Initialization Helpers --------------------------*/

template <typename MPTraits, typename Method>
template <typename First, typename Last>
void
MethodSet<MPTraits, Method>::
AddToUniverse(First, Last) {
  using FirstType = typename boost::mpl::deref<First>::type;
  FirstType first;
  m_universe[first.m_name] = MethodFactory<FirstType>();
  AddToUniverse(typename boost::mpl::next<First>::type(), Last());
}


template <typename MPTraits, typename Method>
template <typename Last>
void
MethodSet<MPTraits, Method>::
AddToUniverse(Last, Last) {}

/*----------------------------------------------------------------------------*/

#endif
