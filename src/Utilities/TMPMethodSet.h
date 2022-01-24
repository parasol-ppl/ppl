#ifndef TMPMethod_SET_H_
#define TMPMethod_SET_H_

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <boost/mpl/list.hpp>
#include <boost/mpl/next_prior.hpp>

#include "IOUtils.h"
#include "XMLNode.h"

class TMPLibrary;
////////////////////////////////////////////////////////////////////////////////
/// @brief Creates new TMPMethod instances from an XML node.
////////////////////////////////////////////////////////////////////////////////
template <typename TMPMethod>
struct TMPMethodFactory {

  ///@name Local Types
  ///@{

  typedef std::shared_ptr<TMPMethod> TMPMethodPointer;

  ///@}
  ///@name Operator
  ///@{

  TMPMethodPointer operator()(XMLNode& _node) const {
    return TMPMethodPointer(new TMPMethod(_node));
  }

  ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// Defines basic TMPMethod container class to hold TMPMethods (for classes like
/// DistanceMetricTMPMethod, LocalPlannerTMPMethod, etc).
///
/// TMPMethodTypeList must be defined within templated class of MPTraits
///   e.g., TMPMethod = NeighborhoodFinderTMPMethod
///         TMPMethodTypeList = boost::mpl::list<BruteForceNF, BandsNF, ...>
///   e.g., TMPMethod = LocalPlannerTMPMethod
///         TMPMethodTypeList = boost::mpl::list<Straightline, RotateAtS, ...>
///
/// TMPMethodSet first parses the TMPMethodTypeList and adds all enumerated TMPMethods to
/// its universe of TMPMethod types. Then, specific instantiations of those types
/// can be added to its map of available TMPMethods by calling 'AddTMPMethod'.
////////////////////////////////////////////////////////////////////////////////
template<typename TMPMethod>
class TMPMethodSet {

  public:

    ///@name Local Types
    ///@{

    typedef std::shared_ptr<TMPMethod>                       TMPMethodPointer;
    typedef typename std::map<std::string, TMPMethodPointer> TMPMethodMap;

    typedef std::function<TMPMethodPointer(XMLNode&)>        FactoryType;
    typedef typename std::map<std::string, FactoryType>      FactoryMap;

    typedef typename TMPMethodMap::iterator                  iterator;
    typedef typename TMPMethodMap::const_iterator            const_iterator;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a TMPMethodSet from a TMPMethodTypeList defined in the MPTraits class.
    /// @param _mtl An instance of the TMPMethod type list.
    /// @param _name The name of this TMPMethod set.
    template <typename TMPMethodTypeList>
    TMPMethodSet(TMPLibrary* const _p, const TMPMethodTypeList& _mtl,
        const std::string& _name);

    ///@}
    ///@name TMPMethod Accessors
    ///@{

    /// Add the appropriate TMPMethods from an XML node.
    void ParseXML(XMLNode& _node);

    void AddMethod(XMLNode& _node);

    void AddMethod(TMPMethodPointer _e, const std::string& _label);

    /// Get a TMPMethod by label.
    /// @param _label The TMPMethod label.
    /// @return The corresponding TMPMethod pointer.
    TMPMethodPointer GetMethod(const std::string& _label);

    /// Find a TMPMethod iterator.
    /// @param _label The TMPMethod label.
    /// @return The iterator to that TMPMethod, or nullptr if it is not found.
    iterator FindMethod(std::string _label);

    /// Prepare all TMPMethods in this set for execution on the owning MPLibrary's
    /// current MPProblem.
    void Initialize();

    /// Display the instantiated TMPMethods.
    void Print(std::ostream& _os) const;

    ///@}
    ///@name Iteration
    ///@{
    /// Iterate over the instantiated TMPMethods.

    iterator begin() noexcept;
    iterator end() noexcept;
    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;

    ///@}

  private:

    ///@name Initialization Helpers
    ///@{
    /// Add a set of TMPMethods to the factory map recursively using template
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

    TMPLibrary* m_tmpLibrary; ///< The owning tmp planning library.

    std::string m_name;     ///< The name of this set of TMPMethods.
    std::string m_default;  ///< The name of the default TMPMethod in this set.

    FactoryMap m_universe;  ///< The set of allowed TMPMethods.
    TMPMethodMap m_elements;   ///< The set of instantiated TMPMethods.

    ///@}

};

/*----------------------------------------------------------------------------*/
#include "TMPLibrary/TMPLibrary.h"

/*------------------------------- TMPMethod Set ---------------------------------*/

template <typename TMPMethod>
template <typename TMPMethodTypeList>
TMPMethodSet<TMPMethod>::
TMPMethodSet(TMPLibrary* const _p, const TMPMethodTypeList& _mtl,
    const std::string& _name) : m_tmpLibrary(_p), m_name(_name) {
  AddToUniverse(typename boost::mpl::begin<TMPMethodTypeList>::type(),
                typename boost::mpl::end<TMPMethodTypeList>::type());
}


template <typename TMPMethod>
void
TMPMethodSet<TMPMethod>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node)
    AddMethod(child);
}


template <typename TMPMethod>
void
TMPMethodSet<TMPMethod>::
AddMethod(XMLNode& _node) {
  auto iter = m_universe.find(_node.Name());

  // Skip if TMPMethod isn't in universe.
  if(iter == m_universe.end())
    return;

  TMPMethodPointer e = iter->second(_node);
  AddMethod(e, e->m_label);
}


template <typename TMPMethod>
void
TMPMethodSet<TMPMethod>::
AddMethod(TMPMethodPointer _e, const std::string& _label) {
  auto iter = m_universe.find(_e->m_name);

  // Throw exception if TMPMethod isn't in universe.
  if(iter == m_universe.end())
    throw ParseException(WHERE, "TMPMethod '" + _e->m_name +
        "' is not contained within the motion planning universe.");

  _e->SetTMPLibrary(m_tmpLibrary);
  _e->SetLabel(_label);
  if(m_elements.empty())
    m_default = _label;
  if(m_elements.find(_label) == m_elements.end())
    m_elements[_label] = _e;
  else
    std::cerr << "\nWarning, TMPMethod list already has a pointer associated with "
         << "\"" << _label << "\", not added\n";
}


template <typename TMPMethod>
typename TMPMethodSet<TMPMethod>::TMPMethodPointer
TMPMethodSet<TMPMethod>::
GetMethod(const std::string& _label) {
  TMPMethodPointer element = (_label == "") ? m_elements[m_default] :
                                           m_elements[_label];
  if(element.get() == nullptr) {
    std::string err = "Element '" + _label + "' does not exist in " + m_name +
        ". Choices are: ";
    for(auto& elem : m_elements)
      if(elem.second.get())
        err += " '" + elem.first + "',";
    err.pop_back();
    throw RunTimeException(WHERE, err);
  }
  return element;
}


template <typename TMPMethod>
typename TMPMethodSet<TMPMethod>::iterator
TMPMethodSet<TMPMethod>::
FindMethod(std::string _label) {
  if(_label == "")
    _label = m_default;
  return m_elements.find(_label);
}


template <typename TMPMethod>
void
TMPMethodSet<TMPMethod>::
Initialize() {
  for(auto& elem : m_elements)
    elem.second->Initialize();
}


template <typename TMPMethod>
void
TMPMethodSet<TMPMethod>::
Print(std::ostream& _os) const {
  size_t count = 0;

  _os << "\n" << m_name << " has these TMPMethods available::\n\n";

  for(auto& elem : m_elements) {
    _os << ++count << ") \"" << elem.first << "\" (" << elem.second->m_name
        << ")\n";
    elem.second->Print(_os);
    _os << std::endl;
  }
  _os << std::endl;
}

/*--------------------------------- Iteration --------------------------------*/

template <typename TMPMethod>
typename TMPMethodSet<TMPMethod>::iterator
TMPMethodSet<TMPMethod>::
begin() noexcept {
  return m_elements.begin();
}


template <typename TMPMethod>
typename TMPMethodSet<TMPMethod>::iterator
TMPMethodSet<TMPMethod>::
end() noexcept {
  return m_elements.end();
}


template <typename TMPMethod>
typename TMPMethodSet<TMPMethod>::const_iterator
TMPMethodSet<TMPMethod>::
begin() const noexcept {
  return m_elements.begin();
}


template <typename TMPMethod>
typename TMPMethodSet<TMPMethod>::const_iterator
TMPMethodSet<TMPMethod>::
end() const noexcept {
  return m_elements.end();
}

/*-------------------------- Initialization Helpers --------------------------*/

template <typename TMPMethod>
template <typename First, typename Last>
void
TMPMethodSet<TMPMethod>::
AddToUniverse(First, Last) {
  using FirstType = typename boost::mpl::deref<First>::type;
  FirstType first;
  m_universe[first.GetName()] = TMPMethodFactory<FirstType>();
  AddToUniverse(typename boost::mpl::next<First>::type(), Last());
}


template <typename TMPMethod>
template <typename Last>
void
TMPMethodSet<TMPMethod>::
AddToUniverse(Last, Last) {}

#endif
