#ifndef XML_NODE_H_
#define XML_NODE_H_

#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

// Tell TinyXML to use the stl.
#ifndef TIXML_USE_STL
#define TIXML_USE_STL
#endif

#include "tinyxml.h"

#include "PMPLExceptions.h"


////////////////////////////////////////////////////////////////////////////////
/// Wrapper class for XML parsing with TinyXML.
///
/// @ingroup IOUtils
/// @details This is a wrapper class for XML handling with TinyXML. It is read
///          only and supports trivial XML parsing.
////////////////////////////////////////////////////////////////////////////////
class XMLNode {

  public:

    ///@name Construction
    ///@{

    /// Construct an XML node object from an XML file.
    /// @param _filename XML Filename
    /// @param _desiredNode Desired XML Node to make root of tree
    ///
    /// Will throw ParseException when \p _desiredNode cannot be found or
    /// \p _filename is poorly formed input
    XMLNode(const std::string& _filename, const std::string& _desiredNode);

  private:

    /// Private constructor for use within BuildChildVector
    /// @param _node New TiXMLNode
    /// @param _filename XML filename
    /// @param _doc TiXmlDocument from tree's root node
    XMLNode(TiXmlNode* _node, const std::string& _filename,
        std::shared_ptr<TiXmlDocument> _doc);

    ///@}

  public:

    ///@name Iteration
    ///@{

    typedef std::vector<XMLNode>::iterator iterator;

    /// Get an iterator to this node's first child.
    iterator begin();

    /// Get an iterator to this node's last child.
    iterator end();

    ///@}
    ///@name Metadata Accessors
    ///@{

    /// Get the XMLNode name.
    const std::string& Name() const;

    /// Get the XML filename.
    const std::string& Filename() const;

    /// Get the directory path containing the XML file.
    std::string GetPath() const;

    ///@}
    ///@name Content Accessors
    ///@{

    /// Get the text between opening and closing tags.
    std::string GetText() const;

    ///@}
    ///@name Attribute Parsing
    ///@{

    /// Read XML attribute.
    /// @tparam T Type of attribute
    /// @param _name Name of attribute
    /// @param _req Is attribute required
    /// @param _default Default value of attribute
    /// @param _min Minimum value of attribute
    /// @param _max Maximum value of attribute
    /// @param _desc Description of attribute
    /// @return Value of attribute
    ///
    /// Reads XML attribute value with \p _name. If _req is specified and no
    /// attribute is given, \p _default is returned, otherwise input value is
    /// required to be in the range [\p _min, \p _max]. Otherwise, an error is
    /// reported and \p _desc is shown to the user.
    template <typename T>
    T Read(const std::string& _name, const bool _req, const T& _default,
        const T& _min, const T& _max, const std::string& _desc);

    /// Read XML boolean attribute
    /// @param _name Name of attribute
    /// @param _req Is attribute required
    /// @param _default Default value of attribute
    /// @param _desc Description of attribute
    /// @return Value of attribute
    ///
    /// Reads XML attribute value with \p _name. If _req is specified and no
    /// attribute is given, \p _default is returned. Otherwise, an error is
    /// reported and \p _desc is shown to the user.
    bool Read(const std::string& _name, const bool _req, const bool _default,
        const std::string& _desc);

    /// Read XML string attribute
    /// @return Value of attribute
    ///
    /// Calls string version of function to avoid confusion with bool -> const
    /// char* conversion in compile.
    std::string Read(const std::string& _name, const bool _req,
        const char* _default, const std::string& _desc);

    /// Read XML string attribute
    /// @param _name Name of attribute
    /// @param _req Is attribute required
    /// @param _default Default value of attribute
    /// @param _desc Description of attribute
    /// @return Value of attribute
    ///
    /// Reads XML attribute value with \p _name. If _req is specified and no
    /// attribute is given, \p _default is returned. Otherwise, an error is
    /// reported and \p _desc is shown to the user.
    std::string Read(const std::string& _name,
        const bool _req,
        const std::string& _default,
        const std::string& _desc);

    ///@}
    ///@name Parsing Flow
    ///@{

    /// Ignore unrequested node/attribute errors for this node.
    void Ignore();

    /// Report warnings for XML tree rooted at this node
    /// @param _warningsAsErrors True will throw exceptions for warnings
    ///
    /// To be called after parsing phase. This will report warnings throughout
    /// entire XML document. Should only be called on root XML node. Warnings to
    /// be reported:
    ///   - unknown/unparsed nodes
    ///   - unrequested attribues
    void WarnAll(const bool _warningsAsErrors = false);

    /// Generate string describing where the node is
    /// @return String representing where node is
    ///
    /// To be used with PMPLExceptions, specifically ParseException. Gives
    /// string with filename, row (line number), and column of XMLNode.
    std::string Where() const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Generate string describing where the node is.
    /// @param _f Filename
    /// @param _l Line number
    /// @param _c Column number
    /// @param _name Report name of node
    /// @return String representing where node is
    ///
    /// To be used with PMPLExceptions, specifically ParseException. Gives
    /// string with filename, name, row (line number), and column of XMLNode.
    std::string Where(const std::string& _f, const int _l, const int _c,
        const bool _name = true) const;

    /// Generate XMLNodes for all children
    ///
    /// Builds the internal vector of children, used when iterating over
    /// the children. This vector is only built with ELEMENT type nodes.
    void BuildChildVector();

    /// Return error report for attribute being the wrong type
    /// @param _name Name of attribute
    /// @param _desc Description of attribute
    /// @return Error report
    std::string AttrWrongType(const std::string& _name, const std::string& _desc)
        const;

    /// Return error report for missing attribute
    /// @param _name Name of attribute
    /// @param _desc Description of attribute
    /// @return Error report
    std::string AttrMissing(const std::string& _name, const std::string& _desc)
        const;

    /// Return error report for attribute being in an invalid range
    /// @tparam T Type of attribute
    /// @param _name Name of attribute
    /// @param _desc Description of attribute
    /// @param _min Minimum value of attribute
    /// @param _max Maximum value of attribute
    /// @param _val The specified value
    /// @return Error report
    template <typename T>
    std::string AttrInvalidBounds(const std::string& _name,
        const std::string& _desc, const T& _min, const T& _max, const T& _val)
        const;

    /// Recursive function computing whether nodes have been accessed
    void ComputeAccessed();

    /// Recursive function reporting all unknown/unparsed nodes and unrequested
    /// attributes.
    /// @param[out] _anyWarnings Initially should be false, and stores whether
    ///                          any warnings have been reported
    void WarnAllRec(bool& _anyWarnings);

    /// Report unknown node warning to cerr.
    void WarnUnknownNode();

    /// Report unrequested attributes to cerr.
    bool WarnUnrequestedAttributes();

    ///@}
    ///@name Internal State
    ///@{

    TiXmlNode* m_node{nullptr};      ///< TiXmlNode
    bool m_childBuilt{false};        ///< Have children been parsed into nodes?
    bool m_accessed{false};          ///< Has this node been accessed or not?
    std::vector<XMLNode> m_children; ///< Children of node
    std::unordered_set<std::string> m_reqAttributes; ///< Requested attributes.
    std::string m_filename;          ///< XML Filename

    /// Overall TiXmlDocument. Can be shared by child nodes.
    std::shared_ptr<TiXmlDocument> m_doc;

    ///@}
};

/*---------------------------- Templated Members -----------------------------*/

template <typename T>
T
XMLNode::
Read(const std::string& _name, const bool _req, const T& _default, const T& _min,
    const T& _max, const std::string& _desc) {
  m_accessed = true;
  m_reqAttributes.insert(_name);
  T toReturn;

  int qr = m_node->ToElement()->QueryValueAttribute(_name, &toReturn);
  switch(qr) {
    case TIXML_WRONG_TYPE:
      throw ParseException(Where(), AttrWrongType(_name, _desc));
      break;
    case TIXML_NO_ATTRIBUTE:
      {
        if(_req)
          throw ParseException(Where(), AttrMissing(_name, _desc));
        else
          toReturn = _default;
        break;
      }
    case TIXML_SUCCESS:
      {
        if(toReturn < _min || toReturn > _max)
          throw ParseException(Where(),
              AttrInvalidBounds(_name, _desc, _min, _max, toReturn));
        break;
      }
    default:
      throw RunTimeException(WHERE, "Logic shouldn't be able to reach this.");
  }

  return toReturn;
}


template <typename T>
std::string
XMLNode::
AttrInvalidBounds(const std::string& _name, const std::string& _desc,
    const T& _min, const T& _max, const T& _val) const {
  std::ostringstream oss;
  oss << "Invalid value for attribute '" << _name << "'."
      << "\n\tAttribute description: " << _desc << "."
      << "\n\tValid range: [" << _min << ", " << _max << "]"
      << "\n\tValue specified: " << _val;
  return oss.str();
}

/*----------------------------------------------------------------------------*/

#endif
