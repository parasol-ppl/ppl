#ifndef CONNECTION_H
#define CONNECTION_H

#include <array>
#include <memory>
#include <string>
#include <utility>

#include "Transformation.h"
using namespace mathtool;

#include "DHParameters.h"
#include "Utilities/IOUtils.h"
#include "Geometry/Boundaries/Range.h"

class Body;
class MultiBody;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Connection information between two Bodys in a MultiBody.
///
/// This class stores information about connection from one body to another one.
/// The information stored in this class includes:
///   - Connection type
///   - Two Body instances
///   - Transform from frame of body1 to DH-Frame
///   - DHParameter
///   - Transform from DH-Frame to frame of body2
///
/// @TODO Need to generalize the connection relationship to handle closed
///       chains and multiple backward connections.
////////////////////////////////////////////////////////////////////////////////
class Connection final {

  public:

    ///@name Local Types
    ///@{

    /// The supported connection types.
    enum class JointType {
      Revolute,   ///< 1 DOF
      Spherical,  ///< 2 DOF
      NonActuated ///< 0 DOF
    };

    ///@}
    ///@name Construction
    ///@{

    /// @param _owner MultiBody who owns this Connection
    Connection(MultiBody* const _owner);

    /// Parse connection info from an XML node.
    /// @param _owner MultiBody who owns this Connection
    /// @param _node The input XML node to read.
    Connection(MultiBody* const _owner, XMLNode& _node);

    /// Copying a connection does not copy the multibody and body pointers, as
    /// this would not be a usable object. Call SetBodies to attach a newly
    /// copied connection to another multibody.
    Connection(const Connection&);  ///< Copy.
    Connection(Connection&&);       ///< Move.

    ///@}
    ///@name Assignment
    ///@{

    Connection& operator=(const Connection&);  ///< Copy.
    Connection& operator=(Connection&&);       ///< Move.

    ///@}
    ///@name I/O
    ///@{

    /// Parse connection info from an old-style env/robot file.
    /// @param _is The input stream to read.
    /// @param _cbs The input counting stream buffer.
    void Read(istream& _is, CountingStreamBuffer& _cbs);

    /// Set the free bodies which are joined by this connection and call their
    /// link functions.
    /// @param _owner The owning multibody, or null to use the current.
    /// @param _parentIndex The parent body index.
    /// @param _childIndex The child body index.
    void SetBodies(MultiBody* const _owner, const size_t _parentIndex,
        const size_t _childIndex);

    /// This overload assumes that the parent/child indexes have already been
    /// set.
    /// @overload
    void SetBodies(MultiBody* const _owner = nullptr);

    /// Set the free bodies which are adjacent without an explicit connection.
    /// @param _owner The owning multibody.
    /// @param _parentIndex The parent body index.
    /// @param _childIndex The child body index.
    void SetAdjacentBodies(MultiBody* const _owner, const size_t _firstIndex,
        const size_t _secondIndex);

    ///@}
    ///@name Joint Information
    ///@{

    /// Get the connection type.
    JointType GetConnectionType() const noexcept;

    /// Check if this is a revolute joint.
    bool IsRevolute() const noexcept;

    /// Check if this is a spherical joint.
    bool IsSpherical() const noexcept;

    /// Check if this is a non-actuated joint.
    bool IsNonActuated() const noexcept;

    /// Get a joint range.
    /// @param _i The range to get (0 normally, 1 for second spherical range).
    /// @return The corresponding range object.
    const Range<double>& GetJointRange(const size_t _i) const noexcept;

    /// Set a joint range. Note this will NOT re-initialize the owning robot's
    /// configuration space.
    /// @param _i The range to set (0 normally, 1 for second spherical range).
    /// @param _r The new joint range.
    void SetJointRange(const size_t _i, const Range<double>& _r) noexcept;

    ///@}
    ///@name Body information
    ///@{

    /// Get a pointer to the child Body.
    const Body* GetPreviousBody() const noexcept;
    Body* GetPreviousBody() noexcept;

    /// Get the index of the parent Body within the parent multibody.
    size_t GetPreviousBodyIndex() const noexcept;

    /// Get a pointer to the child Body.
    const Body* GetNextBody() const noexcept;
    Body* GetNextBody() noexcept;

    /// Get the index of the child Body within the parent multibody.
    size_t GetNextBodyIndex() const noexcept;

    ///@}
    ///@name Transformation information
    ///@{

    /// @return DH frame description
    DHParameters& GetDHParameters() noexcept;
    const DHParameters& GetDHParameters() const noexcept;

    /// @return Transformation to second body
    Transformation& GetTransformationToBody2() noexcept;
    const Transformation& GetTransformationToBody2() const noexcept;

    /// @return Transformation to DH frame
    Transformation& GetTransformationToDHFrame() noexcept;
    const Transformation& GetTransformationToDHFrame() const noexcept;

    ///@}

  private:

    ///@name Internal State
    ///@{

    MultiBody* m_multibody{nullptr};          ///< Owner of this Connection
    Transformation m_transformationToBody2;   ///< Transform to second body
    Transformation m_transformationToDHFrame; ///< Transform to DH frame
    DHParameters m_dhParameters;              ///< DH frame description

    JointType m_jointType;                     ///< Type of connection
    std::pair<size_t, size_t> m_bodyIndices;   ///< (previous body, next body)
    std::array<Range<double>, 2> m_jointRange; ///< The valid joint ranges.

    ///@}

};


/*---------------------------------- Debug -----------------------------------*/

std::ostream& operator<<(std::ostream&, const Connection::JointType&);

std::ostream& operator<<(std::ostream& _os, const Connection& _c);

/*----------------------------------------------------------------------------*/

#endif
