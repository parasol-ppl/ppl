#ifndef PMPL_BODY_H_
#define PMPL_BODY_H_

#include "Geometry/GMSPolyhedron.h"
#include "Utilities/MPUtils.h"

#include "Transformation.h"

#include "glutils/color.h"

#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace mathtool;

class CollisionDetectionMethod;
class Connection;
class MultiBody;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// A single polyhedral body in workspace. One or more of these are composed to
/// form a MultiBody, which is PMPL's working representation of multi-part object
/// geometries.
///
/// @details Each Body has two representations: one for the 'model frame' and one
///          for the 'world frame'. The model frame is the coordinate frame in
///          which the Body was originally described - this is usually centered
///          on the Body's center of mass. The world frame is the coordinate
///          frame in our current environment - this represents the Body after
///          it has been translated/oriented into its place in the planning
///          scene.
/// @ingroup Geometry
///
/// @TODO Remove COM adjust everywhere. This has become more problematic than
///       helpful. We will always do BBX centering from the fix onward.
////////////////////////////////////////////////////////////////////////////////
class Body {

  public:

    ///@name Local Types
    ///@{

    /// The type of part this body represents within its owning multi body.
    enum class Type {
      Planar,     ///< 2D base
      Volumetric, ///< 3D base
      Fixed,      ///< Fixed base
      Joint       ///< Link
    };

    /// The type of movement this body can perform.
    enum class MovementType {
      Fixed,        ///< No movement
      Joint,        ///< Defined by joint connection
      Rotational,   ///< Rotation + translation
      Translational ///< Just translation
    };

    ///@}
    ///@name Construction
    ///@{

    /// Construct an empty body.
    /// @param _owner The owning multibody.
    /// @param _index The body index in the multibody (0 for base).
    Body(MultiBody* const _owner, const size_t _index = 0);

    /// Construct a body from an XML node.
    /// @param _owner The owning multibody.
    /// @param _node The XML node to parse.
    Body(MultiBody* const _owner, XMLNode& _node);

    /// Copying a body does not copy the owning multibody pointer or the
    /// connections as this would not constitute a meaningful object. Moving a
    /// body carries the external pointers as expected.
    Body(const Body& _other); ///< Copy.
    Body(Body&& _other);      ///< Move.

    ~Body();

    ///@}
    ///@name Assignment
    ///@{

    Body& operator=(const Body& _other);  ///< Copy.
    Body& operator=(Body&& _other);       ///< Move.

    ///@}
    ///@name Validation
    ///@{

    /// @TODO Move to GMSPolyhedron.
    /// Determine if the polyhedron is valid, triangulated, closed, and having
    /// all normals facing a consistent 'outward' direction. Throws an exception
    /// if the model is invalid (because invalid models do not behave properly
    /// with collision checks).
    /// @note Uses a combination of the regular and CGAL points as we use CGAL
    ///       for some of these checks.
    void Validate() const;

    ///@}
    ///@name MultiBody Accessors
    ///@{

    /// Get the owning MultiBody.
    MultiBody* GetMultiBody() const noexcept;

    /// Set the owning MultiBody.
    void SetMultiBody(MultiBody* const _owner) noexcept;

    /// Get the index of this body within the owning MultiBody.
    size_t GetIndex() const noexcept;

    ///@}
    ///@name Body Properties
    ///@}

    /// Get the unique string label for this body. If none is provided, the
    /// multibody index will be used.
    const std::string& Label() const noexcept;

    /// Is this body a base?
    bool IsBase() const noexcept;

    /// Set the type of this body.
    void SetBodyType(const Body::Type _type) noexcept;

    /// Get the type of this body.
    Body::Type GetBodyType() const noexcept;

    /// Set the movement type for this body.
    void SetMovementType(const MovementType _type) noexcept;

    /// Get the movement type for this body.
    MovementType GetMovementType() const noexcept;

    ///@}
    ///@name Physical Properties
    ///@{

    /// Get the mass.
    double GetMass() const;

    /// Get the moment of inertia matrix in the model frame.
    const Matrix3x3& GetMoment() const;

    ///@}
    ///@name Geometric Properties
    ///@{

    /// Set the polyhedron model for this body.
    void SetPolyhedron(GMSPolyhedron&& _poly);

    /// Get the polyhedron in model coordinates.
    const GMSPolyhedron& GetPolyhedron() const;

    /// Get the polyhedron in world coordinates.
    const GMSPolyhedron& GetWorldPolyhedron() const;

    /// Get the bounding box in model coordinates.
    const GMSPolyhedron& GetBoundingBox() const;

    /// Compute the bounding box in world coordinates.
    GMSPolyhedron GetWorldBoundingBox() const;

    ///@}
    ///@name Transform Functions
    ///@{

    /// Mark all cached objects as requiring an update.
    void MarkDirty();

    /// Set the transformation from model to world coordinates.
    /// @param _transformation The new transformation for this body.
    void Configure(const Transformation& _transformation);

    /// Get the transformation from model to world coordinates.
    /// @return The current transformation.
    const Transformation& GetWorldTransformation() const;

    ///@}
    ///@name Connection Information
    ///@{

    /// @return Number of forward Connection
    size_t ForwardConnectionCount() const noexcept;

    /// @return Number of backward Connection
    size_t BackwardConnectionCount() const noexcept;

    /// @return Number of additional adjacent Connections
    size_t AdjacencyConnectionCount() const noexcept;

    /// @param _index Index of desired forward Connection
    /// @return Requested forward Connection
    Connection& GetForwardConnection(const size_t _index) const noexcept;

    /// @param _index Index of desired backward Connection
    /// @return Requested backward Connection
    Connection& GetBackwardConnection(const size_t _index) const noexcept;

    /// @param _index Index of desired adjacency Connection
    /// @return Requested adjacency Connection
    Connection& GetAdjacencyConnection(const size_t _index) const noexcept;

    /// Get the connection joining this to another body.
    /// @param _other The other body.
    /// @return The connection joining this and _other, or nullptr if the bodies
    ///         are not connected.
    Connection* GetConnectionTo(const Body* const _other) const noexcept;

    /// Determines if two bodies share the same joint
    /// @param _otherBody Second body
    /// @return True if adjacent
    bool IsAdjacent(const Body* const _otherBody) const;

    /// Determines if two bodies share a parent
    /// @param _otherBody Second body
    /// @return True if sharing a parent
    bool SameParent(const Body* const _otherBody) const;

    /// Attach a forward connection to this body.
    /// @param _c The Connection to attach.
    void LinkForward(Connection* const _c);

    /// Attach a backward connection to this body.
    /// @param _c The Connection to attach.
    void LinkBackward(Connection* const _c);

    /// Attach an Adjacency connection to this body.
    /// @param _c The Connection to attach.
    void LinkAdjacency(Connection* const _c);

    /// Remove a connection to this body. Does not affect the other body or the
    /// connection object.
    void Unlink(Connection* const _c);

    ///@}
    ///@name I/O
    ///@{
    /// @TODO Remove all COM adjust stuff.

    static std::string m_modelDataDir; ///< Directory of geometry files

    /// Get the file name from which this body was constructed.
    const std::string& GetFileName() const;

    /// Get the full path of the file from which this body was constructed.
    std::string GetFilePath() const;

    /// Read geometry information from file.
    /// @param _filename The file to read from.
    void ReadGeometryFile(const std::string& _filename);

    /// Read geometry information from file.
    /// @param _comAdjust Center of mass adjustment method
    void ReadGeometryFile(
        GMSPolyhedron::COMAdjust _comAdjust = GMSPolyhedron::COMAdjust::None);

    /// Parse a body from an old .env or .robot file.
    /// @param _is An open input stream for the geometry file.
    /// @param _cbs A counting stream buffer for error reporting.
    /// @note This function is provided for backwards compatibility with old
    ///       environments. If it does not work with your example, you should
    ///       update the old files to the new XML format. Do not waste time
    ///       working on this function.
    void Read(std::istream& _is, CountingStreamBuffer& _cbs);

    ///@}
    ///@name Visualization
    ///@{

    /// Get the color for the body.
    const glutils::color& GetColor() const;

    /// Set the color for the body.
    void SetColor(const glutils::color& _c);

    /// Check if a texture was loaded.
    bool IsTextureLoaded() const;

    /// Get the loaded texture file name.
    const std::string& GetTexture() const;

    ///@}

  private:

    ///@name Computation Helpers
    ///@{
    /// @note These methods are marked const even though they might change the
    ///       object state. This is because they essentially complete lazy
    ///       computations which begin when we configure the Body and lazily
    ///       resolve when we request the updated information.

    /// Approximate moment of inertia in model coordinates.
    void ComputeMomentOfInertia() const;

    /// Compute the axis-aligned bounding box in model coordinates.
    void ComputeBoundingBox() const;

    const Transformation& FetchBaseTransform() const noexcept;

    const Transformation& FetchLinkTransform() const noexcept;

    /// Compute the transformation of this body wrt its model frame. Applying
    /// this transformation to the body positions it in the world frame.
    /// @param _visited Stores which bodies have been visited
    /// @return The transformation from model to world coordinates.
    const Transformation& ComputeWorldTransformation(std::set<size_t>& _visited)
        const;

    ///@}
    ///@name Internal State
    ///@{

    MultiBody* m_multibody{nullptr};         ///< The owning MultBody.
    size_t m_index{0};                       ///< Index in owning MultiBody
    std::string m_label;                     ///< The unique part label.
    std::string m_filename;                  ///< Geometry filename.

    Body::Type m_bodyType;                   ///< Body type
    MovementType m_movementType;             ///< Movement type

    GMSPolyhedron m_polyhedron;              ///< Model in model coordinates.
    GMSPolyhedron m_boundingBox;             ///< AABB in model coordinates.

    Transformation m_transform;         ///< Transform from model to world frame.
    mutable bool m_transformCached{false};    ///< Is the transform cached?

    mutable GMSPolyhedron m_worldPolyhedron;     ///< Model in world coordinates.
    mutable bool m_worldPolyhedronCached{false}; ///< Is world polyhedron cached?

    double m_mass{1};                        ///< Mass of Body
    Matrix3x3 m_moment;                      ///< Moment of Inertia
    GMSPolyhedron::COMAdjust m_comAdjust{GMSPolyhedron::COMAdjust::COM};

    std::vector<Connection*> m_forwardConnections;  ///< Forward Connections
    std::vector<Connection*> m_backwardConnections; ///< Backward Connections
    std::vector<Connection*> m_adjacencyConnections;///< Adjacency Connections

    const Transformation& (Body::*m_transformFetcher)(void) const noexcept = nullptr;

    ///@}
    ///@name Display Stuff
    ///@{
    /// @TODO Rethink these parts as we move to the new simulator. Perhaps they
    ///       will move to the visualization department. In any case we should
    ///       separate the visualization details from the geometric model stored
    ///       here.

    glutils::color m_color{glutils::color::blue}; ///< Body color.
    std::string m_textureFile;                    ///< Optional texture file.

    ///@}

};

/*-------------------------------- I/O Helpers -------------------------------*/

std::ostream& operator<<(std::ostream& _os, const Body& _body);

#endif
