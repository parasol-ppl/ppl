#ifndef PMPL_ENVIRONMENT_H_
#define PMPL_ENVIRONMENT_H_

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Utilities/MPUtils.h"

#include "Transformation.h"
#include "glutils/color.h"

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

class CollisionDetectionMethod;
class MultiBody;

class Robot;
class WorkspaceDecomposition;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Workspace representation of terrain within the world.
////////////////////////////////////////////////////////////////////////////////
class Terrain {

  public:

    ///@name Local Types
    ///@{

    enum Axis {X, Y, Z};

		///@}

    ///@name Construction
    ///@{

    Terrain();

    Terrain(XMLNode& _node);

    Terrain(const Terrain& _terrain);
    ///@}


    ///@name Accessors
    ///@{

    /// Get the color for visualization
    const glutils::color& Color() const noexcept;

    /// Get the single enclosing boundary of the terrain
    Boundary* GetBoundary() const noexcept;

    /// Get the single enclosing boundaries of the terrain
		const std::vector<std::unique_ptr<Boundary>>& GetBoundaries() const noexcept;

    /// Find the perimeter of all the boundaries
    double GetPerimeter();

    /// Check if point is in terrain
    /// @param _p The point to check
    /// @return True if _p within m_boundaries
		bool InTerrain(const Point3d _p) const noexcept;


    /// Check if configuration is in terrain
    /// @param _cfg The configuration to check
    /// @return True if _cfg within m_boundaries
		bool InTerrain(const Cfg _cfg) const noexcept;

    /// Check if terrain is a neighbor of this terrain
    /// @param _terrain The terrain to check for neighboring
    /// @return True if _terrain is touching this terrain
    bool IsNeighbor(const Terrain& _terrain);

    /// Check if terrain considered virtual
    /// @return Value of m_virtual
		bool IsVirtual() const noexcept;

    /// Check if mesh will be wired
    /// @return Value of m_wire
    bool IsWired() const noexcept;

    ///@}

  private:
		///@name Helpers
		///@{

    /// Determine if two boundaries are touching
    /// @param _bound1 First boundary to test
    /// @param _bound2 Second boundary to test
    /// @param _type Axis (X or Y) that boundaries determined to be touching on
    /// @return True if _bound1 and _bound2 touching in X or Y axis
    bool IsTouching(Boundary* _bound1, Boundary* _bound2, Axis& _type);

    /// Determine how much boundaries overlap
    /// @param _bound1 First boundary to test
    /// @param _bound2 Second boundary to test
    /// @return Amount of overlap, 0 if not touching
    double Overlap(Boundary* _b1, Boundary* _b2);

		///@}
		///@name Internal State
		///@{

    glutils::color m_color{glutils::color::green}; ///< Color for visualization.
    std::unique_ptr<Boundary> m_boundary;          ///< The enclosing boundary.

		std::vector<std::unique_ptr<Boundary>> m_boundaries; ///< represent the space occupied by the terrain

		bool m_virtual{false}; ///< indicates a boundary limit for robots of the same capability
													 ///< but does not invalidate robots of other capabiilties inside it.

    /// A rendering property, if true then the boundary is
    /// rendered as a solid mesh; otherwise, it is rendered in wire frame.
    bool m_wire{true};

		///@}
};


////////////////////////////////////////////////////////////////////////////////
/// Workspace for the motion planning problem, including a boundary and zero or
/// more obstacles.
////////////////////////////////////////////////////////////////////////////////
class Environment {

  public:

    ///@name Local Types
    ///@{

    typedef std::unordered_map<std::string, std::vector<Terrain>>
        TerrainMap;

    ///@}
    ///@name Construction
    ///@{

    Environment();

    explicit Environment(XMLNode& _node);

    Environment(const Environment& _other); ///< Copy.
    Environment(Environment&& _other);      ///< Move.

    virtual ~Environment();

    ///@}
    ///@name Assignment
    ///@{

    Environment& operator=(const Environment& _other); ///< Copy.
    Environment& operator=(Environment&& _other);      ///< Move.

    ///@}
    ///@name I/O
    ///@{

    /// Get the environment file name.
    const std::string& GetEnvFileName() const noexcept;

    /// Parse XML options from the Problem XML node.
    void ReadXMLOptions(XMLNode& _node);

    /// Parse XML environment file.
    void ReadXML(XMLNode& _node);

    /// Parse an old-style environment file.
    /// @param _filename The name of the file to read.
    /// @note This is provided only for supporting old environments. Do not make
    ///       any modifications here - if you need to add features, you must
    ///       upgrade your env file to the newer XML format.
    void Read(std::string _filename);

    /// Print environment resolutions and boundary information.
    /// @param _os The output stream to print to.
    void Print(std::ostream& _os) const;

    /// Output environment to .env file format.
    /// @param _os The output stream to write to.
    void Write(std::ostream& _os);

    ///@}
    ///@name Resolutions
    ///@{

    /// Automatically compute position resolution when not specified.
    /// @param _robots Robots to consider when determining resolution
    void ComputeResolution(const std::vector<std::unique_ptr<Robot>>& _robots);

    /// Get the position resolution
    double GetPositionRes() const noexcept;
    /// Set the position resolution
    void SetPositionRes(double _res) noexcept;

    /// Get the orientation resolution
    double GetOrientationRes() const noexcept;
    /// Set the orientation resolution
    void SetOrientationRes(double _res) noexcept;

    /// Get the time resolution
    double GetTimeRes() const noexcept;

    ///@}
    ///@name Boundary Functions
    ///@{

    /// Get the single boundary of the environemnt
    Boundary* GetBoundary() const noexcept;

    /// Get the single boundary of the environemnt
    /// @param _b The new boundary
    void SetBoundary(std::unique_ptr<Boundary>&& _b) noexcept;

    ///@}
    ///@name Obstacle Functions
    ///@{

    /// Get the number of MultiBodies
    size_t NumObstacles() const noexcept;

    /// Get requested obstacle
    /// @param _index Requested multibody
    /// @return Pointer to static multibody
    MultiBody* GetObstacle(size_t _index) const;

    /// Get random multiboy
    /// @return Pointer to random static multibody
    MultiBody* GetRandomObstacle() const;

    /// Add an obstacle to the environment.
    /// @param _dir Directory for geometry file
    /// @param _filename Geometry filename
    /// @param _t Transformation of object
    /// @return The index of the newly created obstacle.
    size_t AddObstacle(const std::string& _dir, const std::string& _filename,
        const mathtool::Transformation& _t = mathtool::Transformation());

    /// Remove an obstacle from the environment.
    /// @param _position Index in m_obstacleBodies to be removed
    void RemoveObstacle(const size_t _position);

    /// Remove obstacle from environment
    /// @param _obst Obstacle to be removed
    void RemoveObstacle(MultiBody* const _obst);

    /// Compute a mapping of the obstacle vertices.
    /// @return A map from obstacle points to obstacle indexes.
    std::map<mathtool::Vector3d, std::vector<size_t>> ComputeObstacleVertexMap()
        const;

    /// Check if the boundary is also modeled as an obstacle.
    bool UsingBoundaryObstacle() const noexcept;

    ///@}
    ///@name Physical Properties
    ///@{

    /// Get the friction coefficient
    double GetFrictionCoefficient() const noexcept;

    /// Get the gravity 3-vector
    const mathtool::Vector3d& GetGravity() const noexcept;

    ///@}
    ///@name Terrain Functions
    ///@{

    /// Get environment terrains
    const TerrainMap& GetTerrains() const noexcept;

    ///@}
    ///@name Simulation Functions
    ///@{

    /// Get the initial transformation for the camera.
    const mathtool::Transformation& GetInitialCameraTransformation() const
        noexcept;

    ///IROS Hacks

    /// Determine if two configurations are in the same boundary of a terrain
    /// and, if so, sets mutual terrain as new boundary
    /// @param _start First configuration
    /// @param _end Second configuration
    /// @return True if _start and _cfg in same terrain boundary,
    bool IsolateTerrain(Cfg start, Cfg goal);

    /// Determine if two configurations are in the same terrain
    /// @param _start First configuration
    /// @param _end Second configuration
    /// @return True if _start and _cfg in same terrain
		bool SameTerrain(Cfg _start, Cfg _goal);

    /// Restores original boundary
    void RestoreBoundary();

    /// Saves boundary as original boundary
    void SaveBoundary();

    std::unique_ptr<Boundary> m_originalBoundary;
    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Initialize a boundary object of the appropriate type.
    /// @param _type Type of boundary object (box, box2d, sphere, or sphere2d)
    /// @param _where String for parse exception
    void InitializeBoundary(std::string _type, const std::string _where);

    /// Create an obstacle for the boundary.
    void CreateBoundaryObstacle();

    ///@}
    ///@name File Info
    ///@{

    std::string m_filename;     ///< Which file did this environment come from?
    std::string m_modelDataDir; ///< Directory where environment file is located.

    ///@}
    ///@name Resolution Info
    ///@{

    double m_positionRes{-1.};       ///< Positional resolution of movement.
    double m_orientationRes{.05};    ///< Rotational resolution of movement.
    double m_timeRes{.05};           ///< Resolution for time.

    ///@}
    ///@name Models
    ///@{

    std::unique_ptr<Boundary> m_boundary;               ///< Workspace boundary.
    std::vector<std::unique_ptr<MultiBody>> m_obstacles;///< Obstacle multibodies.
    bool m_boundaryObstacle{false}; ///< Use the boundary as an obstacle?

    ///@}
    ///@name Physical Properties
    ///@{

    double m_frictionCoefficient{0}; ///< The uniform friction coefficient.
    mathtool::Vector3d m_gravity;    ///< The gravity direction and magnitude.

    ///@}
    ///@name Terrains
    ///@{

    TerrainMap m_terrains; ///< Environment terrains.

    ///@}
    ///@name Simulation Properties
    ///@{

    mathtool::Transformation m_initialCameraTransform; ///< Camera starts here.

    ///@}

};

#endif
