#ifndef PMPL_DRAWABLE_ROADMAP_H_
#define PMPL_DRAWABLE_ROADMAP_H_

#include "DrawableCfg.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "ConfigurationSpace/Weight.h"
#include "Utilities/Hash.h"

#include "glutils/drawable.h"

#include <chrono>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

class DrawableMultiBody;


////////////////////////////////////////////////////////////////////////////////
/// Renderable representation of a Roadmap. Allows for visualization of vertex
/// and edge insertion.
////////////////////////////////////////////////////////////////////////////////
class DrawableRoadmap : public glutils::drawable  {

  public:

    ///@name Local Types
    ///@{

    typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>>   RoadmapType;
    typedef typename RoadmapType::VI                VI;
    typedef typename RoadmapType::EI                EI;
    typedef typename RoadmapType::VID               VID;
    typedef typename std::pair<VID, VID>            EdgeID;

    typedef typename std::vector<double>            CfgData;
    typedef typename std::vector<glutils::vector3f> PointPath;

    ///@}
    ///@name Constructor
    ///@{

    /// Installs hooks onto _graph and populates existing
    /// vertices and edges.
    /// @param _graph The graph this DrawableRoadmap represents.
    /// @param _color The color to draw the cfgs and edges.
    /// @param _name A name refering to this DrawableRoadmap.
    DrawableRoadmap(RoadmapType* _graph, const glutils::color& _color,
        const std::string& _name = "");

    ~DrawableRoadmap();

    ///@}

  protected:

    ///@name Modifiers
    ///@{
    /// These functions should be installed as roadmap hooks to update the
    /// rendering whenever the map changes.

    /// Add a vertex to the rendering.
    /// @param _vi Vertex iterator to the vertex just added.
    void AddVertex(VI _vi);

    /// Add an edge to the rendering.
    /// @param _ei edge iterator to the edge just added.
    void AddEdge(EI _ei);

    /// Remove a vertex from the rendering.
    /// @param _vi Vertex iterator to the vertex just added.
    void DeleteVertex(VI _vi);

    /// Remove an edge from the rendering.
    /// @param _ei edge iterator to the edge just added.
    void DeleteEdge(EI _ei);

    ///@}
    ///@name Rendering Options
    ///@{
    /// These functions should be installed as key press hooks to change the
    /// way the maps are rendered.
    /// @todo Apply these only to the selected roadmap.

    /// Switch between drawing the robots and drawing points for Cfgs.
    void ToggleRobot();

    ///@}
    ///@name Drawable Overloads
    ///@{

    virtual void initialize() override;

    virtual void draw() override;

    virtual void draw_select() override;

    virtual void draw_selected() override;

    virtual void draw_highlighted() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::unordered_map<VID, DrawableCfg> m_cfgs;    ///< Cfg renderings.
    std::unordered_map<EdgeID, PointPath> m_edges;  ///< Edge renderings.

    /// Buffer for configurations.
    std::unordered_map<VID, DrawableCfg> m_bufferAddCfgs;
    std::unordered_set<VID> m_bufferDeleteCfgs;

    /// Buffer for edges.
    std::unordered_map<EdgeID, PointPath> m_bufferAddEdges;
    std::unordered_set<EdgeID> m_bufferDeleteEdges;

    glutils::color m_color;                   ///< The rendering color.
    MultiBody m_multiBody;                    ///< Local copy of the multibody.
    std::unique_ptr<DrawableMultiBody> m_dmb; ///< The drawable multibody.
    std::atomic<bool> m_drawRobot{false};     ///< Draw the robots or a point?
    mutable std::mutex m_lock;                ///< Lock for updating the rendering.
    RoadmapType* m_graph;                     ///< Pointer to the RoadmapGraph.

    std::string m_name;                       ///< A name for this rendering.

    bool m_initialized{false};                ///<Flag indiciating if the initialization function has been called.

    ///@}

};

#endif
