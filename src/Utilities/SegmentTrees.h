#ifndef SEGMENT_TREES_H_
#define SEGMENT_TREES_H_

#include <memory>
#include <map>
#include <limits>
#include <CGAL/Cartesian.h>
#include <CGAL/Segment_tree_k.h>
#include <CGAL/Range_segment_tree_traits.h>
using namespace std;

#include "Vector.h"
using namespace mathtool;
#include "IOUtils.h"

#include "Geometry/Boundaries/Boundary.h"


/////////////////////////////////////////////////////////////////////////////
/// CGAL Segment Tree implementation for easy interval (bounding volme) 
/// search 
/////////////////////////////////////////////////////////////////////////////

template<typename PropertyType=size_t>
class SegmentTrees {
  public:
    ///@name Local types
    ///{
    typedef CGAL::Cartesian<double> CK;
    typedef CGAL::Segment_tree_map_traits_2<CK, size_t> Traits2;
    typedef CGAL::Segment_tree_2<Traits2> Segment_tree_2_type;
    typedef CGAL::Segment_tree_map_traits_3<CK, size_t> Traits3;
    typedef CGAL::Segment_tree_3<Traits3> Segment_tree_3_type;
    typedef Traits2::Interval Interval2;
    typedef Traits3::Interval Interval3;
    typedef Traits2::Pure_interval Pure_interval2;
    typedef Traits3::Pure_interval Pure_interval3;
    typedef Traits2::Key Key2;
    typedef Traits3::Key Key3;
    ///@}
    ///@name Construction
    ///@{

    SegmentTrees(size_t _d=3);
    ~SegmentTrees();

    ///@}
    ///@name Modifiers
    ///@{

    /// Add a boundary
    /// @param _b Boundaries.
    void AddBoundary(const Boundary* _b) { 
      m_boundaries.push_back(_b);	
    }
		
    /// Add a boundary with a property index
    /// @param _b Boundaries.
    /// @param _p Property index
    void AddBoundary(const Boundary* _b, PropertyType _p);

    /// Build the Segment Trees
    void BuildSegmentTrees();

    /// Finds the Number of Enclosing Boundaries
    /// @param _p Point to search
    /// @param _e Range  of the point 
    /// @return Number of enclosing boundaries
    size_t FindEnclosingBoundaries(const Point3d& _p, 
        double _e=0.5*numeric_limits<float>::epsilon());

    ///@}
    ///@name Accessors
    ///@{

    // Get the number of output enclosing ranges
    size_t GetNumberEnclosingBoundaries() {
      return m_output.size();
    }

    // Get the input key index of the output enclosing range
    // @param _i Index to the output enclosing range
    size_t GetOutputBoundaryIndex(size_t _i) {
      return m_output[_i];
    }

    // Get the property of the output enclosing range
    // @param _i Index to the output enclosing range
    PropertyType GetOutputBoundaryProperty(size_t _i) {
      return m_propertyMap[m_output[_i]];
    }

    // Get the output enclosing range
    // @param _i Index to the output enclosing range
    const Boundary* GetOutputBoundary(size_t _i) {
      return m_boundaries[m_output[_i]];
    }

    // Dimension of the segment tree
    size_t GetDimension() {  return m_dimension; }
    void SetDimension(size_t _d = 3) { m_dimension = _d; }

    ///@}

  private:

    ///@name Internal State
    ///@{

    vector<const Boundary*> m_boundaries; ///< Input set of bounding volumes
    vector<PropertyType> m_propertyMap;	///< Property map associated with the input volumes
    vector<size_t> m_output; ///< Output set of enclosing volumes
    size_t m_dimension{3}; ///< Dimension of the problem or environment
    bool m_debug{false};         ///< Toggle debug messages.
    Segment_tree_2_type* m_tree2{nullptr};
    Segment_tree_3_type* m_tree3{nullptr};

    ///@}
};

/*------------------------------- Construction ----------------------------*/
template<typename PropertyType>
SegmentTrees<PropertyType>::
SegmentTrees(size_t _d) : m_dimension(_d) {
}

template<typename PropertyType>
SegmentTrees<PropertyType>::
~SegmentTrees() {
  m_boundaries.clear();
  m_output.clear();
  delete m_tree2;
  delete m_tree3;
}


/*------------------------- Creation of Segment Trees ---------------------*/

template<typename PropertyType>
void SegmentTrees<PropertyType>::
AddBoundary(const Boundary* _b, PropertyType _p) { 
  AddBoundary(_b);	
  m_propertyMap.emplace_back(_p);
}


template<typename PropertyType>
void SegmentTrees<PropertyType>::BuildSegmentTrees() {
  delete m_tree2;
  delete m_tree3;
  // Store the intervals of each boundary with the input index as key index
  if(m_dimension == 2) {
    vector<Interval2> in;
    for(size_t i = 0; i < m_boundaries.size(); i++) {
      auto b = m_boundaries[i];
      auto x = b->GetRange(0);
      auto y = b->GetRange(1); 
      in.push_back(Interval2(Pure_interval2(Key2(x.min,y.min), 
              Key2(x.max,y.max)),i));
    }
    m_tree2 = new Segment_tree_2_type(in.begin(), in.end()); 
  }
  else {
    vector<Interval3> in;
    for(size_t i = 0; i < m_boundaries.size(); i++) {
      auto b = m_boundaries[i];
      auto x = b->GetRange(0);
      auto y = b->GetRange(1);
      auto z = b->GetRange(2);
      in.push_back(Interval3(Pure_interval3(Key3(x.min,y.min,z.min),
              Key3(x.max,y.max,z.max)),i));
    }
    m_tree3 = new Segment_tree_3_type(in.begin(), in.end());
  }
}

/*------------------------- Find Bounding Volumes -------------------------*/
template<typename PropertyType>
size_t 
SegmentTrees<PropertyType>::
FindEnclosingBoundaries(const Point3d& _p, double _e) {
  // Clear previously queried results
  m_output.clear();
  // Generate a query range with _p as midpoint, _e as offset and 
  // number of input boundaries as index
  // Use enclosing_query to find the enclosing range key indices and 
  // store them
  if(m_dimension == 2) {
    vector<Interval2> out;
    auto p = Interval2(Pure_interval2(Key2(_p[0]-_e,_p[1]-_e), 
          Key2(_p[0]+_e,_p[1]+_e)),m_boundaries.size());
    m_tree2->window_query(p,std::back_inserter(out));
    for(auto i = out.begin(); i != out.end(); i++)
      m_output.push_back(i->second);
  }
  else {
    vector<Interval3> out;
    auto p = Interval3(Pure_interval3(Key3(_p[0]-_e,_p[1]-_e,_p[2]-_e), 
          Key3(_p[0]+_e,_p[1]+_e,_p[2]+_e)),m_boundaries.size());
    m_tree3->window_query(p,std::back_inserter(out));
    for(auto i = out.begin(); i != out.end(); i++)
      m_output.push_back(i->second);
  }
	
  return GetNumberEnclosingBoundaries();
}


/*-------------------------------------------------------------------------*/

#endif
