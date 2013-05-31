
#ifndef FITTING_PRIMITIVES_H
#define FITTING_PRIMITIVES_H

#include <boost/graph/adjacency_list.hpp>
#include "jmesh.h"
#include "clusterGraph.h"
#include <stdlib.h>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/config.hpp>
#include "graph.h"
#include <stack>
#include <pcl/common/common_headers.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include <library/m_geometry.h>

// These are constants to be used within the mask 'what_to_fit'
#define HFP_FIT_PLANES    ((unsigned char)1)
#define HFP_FIT_SPHERES   ((unsigned char)2)
#define HFP_FIT_CYLINDERS ((unsigned char)4)

//////////////////////////////////////////////////////////////////////////
//
// Node of the cluster graph
//
//////////////////////////////////////////////////////////////////////////
class Eye3D;
class ClusterNode;
struct Coefficient{
    Coefficient(){ }
    Coefficient(Point p, Point d, double r):point(p), direction(d), radius(r){ }

    Point point;
    Point direction;                            //!< normal or axis 
    double radius;
};

//typedef Eigen::VectorXf Coefficient;
struct TrNode{
    TrNode(){ 
        size = 0; 
        triangles = NULL; 
        parentId = -1;
        degree = 0;
        dist = 0;
        angle = 0;
        proportion = 0;
    }
    TrNode(int id1_, int id2_):id1(id1_), id2(id2_),
                               size(0), triangles(NULL),
                               parentId(-1), degree(0),
                               dist(0), angle(0), proportion(0) {
    }
    ~TrNode(){
        if(!triangles){
            free(triangles);
        }
    }
    bool operator<(const TrNode& r) const{
        if(id1 < r.id1)
            return true;
        else if(id1 > r.id1)
            return false;
        else if(id2 < r.id2)
            return true;
        else 
            return false;
    };
    bool is_leaf() const{
        return id2 == -1;
    }
    void set_leaf(bool v = true){
        if(v)
            id2 = -1;
        else
            id2 = 1;
    }
    public:
    // node id of cluster graph
    int id1;
    int id2;
    double cost;
    Coefficient coefficient;
    unsigned char type;
    static int numLeafs;
    static Coefficient defaultCoef;
    //! pointer to node of clusterGraph
    void **triangles;
    //! when node is a leaf, #size is useless
    int size;
    //! used in RefineSegManual and RefineSegAuto
    int parentId;
    int friendId;
    //! the center of the component
    m_geometry::PointF3D center;
    int degree;
    float dist;
    float angle;
    double proportion;
    //!
    double weight;
    double tpWeight;
};
bool operator == (const TrNode& a, const TrNode &b);
std::size_t hash_value(TrNode const &b);
using namespace boost;

typedef adjacency_list<setS, vecS, bidirectionalS, no_property, property<edge_weight_t, int> > boostGraph;
typedef m_graph::AutoUniGraph<boostGraph, TrNode> HieraTree;
//! BinaryTree is actually a graph type plus binary tree interfaces
class BinaryTree: public HieraTree
{
    public:
        typedef HieraTree::NodeId NodeId;
        //! return parent id, -1 if no parent
        NodeId parentId(NodeId id){
            auto inEdges = get_in_edges(id);
            if(inEdges.first != inEdges.second){
                return sourceId(*inEdges.first);
            }else
                return -1;
        }
        //! return id of two children, pair(-1, -1) if no children
        std::pair<NodeId, NodeId> childrenId(NodeId p){
            auto outEdges = get_out_edges(p);
            std::pair<NodeId, NodeId> children;
            children.first = -1;
            children.second = -1;
            if (outEdges.first != outEdges.second) 
                children.first = targetId(*outEdges.first);
            if (outEdges.first != outEdges.second && 
                    ++outEdges.first != outEdges.second) 
                children.second = targetId(*outEdges.first);
            return children;
        }
        //! return id of brother node
        NodeId brotherId(NodeId id){
            auto p = parentId(id);
            if (p != -1) {
                auto children = childrenId(p);
                assert(children.first != -1 && children.second != -1);
                return children.first == id? children.second : children.first;
            }else 
                return -1;
        }
        //! whether #t is the brother of one element in _outStack
        bool is_brother_of_out(NodeId t){
            NodeId brother = brotherId(t);
            if(brother ==  -1)
                return false;
            auto temp = _outStack;
            while(!temp.empty()){
                auto id = temp.top();
                if(id == brother)
                    return true;
                temp.pop();
            }
            return false;
        }
        //! return rootId of new subtree #tr
        NodeId sub_tree(BinaryTree *tr, NodeId root) {
            // copy current node
            auto &node = this->get_node(root);
            auto id = tr->add_node(node);
            // connect current node and children
            auto children = this->childrenId(root);
            if (children.first != -1) {
                assert(children.second != -1);
                // copy the edge
                tr->add_edge(id, sub_tree(tr, children.first));
                tr->add_edge(id, sub_tree(tr, children.second));
            }
            return id;
        }
    public:
        std::stack<NodeId> _outStack;           // could replace stack with vector 
        std::stack<NodeId> _inStack;
        static TrNode _currentNode;
        NodeId rootId;

};

typedef pcl::PointXYZRGB PointT;
class ClusterNode : public graphNode
{

    public:
        ClusterNode(Triangle *, int, int);	// Constructor
        ClusterNode(){ }
        virtual ~ClusterNode() {
            areas.freeNodes();
            // clear the list itself
            triangles.removeNodes();
        }
        static void merge(const void *n1, const void *n2);
        static double edgeCostFunction(const void *n1, const void *n2);
        static bool bestFittingCircle(double *, int, double&, double&, double&);
        static bool bestFittingSphere(SymMatrix4x4&, double *, double&, double&, double&, double&);
        static double fittingPlaneCost(const void *, const void *);
        static double fittingSphereCost(const void *, const void *);
        static double fittingCylinderCost(const void *, const void *);

        static double my_fittingPlaneCost(const void *, const void *);
        static double my_fittingSphereCost(const void *, const void *);
        static double my_fittingCylinderCost(const void *, const void *);
        //! should be invoke before segment every object
        static void reset();
        //! graph -> ClusterNode -> triangles
        static BinaryTree &cluster(Eye3D *tin);
    public:
        int id;                //!< Unique identifier of the node
        int childId;
        List triangles;        //!< All the triangles within the cluster
        List areas;            //!< Triangle areas
        Point sum_ctr;         //!< Weighted sum of barycenters
        SymMatrix3x3 Cov_v;    //!< Covariance matrix of cluster vertices
        SymMatrix3x3 Cov_c;    //!< Covariance matrix of normal variation
        SymMatrix4x4 AtA;      //!< To find bf-sphere (matrix)
        double Atb[4];         //!< To find bf-sphere (known term)
        double tot_area;       //!< Total area of the cluster
        std::list<int> _centerIds;            //!< id of center points 
        // internalNode of the tree have @type attribute setted,
        // but no triangles information attached different from leaf node 
        static BinaryTree hierarchyTree;
        // pointers to some element in #edgeInfo
        static std::stack<BinaryTree::NodeId> hierarcStack;
        //        static std::map<int, std::vector<double>> size2costs;
        // members in the instance have different size
        static Coefficient coef1;
        static Coefficient coef2;
        static Coefficient coef3;
        static int num_triangles;
    private:
        // grow when calculating the cost of merging
        static std::unordered_set<TrNode,boost::hash<TrNode>> edgeInfo;
        // Hierarchical tree
        static BinaryTree::EdgeWeightsMap weights;
        //! graph node id to Hierarchical tree id
        static std::unordered_map<int, int> gId2treeId;
        //! graph node id to type
        static pcl::PointCloud<PointT>::Ptr _cloud;
};
#endif // FITTING_PRIMITIVES_H
