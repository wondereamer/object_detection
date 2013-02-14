
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

// These are constants to be used within the mask 'what_to_fit'
#define HFP_FIT_PLANES    ((unsigned char)1)
#define HFP_FIT_SPHERES   ((unsigned char)2)
#define HFP_FIT_CYLINDERS ((unsigned char)4)

//////////////////////////////////////////////////////////////////////////
//
// Node of the cluster graph
//
//////////////////////////////////////////////////////////////////////////
class MyTriangulation;
class ClusterNode;
struct TrNode{
    TrNode(){ size = 0; triangles = NULL;}
    ~TrNode(){
        if(!triangles){
            free(triangles);
        }
    }
    int id1;
    int id2;
    double cost;
    double cost0;
    double cost1;
    double cost2;
    unsigned char type;
    static int numLeafs;
    // pointer to node of clusterGraph
    void **triangles;
    int size;
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
};
using namespace boost;
class ClusterNode : public graphNode
{

    public:
        typedef adjacency_list<vecS, vecS, directedS, no_property, property<edge_weight_t, int> > boostGraph;
        typedef m_graph::AutoUniGraph<boostGraph, TrNode> HieraTree;

        ClusterNode(Triangle *, int);	// Constructor
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
        static void reset();
        //! graph -> ClusterNode -> triangles
        static HieraTree &cluster(MyTriangulation *tin);

    public:
        int id;                //!< Unique identifier of the node
        List triangles;        //!< All the triangles within the cluster
        List areas;            //!< Triangle areas
        Point sum_ctr;         //!< Weighted sum of barycenters
        SymMatrix3x3 Cov_v;    //!< Covariance matrix of cluster vertices
        SymMatrix3x3 Cov_c;    //!< Covariance matrix of normal variation
        SymMatrix4x4 AtA;      //!< To find bf-sphere (matrix)
        double Atb[4];         //!< To find bf-sphere (known term)
        double tot_area;       //!< Total area of the cluster
        static HieraTree hieracTree;
        static HieraTree::NodeId rootId;
    private:
        static std::set<TrNode> edgeInfo;
        // Hierarchical tree
        static HieraTree::EdgeWeightsMap weights;
        // graph node id to Hierarchical tree id
        static std::map<int, int> gId2treeId;
};
#endif // FITTING_PRIMITIVES_H
