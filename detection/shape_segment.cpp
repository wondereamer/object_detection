#include "shape_segment.h"
#include "graph.h" 
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>
#include <set>
#include <vector>
#include "m_util.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>
#include "triangulate.h"
using namespace boost;
const int LOWER_BOUND = 100;                     //!< the minmum number one component have 
int TrNode::numLeafs = 0;
std::set<TrNode> ClusterNode::edgeInfo;
std::map<int, int> ClusterNode::gId2treeId;
ClusterNode::HieraTree ClusterNode::hieracTree;
ClusterNode::HieraTree::NodeId ClusterNode::rootId;
ClusterNode::HieraTree::EdgeWeightsMap ClusterNode::weights = hieracTree.edge_weights();

void ClusterNode::reset(){
    TrNode::numLeafs = 0;
    edgeInfo.clear();
    gId2treeId.clear();
    hieracTree.clear();
}

bool ClusterNode::bestFittingCircle(double *pts, int numpts, double& x0, double &y0, double& r)
{
    SymMatrix3x3 AtA;
    int i;
    double w, w2, r2, xy2, Atb[3] = {0,0,0};
    Point p;

    for (i=0; i<numpts;)
    {
        p.x = pts[i++]; p.y = pts[i++]; p.z = pts[i++];
        w = p.z; w2 = w*w;
        AtA.M[0] += w2*p.x*p.x; AtA.M[1] += w2*p.x*p.y; AtA.M[2] += w2*p.y*p.y;
        AtA.M[3] += w2*p.x;     AtA.M[4] += w2*p.y;     AtA.M[5] += w2;
        xy2 = w2*(p.x*p.x+p.y*p.y);
        Atb[0] += p.x*xy2;
        Atb[1] += p.y*xy2;
        Atb[2] += xy2;
    }
    AtA.M[0] *= 4; AtA.M[1] *= 4; AtA.M[2] *= 4;
    AtA.M[3] *= 2; AtA.M[4] *= 2;
    Atb[0] *= 2;
    Atb[1] *= 2;

    if (!AtA.invert()) return 0;

    x0 = AtA.M[0]*Atb[0] + AtA.M[1]*Atb[1] + AtA.M[3]*Atb[2];
    y0 = AtA.M[1]*Atb[0] + AtA.M[2]*Atb[1] + AtA.M[4]*Atb[2];
    r2 = AtA.M[3]*Atb[0] + AtA.M[4]*Atb[1] + AtA.M[5]*Atb[2] + x0*x0 + y0*y0;

    if (r2 < 0) JMesh::error("bestFittingCircle: Unexpected negative r2 (%f)\n",r2);

    r = sqrt(r2);

    return 1;
}


bool ClusterNode::bestFittingSphere(SymMatrix4x4& AtA, double *Atb, double& x0, double &y0, double &z0, double& r)
{
    if (!AtA.invert()) return 0;

    x0 = (AtA.a2*Atb[0] + AtA.ab*Atb[1] + AtA.ac*Atb[2] + AtA.ad*Atb[3])/2.0;
    y0 = (AtA.ab*Atb[0] + AtA.b2*Atb[1] + AtA.bc*Atb[2] + AtA.bd*Atb[3])/2.0;
    z0 = (AtA.ac*Atb[0] + AtA.bc*Atb[1] + AtA.c2*Atb[2] + AtA.cd*Atb[3])/2.0;
    double r2 = AtA.ad*Atb[0] + AtA.bd*Atb[1] + AtA.cd*Atb[2] + AtA.d2*Atb[3] + x0*x0 + y0*y0 + z0*z0;

    if (r2 < 0) return 0;

    r = sqrt(r2);

    return 1;
}


//////////////////////////////////////////////////////////////////////////
//
// Implementation of the class ClusterNode
//
//////////////////////////////////////////////////////////////////////////

ClusterNode::ClusterNode(Triangle *t, int index)
{
    id = index;
    triangles.appendHead(t);
    double *a = new double(t->area());
    areas.appendHead(a);
    tot_area = *a;
    sum_ctr = t->getCenter();
    Vertex *v1=t->v1(), *v2=t->v2(), *v3=t->v3();
    Cov_v = (SymMatrix3x3(v1->x, v1->y, v1->z)+SymMatrix3x3(v2->x, v2->y, v2->z)+SymMatrix3x3(v3->x, v3->y, v3->z))*(tot_area/3.0);
    sum_ctr *= tot_area;
    Edge *e; double le; Point pe;
    for (e=t->e1; e!=NULL; e=(e==t->e1)?(t->e2):((e==t->e2)?(t->e3):(NULL)))
        if (!e->isOnBoundary())
        {
            le = e->length(); pe = e->toVector()/le;
            Cov_c += (SymMatrix3x3(pe.x, pe.y, pe.z)*(le*0.5*(M_PI-e->dihedralAngle())));
        }

    Vertex *v;
    Atb[0] = Atb[1] = Atb[2] = Atb[3] = 0;
    for (v=v1; v!=NULL; v=(v==v1)?(v2):((v==v2)?(v3):(NULL)))
    {
        le = (tot_area*tot_area)/9.0;
        AtA.a2 += le*v->x*v->x; AtA.ab += le*v->x*v->y; AtA.ac += le*v->x*v->z; AtA.ad += le*v->x;
        AtA.b2 += le*v->y*v->y; AtA.bc += le*v->y*v->z; AtA.bd += le*v->y;
        AtA.c2 += le*v->z*v->z; AtA.cd += le*v->z;
        AtA.d2 += le;

        le = le*(v->x*v->x+v->y*v->y+v->z*v->z);
        Atb[0] += v->x*le;
        Atb[1] += v->y*le;
        Atb[2] += v->z*le;
        Atb[3] += le;
    }
}


double ClusterNode::fittingPlaneCost(const void *cn1, const void *cn2)
{
    ClusterNode *n1 = (ClusterNode *)cn1;
    ClusterNode *n2 = (ClusterNode *)cn2;

    double lcost = 0;
    Node *n, *m;
    Triangle *t;
    ClusterNode *gn;
    double area, tot_area = (n1->tot_area+n2->tot_area);
    // average point position
    Point app = (n1->sum_ctr+n2->sum_ctr)/tot_area;
    // Cov Matrix
    SymMatrix3x3 TM = n1->Cov_v+n2->Cov_v;
    TM -= (SymMatrix3x3(app.x, app.y, app.z)*tot_area);
    // normal vector
    Point nor;
    TM.getMinEigenvector(&(nor.x), &(nor.y), &(nor.z));
    if (nor.isNull()) return DBL_MAX;

    double d, a = -(nor*app);
    for (gn=n1; gn!=NULL; gn=(gn==n1)?(n2):(NULL))
    {
        m=gn->areas.head(); 
        FOREACHVTTRIANGLE((&(gn->triangles)), t, n)
        {
            area = (*((double *)m->data));
            d = ((t->getCenter())*(nor))+a; 
            d *= d; 
            // ignore factors smaller than lower bound of double
            if(!m_util::is_nun(d*area))
                lcost += (d*area);
            m=m->next();
        }
    }

    return lcost;
}


double ClusterNode::fittingSphereCost(const void *cn1, const void *cn2)
{
    ClusterNode *n1 = (ClusterNode *)cn1;
    ClusterNode *n2 = (ClusterNode *)cn2;

    // radius
    double d, area, radius, tot_area = (n1->tot_area+n2->tot_area), lcost = 0;
    ClusterNode *gn;
    Triangle *t;
    Node *n, *o;
    Point p;
    // center point
    Point center;

    SymMatrix4x4 AtA = n1->AtA+n2->AtA;
    double Atb[4] = {0,0,0,0};
    Atb[0] = n1->Atb[0] + n2->Atb[0]; 
    Atb[1] = n1->Atb[1] + n2->Atb[1]; 
    Atb[2] = n1->Atb[2] + n2->Atb[2]; 
    Atb[3] = n1->Atb[3] + n2->Atb[3]; 
    if (!bestFittingSphere(AtA, Atb, center.x, center.y, center.z, radius)) return DBL_MAX;
    if (tot_area/radius < 1.0e-9) return DBL_MAX;

    for (gn=n1; gn!=NULL; gn=(gn==n1)?(n2):(NULL))
    {
        o = gn->areas.head(); FOREACHVTTRIANGLE((&(gn->triangles)), t, n)
        {
            area = (*((double *)o->data));
            // 
            d = t->getCenter().distance(center)-radius; 
            d *= d; 
            if(!m_util::is_nun(d*area))
                lcost += (d*area);
            o=o->next();
        }
    }

    return lcost;
}


double ClusterNode::fittingCylinderCost(const void *cn1, const void *cn2)
{
    ClusterNode *n1 = (ClusterNode *)cn1;
    ClusterNode *n2 = (ClusterNode *)cn2;

    double lcost = 0;
    double tot_area = (n1->tot_area+n2->tot_area);
    Point axis, center_of_mass;
    ClusterNode *gn;
    Triangle *t;
    Node *n, *o;
    Point p, b;
    double radius, area, d, x0, y0;

    double sevals[3], evals[3], evecs[9];
    SymMatrix3x3 Cov_c = n1->Cov_c+n2->Cov_c;

    Cov_c.diagonalize(sevals, evecs);
    evals[0] = FABS(sevals[0]);
    evals[1] = FABS(sevals[1]);
    evals[2] = FABS(sevals[2]);
    int index[3];
    index[0] = (evals[0]>evals[1] && evals[0]>evals[2])?(0):((evals[1]>evals[0] && evals[1]>evals[2])?(1):(2));
    index[2] = (evals[0]<=evals[1] && evals[0]<=evals[2])?(0):((evals[1]<=evals[0] && evals[1]<=evals[2])?(1):(2));
    index[1] = (index[0]!=0 && index[2]!=0)?(0):((index[0]!=1 && index[2]!=1)?(1):(2));

    if (evals[index[0]]/tot_area < 1.0e-9) 
        return DBL_MAX;

    axis.setValue(evecs[3*index[0]], evecs[3*index[0]+1], evecs[3*index[0]+2]);
    //
    center_of_mass = (n1->sum_ctr+n2->sum_ctr)/tot_area;

    Point center_of_cyl;
    Point ce1, ce2;
    ce1.setValue(evecs[3*index[1]], evecs[3*index[1]+1], evecs[3*index[1]+2]);
    ce2.setValue(evecs[3*index[2]], evecs[3*index[2]+1], evecs[3*index[2]+2]);

    double *pts = new double[(n1->triangles.numels()+n2->triangles.numels())*9];
    int i=0;
    for (gn=n1; gn!=NULL; gn=(gn==n1)?(n2):(NULL))
    {
        o = gn->areas.head(); FOREACHVTTRIANGLE((&(gn->triangles)), t, n)
        {
            area = (*((double *)o->data));
            b = (*(t->v1()))-center_of_mass; pts[i++] = b*ce1; pts[i++] = b*ce2; pts[i++] = area;
            b = (*(t->v2()))-center_of_mass; pts[i++] = b*ce1; pts[i++] = b*ce2; pts[i++] = area;
            b = (*(t->v3()))-center_of_mass; pts[i++] = b*ce1; pts[i++] = b*ce2; pts[i++] = area;
            o=o->next();
        }
    }
    if (!bestFittingCircle(pts, (n1->triangles.numels()+n2->triangles.numels())*9, x0, y0, radius)) 
    {delete pts; return DBL_MAX;}
    delete pts;

    if (tot_area/radius < 1.0e-9) return DBL_MAX;

    center_of_cyl = center_of_mass + ce1*x0 + ce2*y0;

    for (gn=n1; gn!=NULL; gn=(gn==n1)?(n2):(NULL))
    {
        o = gn->areas.head(); FOREACHVTTRIANGLE((&(gn->triangles)), t, n)
        {
            area = (*((double *)o->data));
            b = t->getCenter()-center_of_cyl;
            d = (b&axis).length()-radius; 
            d *= d; 
            if(!m_util::is_nun(d*area))
                lcost += (d*area);
            o=o->next();
        }
    }

    return lcost;
}


//////////////////////////////////////////////////////////////////////////////////////
//
// Performs a clustering of based on best-fitting planes, spheres and  cylinders.
// The return value is a list of clustering operations. Each operation consists of:
// (1) A list of triangles
// (2) Identifier of the cluster
// (3) Identifier of the sub-cluster that was lastly merged into this one
//
// The first three nodes of the list represent the final single cluster.
// The second triple of nodes represents the cluster that was merged into the final
// single one.
// And so on, up to the last triple of nodes representing the first pair of adjacent
// triangles that were merged into a cluster.
//
//////////////////////////////////////////////////////////////////////////////////////



/**
 * @brief merge n1 to n2
 *
 * @param n1
 * @param n2
 */
void ClusterNode::merge(const void *n1, const void *n2)
{
    //    std::cout<<edgeCostFunction(n1, n2)<<std::endl;
    ClusterNode *c1 = (ClusterNode *)n1;
    ClusterNode *c2 = (ClusterNode *)n2;

    // create leaf nodes
    HieraTree::NodeId child1Id;
    HieraTree::NodeId child2Id;
    int c2Size = c2->triangles.numels();
    int c1Size = c1->triangles.numels();
    if(c1Size >= LOWER_BOUND || c2Size >= LOWER_BOUND){
        auto i1 = gId2treeId.find(c1->id);
        auto i2 = gId2treeId.find(c2->id);
        if(i1 != gId2treeId.end()){
            // exist internal node
            child1Id = i1->second;
        }else{
            // insert a new leaf node
            // copy triangles data
            TrNode leaf;
            leaf.triangles = c1->triangles.toArray();
            leaf.size = c1->triangles.numels();
            leaf.id1 = TrNode::numLeafs++;
            // mark the node is a leaf
            leaf.id2 = -1;
            child1Id = hieracTree.add_node(leaf);
            leaf.triangles = NULL;
        }
        if(i2 != gId2treeId.end()){
            child2Id = i2->second;
        }else{
            TrNode leaf;
            leaf.triangles = c2->triangles.toArray();
            leaf.size = c2->triangles.numels();
            leaf.id1 = TrNode::numLeafs++;
            leaf.id2 = -1;
            child2Id = hieracTree.add_node(leaf);
            leaf.triangles = NULL;
        }
    }
    // merge c2 to c1, update c1 for further merging
    // store pointers of triangles, c2 is kept in the clusterGraph,
    // but masked to 1. 
    c1->triangles.joinTailList(&(c2->triangles));
    c1->areas.joinTailList(&(c2->areas));
    c1->sum_ctr += c2->sum_ctr;
    c1->tot_area += c2->tot_area;
    c1->Cov_v += c2->Cov_v;
    c1->Cov_c += c2->Cov_c;
    c1->AtA += c2->AtA;
    c1->Atb[0] += c2->Atb[0];
    c1->Atb[1] += c2->Atb[1];
    c1->Atb[2] += c2->Atb[2];
    c1->Atb[3] += c2->Atb[3];
    // create parent node 
    if(c1Size >= LOWER_BOUND || c2Size >= LOWER_BOUND){
        TrNode me1, me2;
        me1.id1 = c1->id;
        me1.id2 = c2->id;
        me2.id1 = c2->id;
        me2.id2 = c1->id;
        std::set<TrNode>::iterator i = edgeInfo.find(me1);
        if (i == edgeInfo.end()) {
            i = edgeInfo.find(me2);
        }
        // attributes of parent is set in function #edgeCostFunction
        // insert a new parent node
        HieraTree::NodeId parentId = hieracTree.add_node(*i);
        rootId = parentId;
        gId2treeId[c1->id] = parentId;
        // insert edges between child and parent
        auto edge1Id = hieracTree.add_edge(parentId, child1Id).first;
        auto edge2Id = hieracTree.add_edge(parentId, child2Id).first;
        if (c1Size > c2Size) {
            weights[edge1Id] = 1;
            weights[edge2Id] = 0;
        }else{
            weights[edge1Id] = 0;
            weights[edge2Id] = 1;
        }
    }

}

// The ACT_AREA_BIAS is a small coefficient that prevents the
// creation of too unbalanced trees on flat areas
#define ACT_AREA_BIAS 1.0e-12

double ClusterNode::edgeCostFunction(const void *cn1, const void *cn2)
{
    //@@!
    ClusterNode *n1 = (ClusterNode *)cn1;
    ClusterNode *n2 = (ClusterNode *)cn2;
    double tot_area = n1->tot_area + n2->tot_area;
    double c1, c2, c3;
    // find the best fitting
    c1 = fittingPlaneCost(cn1, cn2);
    c2 = fittingSphereCost(cn1, cn2);
    c3 = fittingCylinderCost(cn1, cn2);
    double cost = c1;
    if (c2 < cost) cost = c2;
    if (c3 < cost) cost = c3;
    double temp = cost;
    // adjust cost
    if (cost < DBL_MAX) cost += tot_area*ACT_AREA_BIAS;
    // record the fitting cost
    TrNode internalNode;
    internalNode.id1 = int(n1->id);
    internalNode.id2 = int(n2->id);
    // record the fitting primitive
    if(temp == c1)
        internalNode.type = HFP_FIT_PLANES;
    else if(temp == c2)
        internalNode.type = HFP_FIT_SPHERES;
    else if(temp == c3)
        internalNode.type = HFP_FIT_SPHERES;
    else{
        assert(false);
    }
    internalNode.cost = cost;
    internalNode.cost0 = c1;
    internalNode.cost1 = c2;
    internalNode.cost2 = c3;
    edgeInfo.insert(internalNode);
    return cost;
}

ClusterNode::HieraTree& ClusterNode::cluster(MyTriangulation *tin){
    // triangle meshes info
    // the destruction function of clusterGraph will free ClusterNode
    clusterGraph cg(tin->E.numels(), &ClusterNode::edgeCostFunction);
    Triangle *t;
    Edge *e;
    Node *n;
    int i=0;
    // insert nodes to graph
    FOREACHVTTRIANGLE((&(tin->T)), t, n)
        t->info = cg.addNode(new ClusterNode(t, i++));
    // insert edges to graph
    FOREACHVEEDGE((&(tin->E)), e, n){
        if (!e->isOnBoundary()) {
            cg.createEdge(((ClusterNode *)e->t1->info),((ClusterNode *)e->t2->info));
        }
    }
    // cluster the graph, collapse the shortest edge
    while (cg.collapseFirstEdge(&ClusterNode::merge)) { }

    // output info
    //    std::cout<<cg.nodes.numels()<<std::endl;
    //    GraphUtil<HieraTree> gutil(&hieracTree);
    //    gutil.write2dot("hierarchy_tree.dot");
    //    std::cout<<"*************nodes:**************"<<std::endl;    
    //    std::cout<<hieracTree.num_nodes()<<std::endl;
    //    auto nodeRange = hieracTree.get_all_nodes();
    //    for(; nodeRange.first != nodeRange.second; nodeRange.first++){
    //        int id = *nodeRange.first;
    //        const TrNode &node =  hieracTree.get_node(id);
    //        std::cout<<node.size<<std::endl;
    //    }
    std::cout<<"num triangle:"<<cg.nodes.numels()<<std::endl;
    std::cout<<"clustering done!" ;
    return hieracTree;
}
