#include "cluster_node.h"
#include "graph.h" 
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>
#include <set>
#include <vector>
#include <library/m_util.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>
#include "eye3d.h"
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>

bool operator == (const TrNode& a, const TrNode &b) {
    return a.id1 == b.id1 && a.id2 == b.id2;
}
// #snippet
std::size_t hash_value(TrNode const &b)
{
    std::size_t seed =  0;
    boost::hash_combine(seed, b.id1);
    boost::hash_combine(seed, b.id2);
    return seed;
}


//    std::vector<int> inliers;
//    std::vector<double> distances;
//    pcl::SampleConsensusModelSphere<PointT>::Ptr
//        model_s(new pcl::SampleConsensusModelSphere<PointT> (cloud));
//    pcl::RandomSampleConsensus<PointT> ransac (model_s);
//    ransac.setDistanceThreshold (.01);
//    ransac.computeModel();
//    ransac.getInliers(inliers);
//    auto &p = cloud->points;
//    for(int i : inliers){
//        viz.add_point((float)p[i].x, (float)p[i].y, (float)p[i].z, 255, 255, 255, 1); 
//    }
//    Eigen::VectorXf modelCoefficient;
//    ransac.getModelCoefficients(modelCoefficient);
//    model_s->getDistancesToModel(modelCoefficient, distances);
//    std::copy(distances.begin(), distances.end(),    
//            std::ostream_iterator<double>(std::cout, "\n"));
//
//    viz.push_def_cloud();
//    viz.add_sphere(modelCoefficient[0], modelCoefficient[1], modelCoefficient[2], modelCoefficient[3]);


using namespace boost;
const int LOWER_BOUND = 100;                     //!< the minmum number one component have 
int TrNode::numLeafs = 0;
Coefficient TrNode::defaultCoef(Point(0,0,0), Point(0,0,1), 1);
std::unordered_set<TrNode,boost::hash<TrNode>> ClusterNode::edgeInfo;
std::unordered_map<int, int> ClusterNode::gId2treeId;
BinaryTree ClusterNode::hierarchyTree;
BinaryTree::EdgeWeightsMap ClusterNode::weights = hierarchyTree.edge_weights();
TrNode BinaryTree::_currentNode;
int ClusterNode::num_triangles = 0;
//std::map<int, std::vector<double>> ClusterNode::size2costs;
pcl::PointCloud<PointT>::Ptr ClusterNode::_cloud;
Coefficient ClusterNode::coef1;
Coefficient ClusterNode::coef2;
Coefficient ClusterNode::coef3;
void ClusterNode::reset(){
    TrNode::numLeafs = 0;
    edgeInfo.clear();
    gId2treeId.clear();
    hierarchyTree.clear(); 
    //    size2costs.clear();
}
//bool ClusterNode::have_reseted(){
//    return TrNode::numLeafs && edgeInfo.size() == 0 && gId2treeId.size() == 0 && hierarchyTree.size() == 0;
//}

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

ClusterNode::ClusterNode(Triangle *t, int index, int cId)
{

    //coef1.resize(4);
    //coef2.resize(4);
    //coef3.resize(7);
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
    _centerIds.push_back(cId);
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

    coef1.point = app;
    coef1.direction = nor;
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

    coef2.point = center;
    coef2.radius = radius;
    //    std::cout<<center.x<<" "<<center.y<<" "<<center.z<<std::endl;
    //    std::cout<<radius<<std::endl;
    return lcost;
}


double ClusterNode::fittingCylinderCost(const void *cn1, const void *cn2)
{
    ClusterNode *n1 = (ClusterNode *)cn1;
    ClusterNode *n2 = (ClusterNode *)cn2;

    double lcost = 0;
    double tot_area = (n1->tot_area+n2->tot_area);
    ClusterNode *gn;
    Triangle *t;
    Node *n, *o;
    Point p, b;
    // radius, axis, center_of_mass
    double radius, area, d, x0, y0;
    Point axis, center_of_mass;
    // center of cylinder
    Point center_of_cyl;

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
    Point ce1, ce2;
    ce1.setValue(evecs[3*index[1]], evecs[3*index[1]+1], evecs[3*index[1]+2]);
    ce2.setValue(evecs[3*index[2]], evecs[3*index[2]+1], evecs[3*index[2]+2]);

    double *pts = new double[(n1->triangles.numels()+n2->triangles.numels())*9];
    int i=0;
    for (gn=n1; gn!=NULL; gn=(gn==n1)?(n2):(NULL))
    {
        o = gn->areas.head(); 
        FOREACHVTTRIANGLE((&(gn->triangles)), t, n)
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
    //
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
    coef3.point = center_of_cyl;
    coef3.direction = axis;
    coef3.radius = radius;

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
    BinaryTree::NodeId child1Id;
    BinaryTree::NodeId child2Id;
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
            TrNode me1(c1->id, c1->childId);
            std::unordered_set<TrNode>::iterator i = edgeInfo.find(me1);
            TrNode leaf;
            leaf.type = i->type;
            leaf.triangles = c1->triangles.toArray();
            leaf.size = c1->triangles.numels();
            std::cout<<leaf.size<<std::endl;
            leaf.id1 = TrNode::numLeafs++;
            // mark the node  a leaf
            leaf.set_leaf();
            child1Id = hierarchyTree.add_node(leaf);
        }
        if(i2 != gId2treeId.end()){
            child2Id = i2->second;
        }else{
            TrNode me1(c2->id, c2->childId);
            std::unordered_set<TrNode>::iterator i = edgeInfo.find(me1);
            TrNode leaf;
            leaf.type = i->type;
            leaf.triangles = c2->triangles.toArray();
            leaf.size = c2->triangles.numels();
            leaf.id1 = TrNode::numLeafs++;
            leaf.set_leaf();
            child2Id = hierarchyTree.add_node(leaf);
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
    c1->childId = c2->id;
    c1->Atb[0] += c2->Atb[0];
    c1->Atb[1] += c2->Atb[1];
    c1->Atb[2] += c2->Atb[2];
    c1->Atb[3] += c2->Atb[3];
    // join two list
    c1->_centerIds.splice(c1->_centerIds.end(), c2->_centerIds);
    c2->_centerIds.clear();
    //
    int size = c1Size + c2Size;
    TrNode me1(c1->id, c2->id);
    std::unordered_set<TrNode>::iterator i = edgeInfo.find(me1);
    //    size2costs[size].push_back(i->cost);
    //
    // create parent node 
    if(c1Size >= LOWER_BOUND || c2Size >= LOWER_BOUND){
        TrNode me1(c1->id, c2->id);
        std::unordered_set<TrNode>::iterator i = edgeInfo.find(me1);
        // attributes of parent is set in function #edgeCostFunction
        // insert a new parent node
        BinaryTree::NodeId parentId = hierarchyTree.add_node(*i);
        hierarchyTree._inStack.push(parentId);
        hierarchyTree.rootId = parentId;
        gId2treeId[c1->id] = parentId;
        // insert edges between child and parent
        auto edge1Id = hierarchyTree.add_edge(parentId, child1Id).first;
        auto edge2Id = hierarchyTree.add_edge(parentId, child2Id).first;
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
    TrNode internalNode(n1->id, n2->id);
    double tot_area = n1->tot_area + n2->tot_area;
    double c1, c2, c3;
    // find the best fitting
    c1 = fittingPlaneCost(cn1, cn2);
    c2 = fittingSphereCost(cn1, cn2);
    c3 = fittingCylinderCost(cn1, cn2);
    //    std::cout<<c1<<" "<<c2<<" "<<c3<<std::endl;
    double cost = c1;
    if (c2 < cost)
        cost = c2;
    if (c3 < cost)
        cost = c3;
    double temp = cost;
    // adjust cost
    if (cost < DBL_MAX) cost += tot_area*ACT_AREA_BIAS;
    // record the fitting cost
    // record the fitting primitive
    Coefficient coefficient;
    if(temp == c1){
        coefficient = coef1;
        internalNode.type = HFP_FIT_PLANES;
        //        std::cout<<"plane"<<std::endl;
    }
    else if(temp == c2){
        coefficient = coef2;
        internalNode.type = HFP_FIT_SPHERES;
        //        std::cout<<"ball"<<std::endl;
    }
    else if(temp == c3){
        coefficient = coef3;
        internalNode.type = HFP_FIT_CYLINDERS;
        //        std::cout<<"cylinder"<<std::endl;
    }
    else{
        assert(false);
    }
    internalNode.cost = cost;
//    internalNode.cost0 = c1;
//    internalNode.cost1 = c2;
//    internalNode.cost2 = c3;
    internalNode.coefficient = coefficient;
    // update the latest before mergeing,
    // after merging, there would be no cost calculation requirement,
    // so this is always right
    auto rst = edgeInfo.insert(internalNode);
    if (!rst.second) {
        edgeInfo.erase(rst.first);
        edgeInfo.insert(rst.first, internalNode);
    }
    internalNode.id1 = n2->id;
    internalNode.id2 = n1->id;
    rst = edgeInfo.insert(internalNode);
    if (!rst.second) {
        edgeInfo.erase(rst.first);
        edgeInfo.insert(rst.first, internalNode);
    }

    return cost;
}

BinaryTree& ClusterNode::cluster(Eye3D *tin){
    // triangle meshes info
    // the destruction function of clusterGraph will free ClusterNode
    clusterGraph cg(tin->E.numels(), &ClusterNode::edgeCostFunction);
    hierarchyTree.clear();

    Triangle *t;
    Edge *e;
    Node *n;
    int i=0;
    // insert nodes to graph
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
    _cloud = temp;

    FOREACHVTTRIANGLE((&(tin->T)), t, n){
        Point p = t->getCenter();
        PointT p2;
        p2.x = (float)p.x;
        p2.y = (float)p.y;
        p2.z = (float)p.z;
        _cloud->points.push_back(p2);
        t->info = cg.addNode(new ClusterNode(t, i++, _cloud->size() - 1));

    }
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
    //    GraphUtil<BinaryTree> gutil(&hierarchyTree);
    //    gutil.write2dot("hierarchy_tree.dot");
    //    std::cout<<"*************nodes:**************"<<std::endl;    
    //    std::cout<<hierarchyTree.num_nodes()<<std::endl;
    //    auto nodeRange = hierarchyTree.get_all_nodes();
    //    for(; nodeRange.first != nodeRange.second; nodeRange.first++){
    //        int id = *nodeRange.first;
    //        const TrNode &node =  hierarchyTree.get_node(id);
    //        std::cout<<node.size<<std::endl;
    //    }
    std::cout<<"num triangle:"<<cg.nodes.numels()<<std::endl;
    num_triangles = cg.nodes.numels();
    std::cout<<"clustering done!"<<std::endl;
    return hierarchyTree;
}

//double ClusterNode::my_fittingCylinderCost(const void *cn1, const void *cn2){
//
//    std::vector<double> distances;
//    ClusterNode *n1 = (ClusterNode *)cn1;
//    ClusterNode *n2 = (ClusterNode *)cn2;
//    // copy point indices
//    std::vector<int> pointIndices(n1->_centerIds.size() + n2->_centerIds.size());
//    int count = 0;
//    for(int p : n1->_centerIds)
//        pointIndices[count++] = p;
//    for(int p : n2->_centerIds)
//        pointIndices[count++] = p;
//    // 
//
//    if (pointIndices.size() <= 20) {
//        return 1000000;
//    }
//    pcl::SampleConsensusModelCylinder<PointT, pcl::Normal>::Ptr
//        model(new pcl::SampleConsensusModelCylinder<PointT,pcl::Normal> (_cloud, pointIndices));
//
//    // Estimate point normals
//    pcl::NormalEstimation<PointT, pcl::Normal> ne;
//    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//    ne.setSearchMethod (tree);
//    ne.setInputCloud (_cloud);
//    ne.setKSearch (50);
//    ne.compute (*cloud_normals);
//    //     
//    model->setInputNormals(cloud_normals); 
//    pcl::RandomSampleConsensus<PointT> ransac (model);
//    ransac.setDistanceThreshold (.01);
//    ransac.computeModel();
//    ransac.getModelCoefficients(coef3);
//    model->getDistancesToModel(coef3, distances);
//
//    double sum = 0;
//    for(double d : distances){
//        if(m_util::is_nun(d))
//            d = 0;
//        sum += d; 
//    }
//    assert(distances.size() > 0);
//    return sum / distances.size();
//}
////! x y z radius
//double ClusterNode::my_fittingSphereCost(const void *cn1, const void *cn2)
//{
//    std::vector<double> distances;
//    std::vector<int> inliers;
//    ClusterNode *n1 = (ClusterNode *)cn1;
//    ClusterNode *n2 = (ClusterNode *)cn2;
//    // copy point indices
//    std::vector<int> pointIndices(n1->_centerIds.size() + n2->_centerIds.size());
//    int count = 0;
//    for(int p : n1->_centerIds)
//        pointIndices[count++] = p;
//    for(int p : n2->_centerIds)
//        pointIndices[count++] = p;
//
//    if (pointIndices.size() <= 80) {
////        return 0;
//        return 1000000;
//    }
//    // 
//    std::cout<<"*********************************"<<std::endl;    
//    pcl::SampleConsensusModelSphere<PointT>::Ptr
//        model(new pcl::SampleConsensusModelSphere<PointT> (_cloud,pointIndices));
//
//    pcl::RandomSampleConsensus<PointT> ransac (model);
//    
//    ransac.setDistanceThreshold (.01);
//    ransac.computeModel();
//    ransac.getModelCoefficients(coef2);
////    ransac.getInliers(inliers);
//    model->getDistancesToModel(coef2, distances);
//
//    double sum = 0;
//    for(double d : distances){
//        if(m_util::is_nun(d))
//            d = 0;
//        sum += d; 
//    }
//    assert(distances.size() > 0);
//    return sum / distances.size();
////    return inliers.size();
//}
////! a b c d 
//double ClusterNode::my_fittingPlaneCost(const void *cn1, const void *cn2)
//{
//    std::vector<double> distances;
//    std::vector<int> inliers;
//    ClusterNode *n1 = (ClusterNode *)cn1;
//    ClusterNode *n2 = (ClusterNode *)cn2;
//    // copy point indices
//    std::vector<int> pointIndices(n1->_centerIds.size() + n2->_centerIds.size());
//    int count = 0;
//    for(int p : n1->_centerIds)
//        pointIndices[count++] = p;
//    for(int p : n2->_centerIds)
//        pointIndices[count++] = p;
//    //
//    if (pointIndices.size() <= 2) {
//        return 0;
//    }
//    // 
//    pcl::SampleConsensusModelPlane<PointT>::Ptr
//        model(new pcl::SampleConsensusModelPlane<PointT> (_cloud, pointIndices));
//    pcl::RandomSampleConsensus<PointT> ransac (model);
//    
//    ransac.setDistanceThreshold (.01);
//    ransac.computeModel();
//    ransac.getModelCoefficients(coef1);
////    ransac.getInliers(inliers);
//    model->getDistancesToModel(coef1, distances);
//
//    double sum = 0;
//    for(double d : distances){
//        if(m_util::is_nun(d))
//            d = 0;
//        sum += d; 
//    }
//    assert(distances.size() > 0);
//    return sum / distances.size();
////    return inliers.size();
//}
