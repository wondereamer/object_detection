#include "vizblockworld.h" 
#include "eye3d.h"
#include <iostream>
#include <library/m_util.h>
#include "graph.h"
#include <library/m_geometry.h>
#include <array>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include "icp.h" 
#include <stdio.h>
using  m_opencv::RgbColor;
using m_geometry::PointF3D;

typedef pcl::PointXYZRGB PointT;
typedef std::vector<Triangle*> TrianglePtrS;
extern float ZOOM;
static int vv1(0);
static int vv2(1);
static auto &hTree = ClusterNode::hierarchyTree;


int RefineSegManual::_numComp = 11;

double Eye3D::weight_type(const TrNode &node) const
{
        // standard TrNode
        // shape ---- plane, Coefficient----- 
        // shape
        double bias = 0;
        auto &coef = node.coefficient;
        switch(node.type) {
            case HFP_FIT_PLANES:
                {
                    bias = 0;
                    //                    // 0 - 3.14
                    //                    double angle = acos(((float)coef.direction.x * (float)defaultCoef.direction.x + (float)coef.direction.y * (float)defaultCoef.direction.y
                    //                                + (float)coef.direction.z * (float)defaultCoef.direction.z) /
                    //                            (sqrt(pow((float)coef.direction.x,2)+pow((float)coef.direction.y, 2)+pow((float)coef.direction.z, 2))*
                    //                             sqrt(pow((float)defaultCoef.direction.x, 2)+pow((float)defaultCoef.direction.y, 2)+pow((float)defaultCoef.direction.z, 2))));
                    //                    // 
                    //                    double dist = sqrt(pow(((float)coef.point.x - (float)defaultCoef.point.x), 2) + pow(((float)coef.point.y-(float)defaultCoef.point.y), 2)
                    //                            +pow(((float)coef.point.z - (float)defaultCoef.point.z), 2));
                    //                    bias += angle * 0.7 + atan(dist) * 0.6;
                    break;
                }
            case HFP_FIT_CYLINDERS:
                bias = 3.14 * 2;
                break;
            case HFP_FIT_SPHERES:
                bias = 3.14 * 4;
                break;
            default:
                assert(false);
        };        
        // percent

        return bias;
}

double Eye3D::weight_size(const TrNode &node) const
{
}

double Eye3D::weight_degree(const TrNode &node) const
{
}

double Eye3D::weight_pos(const TrNode &node) const
{
}

double Eye3D::weight_angle(const TrNode &node) const
{
}
void Eye3D::triangles_of_component(int root, std::vector<Triangle*> *triangles)const
{

    TrianglesOfComponent get_triangles(triangles);
    boost::breadth_first_search(hTree.get_container(), 
            boost::vertex(root, hTree.get_container()), 
            boost::visitor(get_triangles));

}

void Eye3D::graphic_model(TopoGraph *leafGraph)
{
    // Note: BinaryTree is actually a graph type plus binary tree interfaces
    BinaryTree tree;                            // components tree 
    ComponentsInfo compInfo;
    /// get graph model
    unweighted_graphic_model( &tree , &compInfo);
    /// assgin weights of each edge and node
    TopoGraph topography;                       // the final weighted graph model 
    auto nodes = tree.get_all_nodes();
    auto weights = topography.edge_weights();
    // copy topographical to final graph model and set edge weights
    nodes = tree.get_all_nodes();
    while (nodes.first != nodes.second){
        const TrNode &temp = tree.get_node(*nodes.first);
        // rank weight of target nodes(children)
        std::map<BinaryTree::NodeId, int> id2rank;
        vector<float> wChildren;                // weights of children 
        map<float, int> wRank;                  // rank of weight 
        BinaryTree::OutEdgeRange children = tree.get_out_edges(*nodes.first);
        auto  ctemp = children;
        while( ctemp.first != ctemp.second){
            auto t = tree.targetId(* ctemp.first);
            wChildren.push_back(tree.get_node(t).weight);
            ctemp.first++;
        }
        m_util::rank(wChildren, &wRank);
        ctemp = children;
        while(ctemp.first != ctemp.second){
            auto cId = tree.targetId(*ctemp.first);
            float w = tree.get_node(cId).weight;
            id2rank.insert(make_pair(cId, wRank[w]));
            ctemp.first++;
        }
        // add edges, nodes and set edge weights
        children = tree.get_out_edges(*nodes.first);
        BinaryTree::OutEdgeRange children2 = tree.get_out_edges(*nodes.first);
        auto id = topography.add_node(temp);
        while(children.first != children.second){
            // add child node
            auto t = tree.targetId(*children.first);
            TopoGraph::NodeId childId = topography.add_node(tree.get_node(t));
            // add edge and set weights (equal to rank of weight of children)
            weights[topography.add_edge(id, childId).first] = id2rank[childId] + 1;
            children.first++;
        }
        nodes.first++;
    }
    // save weighted, neighbour connected graph
    LDotty<TopoGraph> dot(&topography);
    dot.write("topgraphy.dot");
    std::cout<<"write to "<< "topgraphy.dot"<<std::endl;
    std::cout<<topography.num_nodes()<<std::endl;


    // copy leafs subgraph
    topography.copy_subgraph(compInfo._leafs , leafGraph);
    assert((int)leafGraph->num_nodes() == compInfo._leafs.size());

}

void Eye3D::refine_segmentation(BinaryTree *tree, ComponentsInfo *compInfo,
                                bool isManual)
{

    std::cout<<"************find meaningful components*********************"<<std::endl;    
    if( isManual ){
        RefineSegManual cmpComponents(compInfo, tree);
        boost::breadth_first_search(hTree.get_container(), 
                                    boost::vertex(ClusterNode::hierarchyTree.rootId,
                                    hTree.get_container()), 
                                    boost::visitor(cmpComponents));
    }
    else{
        RefineSegAuto cmpComponents(compInfo, tree);
        boost::breadth_first_search(hTree.get_container(), 
                                    boost::vertex(ClusterNode::hierarchyTree.rootId,
                                    hTree.get_container()), 
                                    boost::visitor(cmpComponents));
    }
}
void Eye3D::unweighted_graphic_model(BinaryTree *tree, ComponentsInfo *compInfo)
{
    assert(tree);
    _viz->clear();
    /// find meaningful components
    refine_segmentation(tree, compInfo);
    /// compute topographical weight
    auto nodesRange = tree->get_all_nodes();
    BinaryTree::NodeId rootId = 0;
//    topography_weight(tree, rootId);
    /// connect adjacent components (leafs)
    typedef std::set<BinaryTree::NodeId> CompSet;
    std::map<PointF3D, CompSet> points;
    std::array<PointF3D,3> p3;
    auto nodeRange = tree->get_all_nodes();
    // find leafs
    while(nodeRange.first != nodeRange.second){
        BinaryTree::OutEdgeRange rst = tree->get_out_edges(*nodeRange.first);
        // if is a leaf
        if (rst.first == rst.second) {
            compInfo->_leafs.push_back(*nodeRange.first);
        }
        nodeRange.first++;
    };
    
    for(auto id : compInfo->_leafs){
        CompSet components;
        components.insert(id);
        TrNode node = tree->get_node(id);
        TrianglePtrS triangles;
        triangles_of_component(node.friendId, &triangles);
        // mark leaf
        node.set_leaf();
        PointF3D center;
        for(Triangle *t : triangles){
            auto v1 = t->v1(); 
            auto v2 = t->v2(); 
            auto v3 = t->v3(); 
            p3[0] = PointF3D(v1->x, v1->y, v1->z);
            p3[1] = PointF3D(v2->x, v2->y, v2->z);
            p3[2] = PointF3D(v3->x, v3->y, v3->z);
            PointF3D t;
            for(auto &p : p3){
                /// if vertex of an triangles is shared by some components,
                /// then they are adjacent components
                // add current component to the containers(owners)
                auto rst = points.insert(make_pair(p, components));
                if (!rst.second) {
                    // there is some other adjacent components contain the point
                    auto &neighborhood = rst.first->second;
                    for(auto nbId : neighborhood){
                        // add edge between adjacent components
                        if (nbId != id) 
                            tree->add_edge(id, nbId);
                    }
                    // add current component to the containers
                    neighborhood.insert(id);
                }
                t += p;
            }
            node.center += (t/3);
        }
        const float SIZEBIAS = 3.14 * 4;

        node.center /= triangles.size();
        node.proportion = triangles.size() / (double)ClusterNode::num_triangles;
        node.weight = weight_type(node) + 1 / node.proportion * SIZEBIAS;
        node.degree = tree->get_degree(id);
        tree->modify_node(id, node);
    }

}

void Eye3D::topography_weight(BinaryTree *tree, BinaryTree::NodeId nodeId)
{
    auto children = tree->childrenId(nodeId);
    TrNode node = tree->get_node(nodeId);
    int num = 1;
    if (children.first != -1) {
        assert(children.second != -1);
        topography_weight(tree, children.first);
        topography_weight(tree, children.second);
        // compute topographical signature
        BinaryTree subTr1, subTr2;
        tree->sub_tree(&subTr1, children.first);
        tree->sub_tree(&subTr2, children.second);
        num += subTr1.num_nodes() + subTr2.num_nodes();

    }else{

    }
    // modify weight of the node
    node.tpWeight = num;
    tree->modify_node(nodeId, node);

}
//-------------------------------------matching---------------------------------------------
void 
Eye3D::pos_MDS(PointCloudPtr posNodes, const MatrixGraph &dstMatrix) 
{
    // mds
    using namespace boost::python;
//    _viz->set_def_cloud(posNodes);
    // Load the sys module.
    _py.load_file("../python/3d_mds.py");
    object mds_3d = _py.funObj("compute");
    boost::python::list pyMatrix;
    _py.vecs2lists(dstMatrix, &pyMatrix);
    object pos = mds_3d(pyMatrix);
//    // pos[i] ---- wNodes[i]
//    for (int i = 0; i < len(pos); i++) {
//        // input to icp
//        _viz->add_point(extract<float>(pos[i][0]), extract<float>(pos[i][1]),
//                0, 255, 0, 0, 1);
//    }
//    _viz->remove_def_cloud();
}

void
Eye3D::embedding_graph(PointCloudPtr pos, const TopoGraph &graph) 
{

    m_graph::GraphUtil<TopoGraph> util2(const_cast<TopoGraph*>(&graph));
    util2.create_matrix();
    util2.johnson_all_pairs_shortest_paths();
    pos_MDS(pos, util2._dMatrix);
}
float
Eye3D::dynamic_EMD(const PointCloudPtr target, const vector<float> &wInput,
        PointCloudPtr source, const vector<float> &wOutput, const vector<int> &leafs)
{
    _viz->clear();
    float radiusArg = 0.05;
    IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputCloud(source);
    icp.setInputWeight(wInput);
    icp.setInputTarget(target);
    icp.setOutputWeight(wOutput);
    pcl::PointCloud<PointT> result;
    // Set the transformation epsilon (criterion 2)
    icp.setMaximumIterations (150);
    icp.setMaxCorrespondenceDistance (100);
    icp.setTransformationEpsilon (1e-11);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1e-11);
    icp.setRANSACIterations(0); 
    icp.align(result);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    int size = result.points.size();
    // rank the weights
    std::map<float, int> w2rank;
    std::vector<float> wInputRank;
    m_util::rank(wInput, &w2rank);
    for(float w: wInput){
        wInputRank.push_back(w2rank[w] + 1);
    }
    w2rank.clear();
    std::vector<float> wOutputRank;
    m_util::rank(wOutput, &w2rank);
    for(float w: wOutput){
        wOutputRank.push_back(w2rank[w] + 1);
    }
    for (int i = 0; i < size; i++) {
        int a = 0;
        int g = 255;
        if(leafs[i]){
            // mark leafs different
            a = 255;
            g = 0;
        }
        _viz->add_sphere(target->points[i].x, target->points[i].y,
                target->points[i].z, wInputRank[i]* radiusArg, 0, g, a);

        //        _viz->add_sphere(source->points[i].x + 6, source->points[i].y,
        //                source->points[i].z, wOutputRank[i]* radiusArg, 0, 255, 0);

        _viz->add_sphere(result.points[i].x, result.points[i].y,
                result.points[i].z, wOutputRank[i] * radiusArg, 255, 0, a);
    }
    //    icp.visualize_correspondence(_viz,0);
    _viz->reset_camera();
    return icp.pre_EMD_;
}
//-------------------------------------visualize---------------------------------------------
std::string Eye3D::viz_component(const std::vector<Triangle*> &triangles,
                            const RgbColor &color, PointT *min, PointT *max) const
{
    // display the centers of the triangles
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    _viz->set_def_cloud(cloud);
    for(auto *t : triangles){
        auto p = t->getCenter();
        _viz->add_point((float)p.x, (float)p.y, (float)p.z, color.r, color.g, color.b, ZOOM);
    }
    if( min && max )
        m_util::boundary(cloud->points, min, max);
    return _viz->push_def_cloud(vv1, 3);
}
void Eye3D::viz_components( TopoGraph &leafGraph) const
{
    /// coloring the vertex
    m_graph::GraphUtil<TopoGraph> uu(&leafGraph);
    std::map<int, int> colorMap;
    uu.to_matrix_graph();
    m_opencv::RandomColor color(50);
    uu.vertex_coloring(color._defColors.size(), colorMap);
    auto nodes = leafGraph.get_all_nodes();
    /// visualize
    int index = 0;
    for(; nodes.first != nodes.second ; nodes.first++, index++){
        TrNode node = leafGraph.get_node(*nodes.first);
        TrianglePtrS triangles;
        triangles_of_component(node.friendId, &triangles);
        auto temp = color._defColors.begin();
        for (int i = 0; i < colorMap[index]; temp++, i++) ;
//        RgbColor c = temp->second;
        RgbColor c = color.diff_random_color();
        color.accept_color(c);
        PointT min, max;
        viz_component(triangles, c, &min, &max);
        double xx = max.x - min.x;
        double yy = max.y - min.y;
        double zz = max.z - min.z;
        double bound = xx > yy? xx: yy;
        bound = bound > zz? bound: zz;          // bounding of shape primitive
        auto &coef = node.coefficient;
//        switch((int)node.type) {
//            case HFP_FIT_PLANES:
//                std::cout<<"**plane"<<std::endl;
//                _viz->add_plane(coef.point.x*ZOOM, coef.point.y*ZOOM, coef.point.z*ZOOM,
//                            coef.direction.x*ZOOM, coef.direction.y*ZOOM, coef.direction.z*ZOOM);
//                break;
//            case HFP_FIT_SPHERES:
//                std::cout<<"**sphere"<<std::endl;
//                _viz->add_sphere(coef.point.x*ZOOM, coef.point.y*ZOOM, coef.point.z*ZOOM,
//                            coef.radius*ZOOM);
//                break;
//            case HFP_FIT_CYLINDERS:
//                std::cout<<"**clinders"<<std::endl;
//                _viz->add_cylinder(coef.point.x*ZOOM, coef.point.y*ZOOM, coef.point.z*ZOOM,
//                            coef.direction.x*ZOOM, coef.direction.y*ZOOM, coef.direction.z*ZOOM, 
//                            coef.radius*ZOOM, bound);
//
//                break;
//            
//        }
    }
}
void Eye3D::viz_next_level()
{
    // making sure the object is segmented
    static string shapeId;
    if(!_s){
        std::cerr<<"You have to  invoke #segment_points before invoking #viz_next_level!"
            <<std::endl;
        assert(_s);
    }
    if (!hTree._inStack.empty()) {
        // get component id
        auto trId = hTree._inStack.top();
        hTree._inStack.pop();
        if (hTree.is_brother_of_out(trId)){
            viz_next_level();
            return;
        }
//        RgbColor c = _colorGenerator.diff_random_color();
        RgbColor c = _colorGenerator.random_color();
        _colorGenerator.accept_color(c);
        // find triangle meshes in the component
        std::vector<Triangle*> triangles;
        triangles_of_component(trId, &triangles);
        // visualize the component and store id
        PointT min, max;
        _vizIds.push(viz_component(triangles, c, &min, &max));
        double xx = max.x - min.x;
        double yy = max.y - min.y;
        double zz = max.z - min.z;
        double bound = xx > yy? xx: yy;
        bound = bound > zz? bound: zz;
        std::cout<<"*********************************"<<std::endl;    
        std::cout<<(int)trId<<std::endl;
        BinaryTree::_currentNode = hTree.get_node(trId);
        if (shapeId != "") 
            _viz->remove_shape(shapeId);
        auto &coef = BinaryTree::_currentNode.coefficient;
        switch((int)BinaryTree::_currentNode.type) {
            case HFP_FIT_PLANES:
                std::cout<<"**plane"<<std::endl;
                shapeId = _viz->add_plane(coef.point.x*ZOOM, coef.point.y*ZOOM, coef.point.z*ZOOM,
                            coef.direction.x*ZOOM, coef.direction.y*ZOOM, coef.direction.z*ZOOM);
                break;
            case HFP_FIT_SPHERES:
                std::cout<<"**sphere"<<std::endl;
                shapeId = _viz->add_sphere(coef.point.x*ZOOM, coef.point.y*ZOOM, coef.point.z*ZOOM,
                            coef.radius*ZOOM);
                break;
            case HFP_FIT_CYLINDERS:
                std::cout<<"**clinders"<<std::endl;
                std::cout<<"bound:"<<bound<<std::endl;
                shapeId = _viz->add_cylinder(coef.point.x*ZOOM, coef.point.y*ZOOM, coef.point.z*ZOOM,
                            coef.direction.x*ZOOM, coef.direction.y*ZOOM, coef.direction.z*ZOOM, 
                            coef.radius*ZOOM, bound);

                break;
            
        }
        std::cout<<BinaryTree::_currentNode.cost<<std::endl;
        hTree._outStack.push(trId);
    }
}

void Eye3D::viz_previous_level()
{
    if (!_vizIds.empty()) {
        // remove the component in viualizer
        auto &vizId = _vizIds.top();
        _viz->remove_cloud(vizId);
        _vizIds.pop();
        // update level
        auto id = hTree._outStack.top();
        hTree._inStack.push(id);
        std::cout<<"node: "<<id<<" and "<<hTree.brotherId(id)<<" "; 
        std::cout<<"merge to node: "<<hTree.parentId(id)<<std::endl;
        hTree._outStack.pop();
        BinaryTree::_currentNode = hTree.get_node(id);
        _colorGenerator.pop_color();
    }
}
void Eye3D::change_color()
{
    viz_previous_level();
    viz_next_level();
}
//-------------------------prepare-----------------------------------------------------------
void Eye3D::segment_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr points,
                                    float radius)
{
    _s = true;
    prepare4segmentation();
    pcl::PolygonMesh mesh;
    _viz->generte_mesh(points, &mesh, radius);
    assert(mesh.polygons.size() > 0 );
    // load mesh
    load_mesh(points, mesh);
    // segment mesh and create hierarchy tree
    auto &tree = ClusterNode::cluster(this);

}


int Eye3D::load_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PolygonMesh &meshes)
{
    FILE *fp;
    Node *n;
    int num_mesh = meshes.polygons.size();
    int num_vertex = num_mesh * 3;
    int i,j,i1,i2,i3;
    Vertex *v;
    ExtVertex **var = (ExtVertex **)malloc(sizeof(ExtVertex *)*num_vertex);
    if ((fp = fopen("temp" ,"w")) == NULL) return IO_CANTOPEN;
    // create vertex array
    for(auto &p : cloud->points){
        // the real vertex, would be freed by parent
        V.appendTail(new Vertex(p.x, p.y, p.z));
    }
    //
    i=0; 
    FOREACHVERTEX(v, n) 
        var[i++] = new ExtVertex(v);
    JMesh::begin_progress();
    for(const pcl::Vertices &mesh : meshes.polygons){
        int i1 = mesh.vertices[0];
        int i2 = mesh.vertices[1];
        int i3 = mesh.vertices[2];
        if (i1 == i2 || i2 == i3 || i3 == i1) 
            JMesh::warning("\nloadOFF: Coincident indexes at triangle %d! Skipping.\n",i);
        else if (!CreateIndexedTriangle(var, i1, i2, i3))
            JMesh::warning("\nloadOFF: This shouldn't happen!!! Skipping triangle.\n");
    };
    JMesh::end_progress();
    // close #fp and free #var
    closeLoadingSession(fp, i, var, false);
    if(shells() > 1){
        removeSmallestComponents();
        std::cout<<"**small components are removed!**"<<std::endl;
    }
    //    // may added some new vertex
    //    std::cout<<"*********triangulate.h***********"<<std::endl;    
    //    std::cout<<V.numels()/3<<" *"<<num_mesh<<std::endl;
    //    saveOFF("temp.off");
    return 0;
}

