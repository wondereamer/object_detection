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


int RefineSegManual::_numComp = 6;
int RefineSegAuto::_numComp = 6;

void Eye3D::triangles_of_component(int root, std::vector<Triangle*> *triangles)const
{

    TrianglesOfComponent get_triangles(triangles);
    boost::breadth_first_search(hTree.get_container(), 
            boost::vertex(root, hTree.get_container()), 
            boost::visitor(get_triangles));

}

void Eye3D::graphic_model(TopoGraph *topography)
{
    // Note: BinaryTree is actually a graph type plus binary tree interfaces
    BinaryTree tree;                            // components tree 
    ComponentsInfo compInfo;
    const int INCREMANT = 5;
    const int BASIC = 500;
    /// get graph model
    unweighted_graphic_model( &tree , &compInfo);
    /// assgin weights of each edge according to the size of node
    auto nodes = tree.get_all_nodes();
    auto weights = topography->edge_weights();
    // copy topographical to final graph model and set edge weights
    nodes = tree.get_all_nodes();
    while (nodes.first != nodes.second){
        const TrNode &temp = tree.get_node(*nodes.first);
        // rank size of target nodes(children)
        std::map<BinaryTree::NodeId, int> id2rank;
        vector<int> sizeChildren;                // weights of children 
        map<int, int> sizeRank;                  // rank of weight 
        BinaryTree::OutEdgeRange children = tree.get_out_edges(*nodes.first);
        auto  ctemp = children;
        while( ctemp.first != ctemp.second){
            auto t = tree.targetId(* ctemp.first);
            sizeChildren.push_back(tree.get_node(t).size);
            ctemp.first++;
        }
        m_util::rank(sizeChildren, &sizeRank);
        ctemp = children;
        while(ctemp.first != ctemp.second){
            auto cId = tree.targetId(*ctemp.first);
            float w = tree.get_node(cId).size;
            id2rank.insert(make_pair(cId, sizeRank[w]));
            ctemp.first++;
        }
        // add edges, nodes and set edge weights
        children = tree.get_out_edges(*nodes.first);
        BinaryTree::OutEdgeRange children2 = tree.get_out_edges(*nodes.first);
        auto id = topography->add_node(temp);
        while(children.first != children.second){
            // add child node
            auto t = tree.targetId(*children.first);
            TopoGraph::NodeId childId = topography->add_node(tree.get_node(t));
            // add edge 
            auto eid = topography->add_edge(id, childId).first;
            if (topography->get_node(childId).is_leaf() &&
                temp.is_leaf()) 
                weights[eid] = BASIC;
            else
                // set weights (equal to rank of weight of children)
                weights[eid] = id2rank[childId] * INCREMANT + BASIC;
            children.first++;
        }
        nodes.first++;
    }
//    // save weighted, neighbour connected graph
//    LDotty<TopoGraph> dot(topography);
//    dot.write("topgraphy.dot", true);
//    std::cout<<"write to "<< "topgraphy.dot"<<std::endl;
//    std::cout<<topography->num_nodes()<<std::endl;

//    // copy leafs subgraph
//    topography.copy_subgraph(compInfo._leafs , leafGraph);
//    assert((int)leafGraph->num_nodes() == compInfo._leafs.size());

}

void Eye3D::refine_segmentation(BinaryTree *tree, ComponentsInfo *compInfo,
                                bool isManual)
{

//    std::cout<<"************find meaningful components*********************"<<std::endl;    
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
    m_graph::DottyOutput<BinaryTree> dot0(tree);
    dot0.write("tree.dot");
    std::cout<<"write to "<< "tree.dot"<<std::endl;
    m_graph::DottyOutput<BinaryTree> dot1(&ClusterNode::hierarchyTree);
    dot1.write("htree.dot");
    std::cout<<"write to "<< "htree.dot"<<std::endl;

    /// 
    auto nodesRange = tree->get_all_nodes();
    BinaryTree::NodeId rootId = 0;
    /// connect adjacent components (leafs)
    typedef std::set<BinaryTree::NodeId> CompSet;
    std::map<PointF3D, CompSet> points;
    std::array<PointF3D,3> p3;
    auto nodeRange = tree->get_all_nodes();
    std::vector<BinaryTree::NodeId> tab;
    bool onlyLeaf = true;
    // connect adjacent leaf nodes 
    while(nodeRange.first != nodeRange.second){
        auto id = *nodeRange.first;
        TrNode node = tree->get_node(id);
        if (!onlyLeaf || node.is_leaf()) {
            CompSet components;
            components.insert(id);
            TrianglePtrS triangles;
            triangles_of_component(node.friendId, &triangles);
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
                    if (node.is_leaf()) {
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
                    }
                    t += p;
                }
                // t/3 == the center of one triangles mesh
                node.center += (t/3);
            }
            node.center /= triangles.size();
            node.proportion = node.size / float(hTree.get_node(hTree.rootId).size);
            //        node.weight = weight_type(node) + 1 / node.proportion * SIZEBIAS;
            node.degree = tree->get_degree(id);
            node.dist = sqrt(pow(node.center.x - _center.x, 2) + pow(node.center.y - _center.y, 2) +
                    pow(node.center.z - _center.z, 2));
            tree->modify_node(id, node);
        }
        nodeRange.first++;
    }
    /// angle, only for leafs
    int maxSize = -1;
    PointF3D maxCenter; 
    for(auto id : compInfo->_leafs){
        TrNode node = tree->get_node(id);
        if (node.size > maxSize) {
            maxCenter = node.center;
        }
    }
    // the base line
    float x0 = _center.x - maxCenter.x;
    float y0 = _center.y - maxCenter.y;
    float z0 = _center.z - maxCenter.z;
    for(auto id : compInfo->_leafs){
        TrNode node = tree->get_node(id);
        float x = _center.x - node.center.x;
        float y = _center.y - node.center.y;
        float z = _center.z - node.center.z;
        node.angle = (x*x0 + y*y0 + z*z0) / (sqrt(pow(x0, 2) + pow(y0,2) + pow(z0,2))*
                                             sqrt(pow(x, 2)+ pow(y, 2) + pow(z, 2)));
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
    object mds_3d = _py.funObj("compute");
    boost::python::list pyMatrix;
    _py.vecs2lists(dstMatrix, &pyMatrix);
    object pos = mds_3d(pyMatrix);
    // pos[i] ---- wNodes[i]
    _viz->set_def_cloud(posNodes);
    for (int i = 0; i < len(pos); i++) {
        // input to icp
        _viz->add_point(extract<float>(pos[i][0]), extract<float>(pos[i][1]),
                0, 255, 0, 0, 1);
    }
    _viz->remove_def_cloud();
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
        PointCloudPtr source, const vector<float> &wOutput)
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
    if (!icp.hasConverged()) {
        std::cout << "ICP failed to converged!"<<std::endl;
        return -1;
    }
//    /// visualize the align result
//    int size = result.points.size();
//    // rank the weights
//    std::map<float, int> w2rank;
//    std::vector<float> wInputRank;
//    m_util::rank(wInput, &w2rank);
//    for(float w: wInput){
//        wInputRank.push_back(w2rank[w] + 1);
//    }
//    w2rank.clear();
//    std::vector<float> wOutputRank;
//    m_util::rank(wOutput, &w2rank);
//    for(float w: wOutput){
//        wOutputRank.push_back(w2rank[w] + 1);
//    }
//    for (int i = 0; i < size; i++) {
//        _viz->add_sphere(target->points[i].x,target->points[i].y,
//                 target->points[i].z, wInputRank[i]* radiusArg, 0, 255, 0);
//
//        //        _viz->add_sphere(source->points[i].x + 6, source->points[i].y,
//        //                source->points[i].z, wOutputRank[i]* radiusArg, 0, 255, 0);
//
//        _viz->add_sphere(result.points[i].x, result.points[i].y,
//                result.points[i].z, wOutputRank[i] * radiusArg, 255, 0, 0);
//    }
//    //    icp.visualize_correspondence(_viz,0);
//    _viz->reset_camera();
    return icp.best_EMD_;
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
        RgbColor c = temp->second;
//        RgbColor c = color.diff_random_color();
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
void Eye3D::viz_skeleton(TopoGraph &model)const
{

    // visualize skeleton
    _viz->clear();
    PointCloudPtr centers(new pcl::PointCloud<PointT>);
    _viz->set_def_cloud(centers);
    _viz->add_point(_center.x, _center.y, _center.z, 255,0,0);
    auto leafs = model.get_all_nodes();
    TopoGraph::NodeId maxId = 0;
    int maxSize = -1;
    while (leafs.first != leafs.second){
        auto node = model.get_node(*leafs.first);
        if(node.size > maxSize) {
            maxSize = node.size;
            maxId = *leafs.first;
        };
        auto &center = node.center;
        _viz->add_point(center.x, center.y, center.z, 0,255,0);
        _viz->add_line(_center.x, _center.y, _center.z,
                       center.x, center.y, center.z,
                       0, 255, 0);
        leafs.first++;
    }
    auto p = model.get_node(maxId).center;
    _viz->add_line(_center.x, _center.y, _center.z,
            p.x, p.y, p.z, 255, 0, 0);
    _viz->push_def_cloud(0,9);

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
//        switch((int)BinaryTree::_currentNode.type) {
//            case HFP_FIT_PLANES:
//                std::cout<<"**plane"<<std::endl;
//                shapeId = _viz->add_plane(coef.point.x*ZOOM, coef.point.y*ZOOM, coef.point.z*ZOOM,
//                            coef.direction.x*ZOOM, coef.direction.y*ZOOM, coef.direction.z*ZOOM);
//                break;
//            case HFP_FIT_SPHERES:
//                std::cout<<"**sphere"<<std::endl;
//                shapeId = _viz->add_sphere(coef.point.x*ZOOM, coef.point.y*ZOOM, coef.point.z*ZOOM,
//                            coef.radius*ZOOM);
//                break;
//            case HFP_FIT_CYLINDERS:
//                std::cout<<"**clinders"<<std::endl;
//                std::cout<<"bound:"<<bound<<std::endl;
//                shapeId = _viz->add_cylinder(coef.point.x*ZOOM, coef.point.y*ZOOM, coef.point.z*ZOOM,
//                            coef.direction.x*ZOOM, coef.direction.y*ZOOM, coef.direction.z*ZOOM, 
//                            coef.radius*ZOOM, bound);
//
//                break;
//            
//        }
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
    PointF3D center;
    Vertex *v;
    ExtVertex **var = (ExtVertex **)malloc(sizeof(ExtVertex *)*num_vertex);
    if ((fp = fopen("temp" ,"w")) == NULL) return IO_CANTOPEN;
    // create vertex array
    for(auto &p : cloud->points){
        // the real vertex, would be freed by parent
        V.appendTail(new Vertex(p.x, p.y, p.z));
        _center.x += p.x;
        _center.y += p.y;
        _center.z += p.z;
    }
    _center /= cloud->points.size();
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

