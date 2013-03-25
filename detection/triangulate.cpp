#include "vizblockworld.h" 
#include "triangulate.h"
#include <iostream>
#include "m_util.h"
#include "graph.h"
#include "m_geometry.h" 
#include <array>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <pcl/visualization/registration_visualizer.h>
#include "icp.h" 
#include <stdio.h>

int vv1(0);
int vv2(1);



using  m_opencv::RgbColor;
using m_geometry::PointF3D;
typedef pcl::PointXYZRGB PointT;
auto &hTree = ClusterNode::hierarchyTree;
typedef std::vector<Triangle*> TrianglePtrs;
void MyTriangulation::triangles_of_component(int root, std::vector<Triangle*> *triangles)const{

    TrianglesOfComponent get_triangles(triangles);
    boost::breadth_first_search(hTree.get_container(), 
            boost::vertex(root, hTree.get_container()), 
            boost::visitor(get_triangles));

}


//std::vector<PointF3D> source;
//std::vector<PointF3D> target;
void MyTriangulation::pos_and_weight(const std::string &fname,
        PointCloudPtr posNodes, vector<float> *wNodes) {
    // Note: BinaryTree is actually a graph type plus binary tree interfaces
    BinaryTree tree;                            // components tree 
    /// get graph model
    unweighted_graphmodel( &tree );

    LDotty<BinaryTree> dot1(&tree);
    dot1.write(fname + "a.dot");


    /// assgin weights of each edge and node
    TopoGraph topography;                       // the final weighted graph model 
    auto nodes = tree.get_all_nodes();
    auto weights = topography.edge_weights();
    const float SIZEBIAS = 3.14 * 4;
    // compute weight of nodes 
    while (nodes.first != nodes.second){
        TrNode temp = tree.get_node(*nodes.first);
        temp.friendId = *nodes.first;
        TrianglePtrs triangles;
        triangles_of_component(*nodes.first, &triangles);
        temp.percent = triangles.size() / (double)ClusterNode::num_triangles;
        temp.weight = TrNode::geometry_weight(temp) + 1 / temp.percent * SIZEBIAS;
        tree.modify_node(*nodes.first, temp);
        nodes.first++;
    }
    nodes = tree.get_all_nodes();
    // copy topographical to final graph model and set edge weights
    while (nodes.first != nodes.second){
        const TrNode &temp = tree.get_node(*nodes.first);
        std::map<BinaryTree::NodeId, int> id2rank;
        vector<float> wChildren;
        map<float, int> wRank;
        // find weight of children
        BinaryTree::OutEdgeRange children = tree.get_out_edges(*nodes.first);
        auto  ctemp = children;
        while( ctemp.first != ctemp.second){
            auto t = tree.targetId(* ctemp.first);
            wChildren.push_back(tree.get_node(t).weight);
            ctemp.first++;
        }
        // rank children weight
        m_util::rank(wChildren, &wRank);
        // map childId to rank
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
            // add edge and set weights
            weights[topography.add_edge(id, childId).first] = id2rank[childId] + 1;
            children.first++;
        }
        nodes.first++;
    }
    std::cout<<"***********weight and type of nodes*******"<<std::endl;    
    nodes = topography.get_all_nodes();
    // set weight of node
    while (nodes.first != nodes.second) {
        auto &temp = tree.get_node(*nodes.first);
        wNodes->push_back(temp.weight);
        nodes.first++;
        std::cout<<"weight:"<<temp.weight<<std::endl;
        std::cout<<"type: "<<(int)temp.type<<std::endl;
    }
    std::cout<<"*********************************"<<std::endl;    


    /// embedding graph to vectors
    m_graph::GraphUtil<TopoGraph> util2(&topography);
    util2.create_matrix();
    util2.johnson_all_pairs_shortest_paths();
    //    util2.print_matrix();
    pos_MDS(posNodes, util2._dMatrix);
    // weighted, neighbour connected graph
    LDotty<TopoGraph> dot(&topography);
    dot.write(fname + "c.dot");
    std::cout<<"write to "<< fname + "c.dot"<<std::endl;
    //

}
void MyTriangulation::topography_weight(BinaryTree *tree, BinaryTree::NodeId nodeId)
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

//        m_graph::GraphUtil<BinaryTree> util0(tree);
//        util0.write2dot("test.dot");
//        util0.create_matrix();
//        util0.to_matrix_graph();
//
//        using namespace boost::python;
//        boost::python::list pyMatrix;
//        _py.vecs2lists(util0._dMatrix, &pyMatrix);
//        _py.load_file("egien.py");
//        object egiens = _py.funObj("egiens");
//        egiens(pyMatrix);
    }else{

    }
    // modify weight of the node
    node.tpWeight = num;
    tree->modify_node(nodeId, node);

}
void MyTriangulation::unweighted_graphmodel(BinaryTree *tree){
    assert(tree);
    _viz->clear();
    /// find meaningful components
    ComponentsInfo info;
    FindComponents cmpComponents(&info, tree);
    boost::breadth_first_search(hTree.get_container(), 
            boost::vertex(ClusterNode::rootId, hTree.get_container()), 
            boost::visitor(cmpComponents));

    /// compute topographical weight
    auto nodesRange = tree->get_all_nodes();
    BinaryTree::NodeId rootId = -1;
    // find root id
    while(nodesRange.first != nodesRange.second){
        // if it's the root 
        if (tree->parentId(*nodesRange.first) == -1) {
            rootId = *nodesRange.first;
            break;
        }
        nodesRange.first++;
    }
    //
    topography_weight(tree, rootId);

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
            info._leafs.push_back(*nodeRange.first);
        }
        nodeRange.first++;
    };
    for(auto id : info._leafs){
        CompSet value;
        value.insert(id);
        TrNode node = tree->get_node(id);
        node.id2 = -1;
        tree->modify_node(id, node);
        TrianglePtrs triangles;
        triangles_of_component(node.friendId, &triangles);
        /// visualize component
        RgbColor c = _colorGenerator.diff_random_color();
        _colorGenerator.accept_color(c);
        viz_component(triangles, c);
        // if vertex of an triangles is shared by some components,
        // then they are adjacent components
        for(Triangle *t : triangles){
            auto v1 = t->v1(); 
            auto v2 = t->v2(); 
            auto v3 = t->v3(); 
            p3[0] = PointF3D(v1->x, v1->y, v1->z);
            p3[1] = PointF3D(v2->x, v2->y, v2->z);
            p3[2] = PointF3D(v3->x, v3->y, v3->z);
            for(auto &p : p3){
                // mark this point belong to the component
                auto rst = points.insert(make_pair(p, value));
                // belong to other components?
                if (!rst.second) {
                    auto &neighborhood = rst.first->second;
                    for(auto nbId : neighborhood){
                        // add edge between adjacent components
                        if (nbId != id) 
                            tree->add_edge(id, nbId);
                    }
                    // mark again
                    neighborhood.insert(id);
                }
            }

        }

    }
}
void 
MyTriangulation::pos_MDS(PointCloudPtr posNodes, const MatrixGraph &dstMatrix){
    // mds
    using namespace boost::python;
    _viz->set_def_cloud(posNodes);
    // Load the sys module.
    _py.load_file("../python/3d_mds.py");
    object mds_3d = _py.funObj("compute");
    boost::python::list pyMatrix;
    _py.vecs2lists(dstMatrix, &pyMatrix);
    object pos = mds_3d(pyMatrix);
    // pos[i] ---- wNodes[i]
    for (int i = 0; i < len(pos); i++) {
        // input to icp
        _viz->add_point(extract<float>(pos[i][0]), extract<float>(pos[i][1]),
                0, 255, 0, 0, 1);
    }
    _viz->remove_def_cloud();
}

float
MyTriangulation::dynamic_EMD(const PointCloudPtr target, const vector<float> &wInput,
        PointCloudPtr source, const vector<float> &wOutput){
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
        _viz->add_sphere(target->points[i].x, target->points[i].y,
                target->points[i].z, wInputRank[i]* radiusArg, 0, 255, 0);

        //        _viz->add_sphere(source->points[i].x + 6, source->points[i].y,
        //                source->points[i].z, wOutputRank[i]* radiusArg, 0, 255, 0);

        _viz->add_sphere(result.points[i].x, result.points[i].y,
                result.points[i].z, wOutputRank[i] * radiusArg, 255, 0, 0);
    }
    //    icp.visualize_correspondence(_viz,0);
    _viz->reset_camera();
    return icp.pre_EMD_;
}
std::string MyTriangulation::viz_component(const std::vector<Triangle*> &triangles, const RgbColor &color)const{
    // display the centers of the triangles
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    _viz->set_def_cloud(cloud);
    for(auto *t : triangles){
        auto p = t->getCenter();
        _viz->add_point((float)p.x, (float)p.y, (float)p.z, color.r, color.g, color.b, 1);
    }

    return _viz->push_def_cloud(vv1, 3);
}

void MyTriangulation::viz_next_level(){
    // making sure the object is segmented
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
        RgbColor c = _colorGenerator.diff_random_color();
        _colorGenerator.accept_color(c);
        // find triangle meshes in the component
        std::vector<Triangle*> triangles;
        triangles_of_component(trId, &triangles);
        // visualize the component and store id
        _vizIds.push(viz_component(triangles, c));
        std::cout<<"*********************************"<<std::endl;    
        std::cout<<(int)trId<<std::endl;
        BinaryTree::_currentNode = hTree.get_node(trId);
        std::cout<<(int)BinaryTree::_currentNode.type<<std::endl;
        std::cout<<BinaryTree::_currentNode.coefficient.point.x<<std::endl;
        std::cout<<BinaryTree::_currentNode.coefficient.radius<<std::endl;
        hTree._outStack.push(trId);
    }
}

void MyTriangulation::viz_previous_level(){
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
void MyTriangulation::change_color(){
    viz_previous_level();
    viz_next_level();
}
void MyTriangulation::segment_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr points){
    _s = true;
    prepare4segmentation();
    pcl::PolygonMesh mesh;
    _viz->generte_mesh(points, &mesh);
    // load mesh
    load_mesh(points, mesh);
    // segment mesh and create hierarchy tree
    auto &tree = ClusterNode::cluster(this);
    // get meaningful components
    //    FindComponents fun;
    //    boost::breadth_first_search(hTree.get_container(), 
    //            boost::vertex(ClusterNode::rootId, hTree.get_container()), 
    //            boost::visitor(fun));
    //    // save hierarchyTree
    //    m_graph::GraphUtil<BinaryTree> gu(&tree);
    //    gu.write2dot("tree1.dot");

}


int MyTriangulation::load_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PolygonMesh &meshes){
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
