/****************************************************************************
 * EfPiSoft                                                                  *
 *                                                                           *
 * Consiglio Nazionale delle Ricerche                                        *
 * Istituto di Matematica Applicata e Tecnologie Informatiche                *
 * Sezione di Genova                                                         *
 * IMATI-GE / CNR                                                            *
 *                                                                           *
 * Authors: Marco Attene                                                     *
 *                                                                           *
 * Copyright(C) 2006: IMATI-GE / CNR                                         *
 *                                                                           *
 * All rights reserved.                                                      *
 *                                                                           *
 * This program is free software; you can redistribute it and/or modify      *
 * it under the terms of the GNU General Public License as published by      *
 * the Free Software Foundation; either version 2 of the License, or         *
 * (at your option) any later version.                                       *
 *                                                                           *
 * This program is distributed in the hope that it will be useful,           *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 * GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
 * for more details.                                                         *
 *                                                                           *
 ****************************************************************************/
#ifndef TRIANGULATE_H

#define TRIANGULATE_H



#include "jmesh.h"
#include "cluster_node.h"
#include <iostream>
#include "graph.h" 
#include <boost/graph/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <pcl/surface/gp3.h>
#include <library/m_opencv.h>
#include <library/m_util.h>
#include <cassert>
#include <boost/graph/breadth_first_search.hpp>
#include <stack>
using m_opencv::RandomColor;
using m_graph::MatrixGraph;
class VizBlockWorld;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
template < typename T >
class LDotty:public m_graph::DottyOutput<T> 
{
    public:
        LDotty (T *g): m_graph::DottyOutput<T>(g){

        };

        virtual void do_draw_node(typename T::NodeId id){
            //10[fillcolor="red"][shape="rect"][style="filled"][color="green"][weight=5][height=10]
            assert(this->_fout);
            auto &node = this->_g->get_node(id);
            *(this->_fout)<<m_util::string_format("%d", id);
            if (node.is_leaf()) {
                // is leaf
                *(this->_fout)<<"[color = \"red\"][style = \"filled\" ][fixedsize = \"true\" ]" ;
            }
            *(this->_fout)<<"[fixedsize = \"true\" ]"<<std::endl;
        }

        virtual void do_draw_edge(typename T::EdgeId eid){
            auto *_fout = this->_fout;
            auto *_g = this->_g;
            assert(_fout);
            static typename T::EdgeWeightsMap edgeWeights = _g->edge_weights();
            bool isnb = false;
            auto &t = _g->get_node(_g->targetId(eid));
            auto &s = _g->get_node(_g->sourceId(eid));
            if (t.parentId != _g->sourceId(eid)) {
                isnb = true;
            }
            std::string tL = m_util::string_format("%d", _g->targetId(eid));
            std::string sL = m_util::string_format("%d", _g->sourceId(eid));
            if (this->_directed) 
                *_fout <<sL << " -> " << tL
                    << "[label=" << edgeWeights[eid] << "]";
            else
                *_fout <<sL << " -- " << tL
                    << "[label=" << edgeWeights[eid] << "]";
            if( isnb )
                *_fout<<"[color = \"red\" ]\n" ;
            else
                *_fout<<"\n" ;
        }
        virtual ~LDotty (){ };

};
//
//! find triangles of specific meaningful component
class TrianglesOfComponent : public boost::default_bfs_visitor
{
    public:
        std::vector<Triangle*> *_triangles;
        TrianglesOfComponent(std::vector<Triangle*> *t):_triangles(t){ }
        template < typename Vertex, typename Graph >
            void discover_vertex(Vertex u, const Graph & g) const
            {
                auto &tree = ClusterNode::hierarchyTree;
                auto &node = tree.get_node(u);
                // is leaf node?
                if (node.id2 == -1) {
                    // making sure every node in the hierarchy tree have two children
                    assert(node.size >= 2);
                    for (int i = 0; i < node.size; i++) {
                        _triangles->push_back(static_cast<Triangle*>(node.triangles[i]));
                    }
                }
            }
};
struct ComponentsInfo{
    ComponentsInfo(){
        _num_components = 0;
    }
    int _num_components;
    std::vector<BinaryTree::NodeId> _leafs;
    m_opencv::RgbColor _color;
    void clear(){
        _num_components = 0;
        _leafs.clear();
    }
};

//! find meaningful components for every object manually
class RefineSegManual : public boost::default_bfs_visitor
{
    public:
        ComponentsInfo *_compInfo;
        BinaryTree *_btree;
        // number of components user required
        static int _numComp;


        RefineSegManual(ComponentsInfo *c, BinaryTree *tr):_compInfo(c),_btree(tr){ }
        template < typename Vertex, typename Graph >
            // modify ClusterNode::hierarchyTree and generate _btree
            // _btree does not contain triangles information
            void discover_vertex(Vertex u, const Graph & g)
            {
                if(_compInfo->_num_components < RefineSegManual::_numComp){
                    auto &tree = ClusterNode::hierarchyTree;
                    auto &node = tree.get_node(u);
                    auto inEdges = tree.get_in_edges(u);
                    auto outEdges = tree.get_out_edges(u);
                    const BinaryTree::NodeType *parent = NULL;
                    const BinaryTree::NodeType *child1 = NULL;
                    const BinaryTree::NodeType *child2 = NULL;
                    auto children = tree.childrenId(u);
                    TrNode temp;
                    temp = node;
                    temp.friendId = (int)u;
                    temp.parentId = node.parentId;

                    // id in components tree
                    int id = _btree->add_node(temp);
                    if(inEdges.first != inEdges.second){
                        parent = &tree.get_node(tree.sourceId(*inEdges.first));
                    }
                    //
                    if (children.first != -1) {
                        child1 = &tree.get_node(children.first);
                        TrNode temp = *child1;
                        // parentId of nodes in friend _btree 
                        // a bad style
                        temp.parentId = id;
                        tree.modify_node(children.first, temp);
                    }

                    if (children.second != -1) {
                        child2 = &tree.get_node(children.second);
                        TrNode temp = *child2;
                        temp.parentId = id;
                        tree.modify_node(children.second, temp);
                    }

                    //
                    if (node.parentId != -1) 
                        _btree->add_edge(node.parentId, id);
                    // if node is an leaf node

                    _compInfo->_num_components++;
                    //                    _compInfo->_components.push_back(&node);
                }

            }
        //        void clear_components(){
        //            _compInfo._num_components = 0;
        //            _compInfo._components.clear();
        //        }
};

//! find meaningful components for every object, just one time
class RefineSegAuto : public boost::default_bfs_visitor
{
    public:
        ComponentsInfo *_compInfo;
        BinaryTree *_btree;


        RefineSegAuto(ComponentsInfo *c, BinaryTree *tr):_compInfo(c),_btree(tr){ }
        template < typename Vertex, typename Graph >
            // modify ClusterNode::hierarchyTree and generate _btree
            // _btree does not contain triangles information
            void discover_vertex(Vertex u, const Graph & g)
            {
                auto &tree = ClusterNode::hierarchyTree;
                auto &node = tree.get_node(u);
                if ( u == ClusterNode::hierarchyTree.rootId ||
                    node.parentId != -1) {
                    bool enable_divide = true;
                    auto inEdges = tree.get_in_edges(u);
                    auto outEdges = tree.get_out_edges(u);
                    const BinaryTree::NodeType *parent = NULL;
                    const BinaryTree::NodeType *child1 = NULL;
                    const BinaryTree::NodeType *child2 = NULL;
                    auto children = tree.childrenId(u);
                    TrNode temp;
                    temp = node;
                    temp.friendId = (int)u;
                    temp.parentId = node.parentId;

                    // id in components tree
                    int id = _btree->add_node(temp);
                    if(inEdges.first != inEdges.second){
                        parent = &tree.get_node(tree.sourceId(*inEdges.first));
                    }
                    if (children.first != -1 && children.second != -1) {
                        child1 = &tree.get_node(children.first);
                        child2 = &tree.get_node(children.second);
                        auto &c1 = *child1;
                        auto &c2 = *child2;
                        // whther this component could be divided to two parts
                        if (c1.type == c2.type) {
                            // snipet
                            auto p1 = c1.coefficient.point;
                            auto p2 = c2.coefficient.point;
                            auto r1 = c1.coefficient.radius;
                            auto r2 = c2.coefficient.radius;
                            auto d1 = c1.coefficient.direction;
                            auto d2 = c2.coefficient.direction;
                            float cDiff = sqrt(pow(p1.x - p2.x, 2)+
                                    pow(p1.y - p2.y, 2)+
                                    pow(p1.z - p2.z, 2));
                            float rDiff = abs(r1 - r2);
                            float lR = r1 > r2 ? r1 : r2;

                            float product = d1.x * d2.x + d1.y * d2.y+
                                d1.z * d2.z;
                            float nm1 = sqrt(d1.x*d1.x + d1.y*d1.y+
                                    d1.z*d1.z);
                            float nm2 = sqrt(d2.x*d2.x + d2.y*d2.y+
                                    d2.z*d2.z);
                            float cosV = product/(nm1 * nm2);
                            float dDiff = acos(cosV) * 180 / 3.14;
                            switch((int)c1.type) {
                                case HFP_FIT_PLANES:
                                    if((!m_util::is_nun(dDiff)) && dDiff < 2)
                                        enable_divide = false;
                                    else if(node.parentId != -1 && node.parentId != 0 &&
                                             tree.get_node(node.parentId).type == HFP_FIT_PLANES)
                                            // if it's parent is a plane then disable' 
                                            enable_divide = false;
                                    break;

                                case HFP_FIT_SPHERES:
                                    if(rDiff / lR < 0.5 && cDiff / lR < 0.5)
                                        enable_divide = false;
                                    break;
                                case HFP_FIT_CYLINDERS:
                                    if((!m_util::is_nun(dDiff)) && dDiff < 25 && rDiff / lR < 0.3)
                                    {
                                        enable_divide = false;
                                    }
                                    break;
                            }
                            
                            if (m_util::is_nun(dDiff)) {
                                enable_divide = false;
                                if (c1.type == HFP_FIT_SPHERES &&
                                   (!(rDiff / lR < 0.5 && cDiff / lR < 0.5))) {
                                    enable_divide = true;
                                    std::cout<<"divide two similar shape: " 
                                             <<(int)c1.type<<std::endl;
                                }else if(c1.type == HFP_FIT_SPHERES)
                                    std::cout<<"merge two similar shape: "
                                             <<(int)c1.type<<std::endl;
                            }else{
                                if(!enable_divide)
                                    std::cout<<"merge two similar shape: "
                                             <<(int)c1.type<<std::endl;
                                else
                                    std::cout<<"divide two similar shape: " 
                                             <<(int)c1.type<<std::endl;

                            }
                                


//                            std::cout<<"find similar shape type: "<<(int)node.type<<std::endl;
//                            std::cout<<"dDiff: "<<dDiff<<std::endl;
//                            std::cout<<"cDiff: "<<cDiff<<std::endl;
//                            std::cout<<"rDiff: "<<rDiff<<std::endl;
//                            std::cout<<"lR: "<<lR<<std::endl;
//                            std::cout<<"rDiff / lR: "<<rDiff/lR<<std::endl;
//                            std::cout<<"cDiff / lR: "<<cDiff/lR<<std::endl;

                        }
                    }

                    // mark children if divideable or have different types
                    if (enable_divide && child1 && child2) {
                        TrNode temp = *child1;
                        temp.parentId = id;
                        tree.modify_node(children.first, temp);
                        temp = *child2;
                        temp.parentId = id;
                        tree.modify_node(children.second, temp);

                    std::cout<<id<<std::endl;
                    }
                    //
                    if (node.parentId != -1) 
                        _btree->add_edge(node.parentId, id);
                    _compInfo->_num_components++;
                    //                    _compInfo->_components.push_back(&node);
                }

            }
        //        void clear_components(){
        //            _compInfo._num_components = 0;
        //            _compInfo._components.clear();
        //        }
};
class Eye3D :public Triangulation
{
    public:

        typedef adjacency_list<setS, vecS, undirectedS, no_property,
                property<edge_weight_t, int> > boostGraph2;
        typedef m_graph::AutoUniGraph<boostGraph2, TrNode> TopoGraph;

        Eye3D (VizBlockWorld *viz):_viz(viz), _s(false), _colorGenerator(50){ }
        virtual ~Eye3D (){ };
        //-------------------------------------graphic model---------------------------------------------
        //! compute the graph model
        void graphic_model(TopoGraph *leafGraph);
        double weight_type(const TrNode &node) const;
        double weight_size(const TrNode &node) const;
        double weight_degree(const TrNode &node) const;
        double weight_pos(const TrNode &node) const;
        double weight_angle(const TrNode &node) const;
        //-------------------------------------matching---------------------------------------------
        // red -- target; green -- source; blue -- result
        //! compute dynamic emd distance and display result
        float dynamic_EMD(const PointCloudPtr target, const vector<float> &wInput,
                PointCloudPtr source, const vector<float> &wOutput,
                const vector<int> &leafs);
        //! embedding the nodes of graph to vectors
        void embedding_graph(PointCloudPtr pos, const TopoGraph &graph);
        //! segment the object recursively

        //! making the segmentation result more sense to human being
        void refine_segmentation(BinaryTree *tree, ComponentsInfo *compInfo, 
                                 bool isManual = true);

        //-------------------------------------visualize---------------------------------------------
        //! segment the object
        void segment_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr points,
                float radius);
        //! get triangles of a specific node in the tree
        void triangles_of_component(int root, std::vector<Triangle*> *t)const;
        //! visualize the component and return it's id in visualizeer
        std::string viz_component(const std::vector<Triangle*> &triangles,
                const m_opencv::RgbColor &color, 
                PointT *min = NULL,
                PointT *max = NULL) const;
        //! visualize meaningful components
        void viz_components(TopoGraph &leafGraph) const;
        void viz_next_level();
        void viz_previous_level();
        void change_color();

    protected:
        inline void prepare4segmentation()const{
            ClusterNode::reset();
        }
        //! embedding an graph to vectors(postions) in normal space with MDS
        void pos_MDS(PointCloudPtr posNodes, const MatrixGraph &dstMatrix);

        //! load meshes which generate by  PointCloud Library
        int load_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                const pcl::PolygonMesh &meshes);
        /**
         * @brief find meaningful sub-tree and connect adjacent components(leafs), 
         * ending width a graph. and display leafs
         */
        void unweighted_graphic_model(BinaryTree *tree, ComponentsInfo *compInfo);

        //! this function is not used right now
        void topography_weight(BinaryTree *tree, BinaryTree::NodeId nodeId);
    private:
        VizBlockWorld *_viz;
        std::stack<std::string> _vizIds; //!< store id of components in visualizer
        m_opencv::RandomColor _colorGenerator;
        m_util::PyModule _py;

        // if the mesh segmented
        bool _s;


};


#endif /* end of include guard: TRIANGULATE_H */
