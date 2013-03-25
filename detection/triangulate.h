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
#include "m_opencv.h"
#include "m_util.h"
#include <cassert>
#include <boost/graph/breadth_first_search.hpp>
#include <stack>
using m_opencv::RandomColor;
using m_graph::MatrixGraph;
class VizBlockWorld;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
template < typename T >
class LDotty:public m_graph::DottyOutput<T> {
public:
    LDotty (T *g): m_graph::DottyOutput<T>(g){

    };

    virtual void do_draw_node(typename T::NodeId id){
        //10[fillcolor="red"][shape="rect"][style="filled"][color="green"][weight=5][height=10]
        assert(this->_fout);
        *(this->_fout)<<id;
        auto &node = this->_g->get_node(id);
        if (node.id2 == -1) {
            *(this->_fout)<<"[color = \"red\"][style = \"filled\" ]";
        }
        *(this->_fout)<<std::endl;
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

//! find meaningful components for every object, just one time
class FindComponents : public boost::default_bfs_visitor
{
    public:
        ComponentsInfo *_compInfo;
        BinaryTree *_btree;


        FindComponents(ComponentsInfo *c, BinaryTree *tr):_compInfo(c),_btree(tr){ }
        template < typename Vertex, typename Graph >
            // modify ClusterNode::hierarchyTree and generate _btree
            // _btree does not contain triangles information
            void discover_vertex(Vertex u, const Graph & g)
            {
                if(_compInfo->_num_components < 11){
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
class MyTriangulation :public Triangulation{
    public:
        MyTriangulation (VizBlockWorld *viz):_viz(viz), _s(false), _colorGenerator(50){ }
        virtual ~MyTriangulation (){ };
        //! get triangles of a specific node in the tree
        void triangles_of_component(int root, std::vector<Triangle*> *t)const;
        //! visualize the component and return it's id in visualizeer
        std::string viz_component(const std::vector<Triangle*> &triangles,
                                        const m_opencv::RgbColor &color)const;
        void viz_next_level();
        void viz_previous_level();
        void change_color();
        //! compute postions with MDS and calculate weights of node
        void pos_and_weight(const std::string &fname,
                                PointCloudPtr posNodes, vector<float> *wNodes);
        // red -- target; green -- source; blue -- result
        //! compute dynamic emd distance and display result
        float dynamic_EMD(const PointCloudPtr target, const vector<float> &wInput,
                            PointCloudPtr source, const vector<float> &wOutput);

        //! 
        void segment_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr points);

    protected:
        typedef adjacency_list<vecS, vecS, undirectedS, no_property,
                             property<edge_weight_t, int> > boostGraph2;
        typedef m_graph::AutoUniGraph<boostGraph2, TrNode> TopoGraph;
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
        void unweighted_graphmodel(BinaryTree *tree);

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
