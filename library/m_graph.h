/**
 * @file m_graph.h
 * @brief 
 * @author Dingjie.Wang(dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-11-22
 */
#ifndef M_GRAPH_H

#define M_GRAPH_H



#include <iostream>                  
#include <utility>                   
#include <algorithm>                 
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <ogdf/basic/Graph.h>
#include <ogdf/basic/graph_generators.h>
#include <ogdf/basic/GraphAttributes.h>

using namespace boost;

//    typedef std::pair<int,int> Edge;
//    Edge edges[]={
//        Edge(1,2),
//        Edge(3,2),
//    };
//    double weights[2] = {0,1};
//    m_graph::UWGraph<int> graph(edges, edges + 2, 4, weights);
//    auto i = graph.get_edges();
namespace m_graph {

    /**
     * @brief a simple undirected weighted(type double) graph class wapper
     *
     * @tparam T the type of nodes in graph
     */
    template < typename T>
        class UWGraph {
            public:
                typedef adjacency_list<vecS, vecS, undirectedS, no_property, property<edge_weight_t, double> > MGraph;
                typedef std::pair<T, T> Edge;
                typedef graph_traits<MGraph>::edge_iterator edge_iterator;
                //                typedef graph_traits<MGraph>::vertex_descriptor Vertex;
                UWGraph (Edge *e_begin, Edge *e_end, int num_nodes, double weights[]){
                    _graph = new MGraph(e_begin, e_end, weights, num_nodes);
                }
            public:
                void add_edge(T a, T b){
                    boost::add_edge(a, b, *_graph);
                }
                std::vector<edge_iterator>  get_edges()const
                {
                    //                    std::vector<edge_iterator> edge_iterators;
                    //                    edge_iterator ei, ei_end;
                    //                    for (tie(ei, ei_end) =  boost::edges(*_graph); ei != ei_end; ++ei){
                    ////                        std::cout << "(" << source(*ei, *_graph) << "," << target(*ei, *_graph) << ") ";
                    //                        edge_iterators.push_back(ei);
                    //                        auto a = source(*ei, *_graph);
                    //                    }
                    //                    return edge_iterators;
                }

            private:
                MGraph *_graph;

        };

    //    template < typename T >
    //        std::ostream& operator << (std::ostream& out, const UWGraph<T>& g){
    //
    ////            typename UWGraph<T>::edge_iterator ei, ei_end;
    ////            for (tie(ei, ei_end) =  boost::edges(*_graph); ei != ei_end; ++ei){
    ////                std::cout << "(" << source(*ei, *_graph) << "," << target(*ei, *_graph) << ") ";
    ////            }
    ////            std::cout<<std::endl;
    ////            std::cout << std::endl;
    ////            return out;
    //        }

    template < typename NodeAttrs, typename EdgeAttrs = bool>
        class VizGraph {

            /*---------------------------  lifecycle  ------------------------------------------------ */
            public:
                //VizGraph():_graph(), _viz_attrs(_graph, ogdf::GraphAttributes::nodeGraphics |
                //ogdf::GraphAttributes::edgeGraphics){ } 
                // visualisation attributes of nodes and edges 
                VizGraph(bool directed = false):_graph(), _nodes_attrs(_graph), _edges_attrs(_graph),
                                            _viz_attrs(_graph, ogdf::GraphAttributes::nodeGraphics|
                                                    ogdf::GraphAttributes::edgeGraphics){

                    _viz_attrs.directed(directed);

                }
                ~VizGraph(){ } 
            public:
                //! node handle type
                typedef ogdf::node NodeH;
                //! edge handle type
                typedef ogdf::edge EdgeH;

            public:
                //! construct an random graph
                bool random_graph(int numNodes, int numEdges){
                    return ogdf::randomSimpleGraph(_graph, numNodes, numEdges);
                }
                //! add a new node with node attributes, return handle of the node
                NodeH add_node(NodeAttrs node){
                    NodeH nh = _graph.newNode();
                    _nodes_attrs[nh] = node;
                    return nh;
                }
                //! add a new node, return handle of the node 
                NodeH add_node(){
                    return _graph.newNode();
                }
                //! add a new edge, return handle of the edge
                EdgeH add_edge(NodeH source, NodeH target){
                    return _graph.newEdge(source, target);
                }

                //! add a new edge with edge attributes, return handle of the edge
                EdgeH add_edge(NodeH source, NodeH target, EdgeAttrs etr){
                    EdgeH eh = _graph.newEdge(source, target);
                    _edges_attrs[eh] = etr;
                    return eh;
                }
                //! add a new edge with NodeAttrs directly
                EdgeH add_edge(NodeAttrs source, NodeAttrs target, EdgeAttrs etr){
                    EdgeH eh = _graph.newEdge(add_node(source), add_node(target));
                    _edges_attrs[eh] = etr;
                    return eh;
                }
                //
                EdgeH add_edge(NodeAttrs source, NodeAttrs target){
                    return _graph.newEdge(add_node(source), add_node(target));
                }

                //! set attributes of an node
                void set_node_attrs(NodeH nh, NodeAttrs attrs){
                    _nodes_attrs[nh] = attrs;
                }

                //! get attributes of an node
                NodeAttrs get_node_attrs(NodeH nh){
                    return _nodes_attrs[nh];
                }

                //! set attributes of an edge
                void set_edge_attrs(EdgeH nh, EdgeAttrs attrs){
                    _edges_attrs[nh] = attrs;
                }

                //! get attributes of an edge
                EdgeAttrs get_edge_attrs(EdgeH nh){
                    return _edges_attrs[nh];
                }
                //! save the graph to gml file
                void write(std::string filename){
                    _viz_attrs.writeGML((filename + ".gml").c_str());
                }


                /*-----------------------  attributes  ------------------------------------------------ */
            private:
                ogdf::Graph _graph;
                // custom attributes of nodes and edges
                ogdf::NodeArray<NodeAttrs> _nodes_attrs;
                ogdf::EdgeArray<EdgeAttrs> _edges_attrs;
                // visualisation attributes of graph
                ogdf::GraphAttributes _viz_attrs;

                //GraphAttributes GA(G, GraphAttributes::nodeGraphics |	
                //GraphAttributes::edgeGraphics );


        }; 


} /* m_graph */

#endif /* end of include guard: M_GRAPH_H */
