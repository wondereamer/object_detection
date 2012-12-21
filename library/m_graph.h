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
#include <ogdf/basic/List.h>
#include <ogdf/basic/String.h>
#include <ogdf/basic/graph_generators.h>
#include <ogdf/basic/GraphAttributes.h>
#include <vector>
#include <fstream>
#include <algorithm>
#include <functional>

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

    //! matrix graph
    typedef std::vector< std::vector<int> > MatrixGraph;

    //! read_graph
    void read_graph(std::string filename, MatrixGraph &graph){
        int N, edge;
        std::ifstream infile (filename);
        infile>>N;
        for(int i=0; i<N; i++)
        {
            std::vector<int> row;
            for(int j=0; j<N; j++)
            {
                infile>>edge;
                row.push_back(edge);
            }
            graph.push_back(row);
        }
    }
    //! print graph
    void print_graph(MatrixGraph &graph){
        // print matrix graph
        std::cout<<graph.size()<<std::endl;
        for(auto &row : graph){
            for(int v : row){
                std::cout<<" "<<v;
            }
            std::cout<<std::endl;
        }

    }

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

    /**
     * @brief vertexing color algorithm from internet:
     *
     * http://www.dharwadker.org/vertex_coloring/
     * @param Graph: matrix graph
     * @param colorMap: map from color index to string color
     * @param rst: returned, map from vertex index to string color
     *
     * @return: if the algorithm could give an result 
     */
    bool vertex_coloring(const MatrixGraph &graph, std::map<int, std::string> &colorMap,
            std::map<int, std::string> &rst);

    /**
     * @brief 
     *
     * @tparam NodeAttrs
     * @tparam EdgeAttrs
     */
    template < typename NodeAttrs, typename EdgeAttrs = bool>
        class VizGraph {


            public:
                //VizGraph():_graph(), _viz_attrs(_graph, ogdf::GraphAttributes::nodeGraphics |
                //ogdf::GraphAttributes::edgeGraphics){ } 
                // visualisation attributes of nodes and edges 
                VizGraph(bool directed = false):_graph(), _nodes_attrs(_graph), _edges_attrs(_graph),_directed(directed),
                _viz_attrs(_graph, ogdf::GraphAttributes::nodeGraphics|
                        ogdf::GraphAttributes::edgeGraphics|
                        ogdf::GraphAttributes::nodeColor|
                        ogdf::GraphAttributes::nodeId|
                        ogdf::GraphAttributes::edgeDoubleWeight){

                    _viz_attrs.directed(directed);

                }
                ~VizGraph(){ } 
            public:
                //! node handle type
                typedef ogdf::node NodeH;
                //! edge handle type
                typedef ogdf::edge EdgeH;
                //! matrix graph
                typedef std::vector< std::vector<int> > MatrixGraph;
                //! data structure to make sure edge is unique in graph
                struct Uni_Edge {
                    Uni_Edge(NodeH src, NodeH tgt):_src(src), _tgt(tgt){ }
                    bool operator < (const Uni_Edge &r) const{
                        if( _src < r._src)
                            return true;
                        else if( _src > r._src)
                            return false;
                        else
                            return _tgt < r._tgt;
                    }

                    NodeH _src;
                    NodeH _tgt;
                };
                struct _less_edge : public std::binary_function<EdgeH, EdgeH, bool> {
                    _less_edge(const VizGraph<NodeAttrs, EdgeAttrs> &graph):_graph(graph){ }
                    //
                    bool  operator()(EdgeH arg1, EdgeH arg2) const{
                        return _graph.get_edge_weight(arg1) < _graph.get_edge_weight(arg2);
                    }
                    protected: 
                    const VizGraph<NodeAttrs, EdgeAttrs>  &_graph;
                };

            public:
                //! construct an random graph
                bool random_graph(int numNodes, int numEdges){
                    return ogdf::randomSimpleGraph(_graph, numNodes, numEdges);
                }
                //! add a new node with node attributes, return handle of the node
                NodeH add_node(NodeAttrs node, int size = 1, std::string color = "#000000"){
                    NodeH nh = _graph.newNode();
                    _nodes_attrs[nh] = node;
                    _viz_attrs.width(nh) = size;
                    _viz_attrs.height(nh) = size;
                    _viz_attrs.colorNode(nh) = ogdf::String(color.c_str());
                    return nh;
                }
                //! add a new node, return handle of the node 
                NodeH add_node(int size = 1, std::string color = "#00ff00"){
                    NodeH nh = _graph.newNode();
                    _viz_attrs.width(nh) = size;
                    _viz_attrs.height(nh) = size;
                    _viz_attrs.colorNode(nh) = ogdf::String(color.c_str());
                    return nh;
                }
                //! add a new edge, return handle of the edge
                EdgeH add_edge(NodeH source, NodeH target){
                    Uni_Edge uniEdge(source, target);
                    // exist, return
                    auto e = _edges_set.find(uniEdge);
                    if(e != _edges_set.end())
                        return e->second;
                    if(!_directed){
                        // check reverse edge if this is an undirected graph
                        Uni_Edge uniEdge(target, source);
                        auto e = _edges_set.find(uniEdge);
                        if(e != _edges_set.end())
                            return e->second;
                    }
                    // add new edge
                    EdgeH eh =  _graph.newEdge(source, target);
                    _edges_set.insert(std::make_pair(uniEdge, eh));
                    return eh;
                }

                //! add a new edge with edge attributes, return handle of the edge
                EdgeH add_edge(NodeH source, NodeH target, EdgeAttrs etr){
                    Uni_Edge uniEdge(source, target);
                    // exist, return
                    auto e = _edges_set.find(uniEdge);
                    if(e != _edges_set.end())
                        return e->second;
                    if(!_directed){
                        // check reverse edge if this is an undirected graph
                        Uni_Edge uniEdge(target, source);
                        auto e = _edges_set.find(uniEdge);
                        if(e != _edges_set.end())
                            return e->second;
                    }
                    // add new edge
                    EdgeH eh = _graph.newEdge(source, target);
                    _edges_set.insert(std::make_pair(uniEdge, eh));
                    // modify attribution
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
                //!
                void set_node_color(NodeH nh, const std::string color){
                    _viz_attrs.colorNode(nh) = ogdf::String(color.c_str());
                }
                //! 
                void set_edge_weight(EdgeH eh, double w){
                    _viz_attrs.doubleWeight(eh) = w;
                }
                //! 
                double get_edge_weight(EdgeH eh) const{
                    return _viz_attrs.doubleWeight(eh);
                }
                void set_node_pos(NodeH nh, double x, double y){
                    _viz_attrs.x(nh) = x;
                    _viz_attrs.y(nh) = y;
                }
                //! get attributes of an node
                NodeAttrs& get_node_attrs(NodeH nh){
                    return _nodes_attrs[nh];
                }

                //! get attributes of an node
                const NodeAttrs& get_node_attrs(NodeH nh)const {
                    return _nodes_attrs[nh];
                }
                //! set attributes of an edge
                void set_edge_attrs(EdgeH nh, EdgeAttrs attrs){
                    _edges_attrs[nh] = attrs;
                }

                //! get attributes of an edge
                EdgeAttrs& get_edge_attrs(EdgeH nh){
                    return _edges_attrs[nh];
                }

                //! get attributes of an edge
                const EdgeAttrs& get_edge_attrs(EdgeH nh) const{
                    return _edges_attrs[nh];
                }
                //! save the graph to gml file
                void write(std::string filename){
                    _viz_attrs.writeGML((filename + ".gml").c_str());
                }
                //! return first node in the node list
                NodeH first_node() const{ return _graph.firstNode();}
                //! return last node in the node list
                NodeH last_node() const{ return _graph.lastNode(); }

                //! return first edge in the edge list
                NodeH first_edge() const{ return _graph.firstEdge();}
                //! return last edge in the edge list
                NodeH last_edge() const{ return _graph.lastEdge(); }
                //! get adjacency list of the node
                void adjacency_nodes(NodeH nh, std::vector<NodeH> &rst){
                    rst.clear();
                    ogdf::List<ogdf::adjEntry> entries;
                    _graph.adjEntries(nh, entries);
                    for(auto entry : entries){
                        rst.push_back(entry->twinNode());
                    }
                }
                //! get edges connecting the node
                void adjacency_edges(NodeH nh, std::vector<EdgeH> &rst){
                    rst.clear();
                    ogdf::List<EdgeH> edges;
                    _graph.adjEdges(nh, edges);
                    for(auto edge : edges){
                        rst.push_back(edge);
                    }
                }
                //! get all nodes in the graph
                void all_nodes( std::vector<NodeH> &rst){
                    rst.clear();
                    ogdf::List<NodeH> nodes;
                    _graph.allNodes(nodes);
                    rst.resize(nodes.size());
                    int index = 0;
                    for(NodeH node : nodes){
                        rst[index++] = node;
                    }
                }

                //! get all edges in the graph
                void all_edges( std::vector<EdgeH> &rst){
                    rst.clear();
                    ogdf::List<EdgeH> edges;
                    _graph.allEdges(edges);
                    rst.resize(edges.size());
                    int index = 0;
                    for(EdgeH edge : edges){
                        rst[index++] = edge;
                    }
                }
                //! sorted in increasing oder of weight
                void sorted_edges(std::vector<EdgeH> &rst){
                    all_edges(rst);
                    std::sort(rst.begin(), rst.end(), _less_edge(*this));
                }
                //! index map to allNodes list
                void matrix_graph(MatrixGraph &matrixGraph){
                    std::vector<NodeH> nodes;
                    // get all vertex
                    all_nodes(nodes);
                    for(NodeH nh : nodes){
                        // for every row
                        std::vector<int> row(nodes.size(), 0);
                        // get adjacency list of the node
                        std::vector<NodeH> adjs;
                        adjacency_nodes(nh, adjs);
                        for(NodeH adj: adjs){
                            // find index of the adjacency vertex in all nodes list
                            int index = 0;
                            for(NodeH  t: nodes){
                                if(t == adj)
                                    break;
                                index++;
                            }
                            // mark the adjacency vertex
                            row[index] = 1;
                        }
                        // add the row
                        matrixGraph.push_back(row);
                    }
                }
                //! remove edge
                void remove_edge(EdgeH eh){
                    _graph.delEdge(eh);
                    _edges_set.erase(Uni_Edge(eh->source(), eh->target()));
                }
                //! remove node
                void remove_node(NodeH nh){
                    // remove related item in unique edge set
                    ogdf::List<EdgeH> edges;
                    _graph.adjEdges(nh, edges);
                    for(auto eh : edges){
                        _edges_set.erase(Uni_Edge(eh->source(), eh->target()));
                    }
                    // remove node and related edges in graph
                    _graph.delNode(nh);
                }
                //!
                int size() const{
                    return _graph.numberOfNodes();
                }


                //! color the vertex of graph to using vertex coloring algorithm
                bool vertex_coloring(std::map<int, std::string> &colormap, 
                        std::map<int, std::string> &vertex2color){
                    MatrixGraph mGraph;
                    matrix_graph(mGraph);
                    /*print_graph(mGraph);*/
                    bool rst = m_graph::vertex_coloring(mGraph, colormap, vertex2color);
                    if(rst){
                        // modify color attribution of nodes in graph
                        std::cout<<"coloring vertex sucess!*****"<<std::endl;
                        std::vector<NodeH> nodeHandles;
                        all_nodes(nodeHandles);
                        int index;
                        std::string color;
                        for(auto m : vertex2color){
                            index = m.first;
                            color = m.second;
                            // @bound to matrix_graph()
                            set_node_color(nodeHandles[index], color);
                        }
                        return true;
                    }
                    std::cout<<"coloring vertex failed!*****"<<std::endl;
                    return false;
                }
            protected: 


            private:
                ogdf::Graph _graph;
                // custom attributes of nodes and edges
                ogdf::NodeArray<NodeAttrs> _nodes_attrs;
                ogdf::EdgeArray<EdgeAttrs> _edges_attrs;
                // visualisation attributes of graph
                ogdf::GraphAttributes _viz_attrs;
                //! edge set, to make sure edge is unique in graph

                //GraphAttributes GA(G, GraphAttributes::nodeGraphics |	
                //GraphAttributes::edgeGraphics );
                bool _directed;
                std::map<Uni_Edge, EdgeH> _edges_set;



        }; 




} /* m_graph */

#endif /* end of include guard: M_GRAPH_H */
