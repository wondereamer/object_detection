#include <iostream>                  
#include <utility>                   
#include <algorithm>                 
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

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

} /* m_graph */

