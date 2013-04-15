/**
 * @file graph.h
 * @brief graph classes based on boost graph library
 * @author Dingjie.Wang(dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2013-01-07
 */
#ifndef GRAPH_H

#define GRAPH_H
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/graph/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <map>
#include <cassert>
#include <fstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <library/m_util.h>
#include "graph_draw.h" 
/// @todo replace map with unordered_map
enum vertex_node_t { vertex_node };
namespace boost {
    BOOST_INSTALL_PROPERTY(vertex, node);
}
namespace m_graph {
    
/******** 
  1) install new property type:
  enum vertex_properties_t { vertex_properties };
  namespace boost {
  BOOST_INSTALL_PROPERTY(vertex, properties);
  }
  // http://stackoverflow.com/questions/671714/modifying-vertex-properties-in-a-boostgraph
  2) default property tags:
  vertex_index_t
  edge_index_t
  graph_name_t
  vertex_name_t
  edge_name_t
  edge_weight_t
  vertex_distance_t
  vertex_color_t
  vertex_degree_t
  vertex_out_degree_t
  vertex_in_degree_t
  vertex_discover_time_t
  vertex_finish_time_t 
  */

typedef std::vector<std::vector<int> > MatrixGraph;

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
    bool vertex_coloring(const MatrixGraph &graph, int colorNum,
            std::map<int, int> &rst);

/**
 * @brief a helper class carring out some drawing function and algorithms related to graph
 *
 * @tparam Graph the graph
 */
template < typename Graph >
class GraphUtil {
    public:
    GraphUtil (Graph *graph):_g(graph){
        assert(graph);  
    };

    /// @todo output node , output .gml
    //! save the graph to dot file
    void write2dot(std::string filename){
        DottyOutput<Graph> dotty(_g);
        dotty.write(filename);
    }
    //! calcuating the length of shortest path between all pairs
    void johnson_all_pairs_shortest_paths(){
        //            assert(_dMatrix);
        boost::johnson_all_pairs_shortest_paths(_g->get_container(), _dMatrix);
    }

    //! if type is a directedS, then the result would be bidirectionalS!
    void random_graph(int num_vertex, int num_edge){
        boost::mt19937 gen;
        boost::generate_random_graph(_g->get_container(), num_vertex, num_edge, gen);
        //randomize_property<edge_weight_t>(g.get_container(), rand_int);
    }
    //! create an matrix related to *latest graph
    inline void create_matrix(){
        _dMatrix.clear();
        std::vector<int> row(_g->num_nodes(), 0);
        for (int i = 0; i < _g->num_nodes(); i++) 
            _dMatrix.push_back(row);
    }
    
    //! print matrix attached to the graph
    void print_matrix() const{
        int V = _g->num_nodes();
        std::cout << "       ";
        for (int k = 0; k < V; ++k)
            std::cout << std::setw(5) << k;
        std::cout << std::endl;
        for (int i = 0; i < V; ++i) {
            std::cout << std::setw(3) << i << " -> ";
            for (int j = 0; j < V; ++j) {
                if (_dMatrix[i][j] == (std::numeric_limits<int>::max)())
                    std::cout << std::setw(5) << "inf";
                else
                    std::cout << std::setw(5) << _dMatrix[i][j];
            }
            std::cout << std::endl;
        }

    }
    /// @todo strategy model
    //! index map to allNodes list
    void to_matrix_graph( ){
        create_matrix();
        // get all vertex
        typename Graph::NodeRange nodeRange = _g->get_all_nodes();
        // map node id to vertex num [0,n]
        std::map<typename Graph::NodeId, int> id2index;
        int row = 0;
        for(auto i = nodeRange.first; i != nodeRange.second; i++, row++){
            id2index[*i] = row;
        }
        for(row = 0; nodeRange.first != nodeRange.second; nodeRange.first++, row++){
            // for each node
            int row = id2index[*nodeRange.first];
            // get adjacent nodes <out>
            typename Graph::AdjRange adjRange = _g->get_adj_nodes(*nodeRange.first);
            for(; adjRange.first != adjRange.second; adjRange.first++){
                int col = id2index[*adjRange.first];
                // mark out edge
                _dMatrix[row][col] = 1;
                _dMatrix[col][row] = 1;
            }

        }
    }

    bool vertex_coloring(int colorNum, std::map<int, int> &vertex2color)
    {
        // map from vertex num to color
        std::map<int, int> temp;
        bool rst = m_graph::vertex_coloring(_dMatrix, colorNum, temp);
        if(rst){
            std::cout<<"coloring vertex sucess!*****"<<std::endl;
            auto nodeRange = _g->get_all_nodes();
            auto ii = temp.begin();
            for(auto i = nodeRange.first; i != nodeRange.second; 
                i++, ii++){
                // map from id to color
                vertex2color.insert(make_pair(*i, ii->second));

            }
            return true;
        }
        std::cout<<"coloring vertex failed!*****"<<std::endl;
        return false;
    }
    virtual ~GraphUtil (){ };

    private:
    Graph *_g;                              //!< the graph 
    public:
    MatrixGraph _dMatrix;                        //!< distance matrix 
};
using namespace boost;
/// @todo subgraph copy
//#define graph_traits boost::graph_traits
//#define property_map boost::property_map
/**
 * @brief an wrapper class to operators of boost graph class,
 *  node is the vertex
 *  
 * usage example:
 *   typedef adjacency_list<vecS, vecS, undirectedS/directedS/bidirectionalS, no_property, 
 *                       property<edge_weight_t, int> > boostGraph;
 *   
 *   typedef BaseGraph<boostGraph> MyGraph;
 * @tparam the boost graph
 */
template < typename GraphContainer>
class BaseGraph
{
    public:

        /* a bunch of graph-specific typedefs */
        typedef typename graph_traits<GraphContainer>::vertex_descriptor NodeId;
        typedef typename graph_traits<GraphContainer>::edge_descriptor EdgeId;
        typedef typename property_map<GraphContainer, boost::edge_weight_t>::type EdgeWeightsMap;
        typedef typename property_map<GraphContainer, boost::edge_weight_t>::const_type ConstEdgeWeightsMap;
        typedef std::pair<NodeId, NodeId> EdgePair;
        typedef typename graph_traits<GraphContainer>::vertex_iterator NodeIter;
        typedef typename graph_traits<GraphContainer>::edge_iterator EdgeIter;
        typedef typename graph_traits<GraphContainer>::adjacency_iterator AdjIter;
        typedef typename graph_traits<GraphContainer>::out_edge_iterator OutEdgeIter;
        typedef typename graph_traits<GraphContainer>::in_edge_iterator InEdgeIter;
        typedef typename graph_traits<GraphContainer>::degree_size_type DegreeSize;
        typedef typename graph_traits<GraphContainer>::vertices_size_type SizeType;

        typedef std::pair<AdjIter, AdjIter> AdjRange;
        typedef std::pair<NodeIter, NodeIter> NodeRange;
        typedef std::pair<OutEdgeIter, OutEdgeIter> OutEdgeRange;
        typedef std::pair<InEdgeIter, InEdgeIter> InEdgeRange;
        typedef std::pair<EdgeIter, EdgeIter> EdgeRange;
        /* constructors etc. */
        BaseGraph() {
        }
        BaseGraph(int numNode) :_g(numNode){
        }

        BaseGraph(const BaseGraph& g) : _g(g._g) {
        }

        /* operators */
        BaseGraph& operator = (const BaseGraph &rhs)
        {
            _g = rhs._g;
            return *this;
        }
        virtual ~BaseGraph() {

        }
    public:
        inline NodeId add_node() {
            return boost::add_vertex(this->_g);
        }
        inline void remove_node(const NodeId& id)
        {
            boost::clear_vertex(id, this->_g);
            boost::remove_vertex(id, this->_g);
        }

        /* structure modification methods */
        inline void clear()
        {
            clear_();
            _g.clear();
        }
        virtual void clear_(){ }

        // If putting this function to a loop, program would be time comsuming
        inline const EdgeWeightsMap edge_weights() const{
            return boost::get(boost::edge_weight, _g);
        }
        // If putting this function to a loop, program would be time comsuming
        //! return a reference to weight attribute of edges
        inline EdgeWeightsMap edge_weights() {
            return boost::get(boost::edge_weight, _g);
        }
        //! you could add adge without adding related node 
        inline std::pair<EdgeId, bool>
            add_edge(const NodeId& source, const NodeId& target)
            {
                // assert #source and #target is in the graph
                auto i = boost::add_edge(source, target, _g);
                return i;
            }
        //!
        inline EdgePair get_edge(const EdgeId& id) const {
            return std::make_pair(boost::source(id, _g), boost::target(id, _g));

        }
        //!
        inline NodeId sourceId(const EdgeId& id){
            return boost::source(id,_g);
        }
        //!
        inline NodeId sourceId(const EdgeId& id) const{
            return boost::source(id,_g);
        }

        inline NodeId targetId(const EdgeId& id){
            return boost::target(id,_g);
        }

        inline NodeId targetId(const EdgeId& id) const{
            return boost::target(id,_g);
        }


        /* selectors and properties */
        inline const GraphContainer& get_container() const
        {
            return _g;
        }

        inline GraphContainer& get_container() 
        {
            return _g;
        }

        inline NodeRange get_all_nodes() const
        {
            return boost::vertices(_g);
        }
        //! out node if directed
        inline AdjRange get_adj_nodes(const NodeId& id) const
        {
            return boost::adjacent_vertices(id, _g);
        }

        inline int num_nodes() const
        {
            return boost::num_vertices(_g);
        }
        inline int num_edges( ) const{
            return boost::num_edges(_g);
        }

        inline int get_degree(const NodeId& id) const
        {
            return boost::degree(id, _g);
        }
        inline int get_in_degree(const NodeId& id) const
        {
            return boost::in_degree(id, _g);
        }

        inline int get_out_degree(const NodeId& id) const
        {
            return boost::out_degree(id, _g);
        }
        //! first == second if no in edges
        inline InEdgeRange get_in_edges(const NodeId& id) const{
            return boost::in_edges(id, _g);
        }
        //! will return all edges of undirected graph
        inline OutEdgeRange get_out_edges(const NodeId& id) const{
            return boost::out_edges(id, _g);
        }

        inline EdgeRange get_all_edges() const{
            return boost::edges(_g);
        }
        void set_container(GraphContainer *g){
            /// @todo modify _g as a pointer, which make the class more easy to use with algorithms likelihood
            /// breadth_first_search
        }

    public:
    protected:
        GraphContainer _g;
};
/// @todo does "set" make sure unique?

/**
 * @brief an wrapper class to operators of boost graph class
 *  ,this class will ensure nodes(the property of vertex) in the graph is unique
 *
 * usage example:
 *   typedef adjacency_list<vecS, vecS, undirectedS, no_property, 
 *                       property<edge_weight_t, int> > boostGraph;
 *   
 *   typedef AutoUniGraph<boostGraph, Node> MyGraph;
 * @tparam the boost graph
 * @tparam Node an property type, must be assignable and comparable
 */
template < typename GraphContainer, typename Node>
class AutoUniGraph :public BaseGraph<GraphContainer>{
    public:
        /* a bunch of graph-specific typedefs */
        
        /// @todo using
//      using Registration<PointSource, PointTarget>::reg_name_;
        typedef typename BaseGraph<GraphContainer>::NodeId NodeId;
        typedef typename BaseGraph<GraphContainer>::EdgeId EdgeId;
        typedef typename BaseGraph<GraphContainer>::NodeIter NodeIter;
        typedef typename BaseGraph<GraphContainer>::EdgeIter EdgeIter;
        typedef typename BaseGraph<GraphContainer>::AdjIter AdjIter;
        typedef typename BaseGraph<GraphContainer>::EdgeWeightsMap EdgeWeightsMap;
        typedef typename BaseGraph<GraphContainer>::OutEdgeIter OutEdgeIter;
        typedef typename BaseGraph<GraphContainer>::InEdgeIter InEdgeIter;
        typedef typename BaseGraph<GraphContainer>::DegreeSize DegreeSize;
        typedef typename BaseGraph<GraphContainer>::SizeType SizeType;
        typedef Node NodeType;

        typedef std::pair<AdjIter, AdjIter> AdjRange;
        typedef std::pair<NodeIter, NodeIter> NodeRange;
        typedef std::pair<OutEdgeIter, OutEdgeIter> OutEdgeRange;
        typedef std::pair<InEdgeIter, InEdgeIter> InEdgeRange;
        typedef std::pair<EdgeIter, EdgeIter> EdgeRange;
        typedef std::pair<NodeId, NodeId> EdgePair;
        AutoUniGraph (){

        }

        AutoUniGraph (int numNode):BaseGraph<GraphContainer>(numNode){

        }
        virtual ~AutoUniGraph (){

        };
        virtual void clear_(){
            _id2nodeIdPairPtr.clear();
            _node2id.clear();
        }
        inline NodeId add_node(const Node& node) {
            auto i = _node2id.find(node);
            // the node is exist, return
            if(i != _node2id.end())
                return i->second;
            // add new vertex to graph
            NodeId id = add_vertex(this->_g);
            // set extral attributes of the vertex
            _id2nodeIdPairPtr.insert(std::make_pair(id, 
                        _node2id.insert(std::make_pair(node, id)).first));
            return id;
        }
        /// @todo solve this problem
        //! would reassign id value from 0 if node type is vecS
        inline void remove_node(const NodeId& id)
        {
            _node2id.erase(_id2nodeIdPairPtr[id]);
            _id2nodeIdPairPtr.erase(id);
            clear_vertex(id, this->_g);
            remove_vertex(id, this->_g);
        }

        /// @todo is there elegant way of moving this function to parent?
        void copy_subgraph(const std::vector<NodeId> &ids,
                           AutoUniGraph<GraphContainer, Node> *subgraph)
        {
            std::map<NodeId, NodeId> id2sId;
            EdgeWeightsMap weights = this->edge_weights();
            EdgeWeightsMap subWeights = subgraph->edge_weights();
            for(auto id : ids){
                auto c = id2sId.find(id);
                NodeId sId;
                if( c == id2sId.end()){
                    // copy current node
                    auto &node = get_node(id);
                    sId = subgraph->add_node(node);
                    id2sId.insert(make_pair(id, sId));
                }else
                    sId = c->second;
                //
                auto edgeRange = get_out_edges(id);
                for( ; edgeRange.first != edgeRange.second;
                       edgeRange.first++){
                    NodeId nId;
                    auto adjId = targetId(*edgeRange.first);
                    auto &adjnode = get_node(adjId);
                    assert(adjId != id);
                    /// @todo make this condition more general
                    if( !adjnode.is_leaf())
                        continue;
                    auto nb = id2sId.find(adjId); 
                    if( nb == id2sId.end()){
                        // copy  neighbor node
                        nId = subgraph->add_node(adjnode);
                        id2sId.insert(make_pair(adjId, nId));
                    }else
                        nId = nb->second;
                    /// @todo make sure unique edge
                    // copy edge
                    auto rst = subgraph->add_edge(sId, nId);
                    if(rst.second)
                        subWeights[rst.first] = weights[*edgeRange.first];
                }
            }

        }
        inline void modify_node(const NodeId& id, const Node& node){
            auto i = _id2nodeIdPairPtr.find(id);
            assert(i != _id2nodeIdPairPtr.end());
            // remove old value
            _node2id.erase(i->second);
            // add new
            _id2nodeIdPairPtr[id]=_node2id.insert(std::make_pair(node, id)).first;
        }
        inline const Node& get_node(const NodeId& id)
        {
            // return a key of a  map
            return _id2nodeIdPairPtr[id]->first;
        }
        /// @todo this function doesn't compile !
        inline  typename std::map<Node, NodeId>::key_type
        get_node(const NodeId& id) const
        {
            return  _id2nodeIdPairPtr[id]->first;
        }

        inline bool contain_node(const Node& node){
            if(_node2id.find(node) != _node2id->end())
                return false;
            return true;
        }

    private:
        /// @todo replace with boost::bimap
        //! used in put function
        typename std::map<NodeId, typename std::map<Node, NodeId>::iterator>  
            _id2nodeIdPairPtr;
        //! map extral node attributes to id
        typename std::map<Node, NodeId> _node2id;
};
// 
/**
 * @brief an wrapper class to operators of boost graph class,
 *  users required to make sure that nodes(the property of vertex) added to the graph is unique, 
 *  because this class won't check the unique of vertex, it's a bit faster
 *
 * usage example:
 *   typedef adjacency_list<vecS, vecS, undirectedS, property<vertex_node_t, Node>,
 *                       property<edge_weight_t, int> > boostGraph;
 *   
 *   typedef ManualUniGraph<boostGraph, Node> MyGraph;
 * @tparam the boost graph
 * @tparam Node an property type, must be assignable and comparable
 */
template < typename GraphContainer, typename Node>
class ManualUniGraph :public BaseGraph<GraphContainer>{
    public:
        /* a bunch of graph-specific typedefs */

        typedef typename BaseGraph<GraphContainer>::NodeId NodeId;
        typedef typename BaseGraph<GraphContainer>::EdgeId EdgeId;
        typedef typename BaseGraph<GraphContainer>::NodeIter NodeIter;
        typedef typename BaseGraph<GraphContainer>::EdgeIter EdgeIter;
        typedef typename BaseGraph<GraphContainer>::AdjIter AdjIter;
        typedef typename BaseGraph<GraphContainer>::OutEdgeIter OutEdgeIter;
        typedef typename BaseGraph<GraphContainer>::InEdgeIter InEdgeIter;
        typedef typename BaseGraph<GraphContainer>::DegreeSize DegreeSize;
        typedef typename BaseGraph<GraphContainer>::SizeType SizeType;
        typedef typename property_map<GraphContainer, vertex_node_t>::type NodesMap;
        typedef typename BaseGraph<GraphContainer>::EdgeWeightsMap EdgeWeightsMap;
        typedef Node NodeType;

        typedef std::pair<AdjIter, AdjIter> AdjRange;
        typedef std::pair<NodeIter, NodeIter> NodeRange;
        typedef std::pair<OutEdgeIter, OutEdgeIter> OutEdgeRange;
        typedef std::pair<InEdgeIter, InEdgeIter> InEdgeRange;
        typedef std::pair<EdgeIter, EdgeIter> EdgeRange;
        typedef std::pair<NodeId, NodeId> EdgePair;

        ManualUniGraph (){ 
            nodes = get(vertex_node, this->_g);
        }
        ManualUniGraph (int numNode):BaseGraph<GraphContainer>(numNode){ 
            nodes = get(vertex_node, this->_g);
        }
        virtual ~ManualUniGraph (){ };

        inline NodeId add_node(const Node& node) {
            // add new vertex to graph
            NodeId id = add_vertex(this->_g);
            // set extral attributes of the vertex
            nodes[id] = node;
            return id;
        }
        inline void remove_node(const NodeId& id)
        {
            clear_vertex(id, this->_g);
            remove_vertex(id, this->_g);
        }

        inline void modify_node(const NodeId& id, const Node& node){
            nodes[id] = node;
        }

        inline const Node& get_node(const NodeId& id)const
        {
            return nodes[id];
        }
    private:
        NodesMap nodes;

};

    typedef std::vector< std::vector<int> > MatrixGraph;

    //! read_graph
    void read_graph(std::string filename, MatrixGraph &graph);
    //! print graph
    void print_graph(MatrixGraph &graph);
} /* m_graph */

#endif /* end of include guard: GRAPH_H */
