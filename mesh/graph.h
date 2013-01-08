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
#include <map>
#include <boost/foreach.hpp>
#include <cassert>

namespace boost
{

    // Suggested work-around for https://svn.boost.org/trac/boost/ticket/6131
    namespace BOOST_FOREACH =  foreach;
    namespace BOOST_REVERSE_FOREACH = reverse_each
}
#define foreach   BOOST_FOREACH
#define reverse_each   BOOST_REVERSE_FOREACH
//enum vertex_properties_t { vertex_properties };
//namespace boost {
//    BOOST_INSTALL_PROPERTY(vertex, properties);
//}
/* default property tags
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
   vertex_finish_time_t */

//    typedef property<vertex_index_t, double, 
//            property<vertex_properties_t, std::map<VertexProperty, size_t>::iterator> > NodeAttr;
/* the graph base class template */
using namespace boost;
template < typename GraphContainer>
class BaseGraph
{
    public:

        /* a bunch of graph-specific typedefs */
        typedef typename graph_traits<GraphContainer>::vertex_descriptor NodeId;
        typedef typename graph_traits<GraphContainer>::edge_descriptor EdgeId;
        typedef typename property_map<GraphContainer, edge_weight_t>::type EdgeWeightsMap;
        typedef typename property_map<GraphContainer, edge_weight_t>::const_type ConstEdgeWeightsMap;
        typedef std::pair<NodeId, NodeId> EdgePair;
        typedef typename graph_traits<GraphContainer>::vertex_iterator NodeIter;
        typedef typename graph_traits<GraphContainer>::edge_iterator EdgeIter;
        typedef typename graph_traits<GraphContainer>::adjacency_iterator AdjIter;
        typedef typename graph_traits<GraphContainer>::out_edge_iterator OutEdgeIter;
        typedef typename graph_traits<GraphContainer>::in_edge_iterator InEdgeIter;
        typedef typename graph_traits<GraphContainer>::degree_size_type DegreeSize;
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
        NodeId add_node() {
            return add_vertex(this->_g);
        }
        void remove_node(const NodeId& id)
        {
            clear_vertex(id, this->_g);
            remove_vertex(id, this->_g);
        }

        /* structure modification methods */
        void clear()
        {
            clear_();
            _g.clear();
        }
        virtual void clear_(){ }

        const EdgeWeightsMap& edge_weights() const{
            return get(edge_weight, _g);
        }

        //! return a reference to weight attribute of edges
        EdgeWeightsMap edge_weights() {
            return get(edge_weight, _g);
        }

        std::pair<EdgeId, bool>
            add_edge(const NodeId& source, const NodeId& target)
            {
                // assert #source and #target is in the graph
                return boost::add_edge(source, target, _g);
            }

        EdgePair get_edge(const EdgeId& id) const {
            return std::make_pair(boost::source(id, _g), boost::target(id, _g));

        }
        NodeId source(const EdgeId& id){
            return boost::source(id,_g);
        }

        NodeId source(const EdgeId& id) const{
            return boost::source(id,_g);
        }

        NodeId target(const EdgeId& id){
            return boost::target(id,_g);
        }

        NodeId target(const EdgeId& id) const{
            return boost::target(id,_g);
        }


        /* selectors and properties */
        const GraphContainer& get_container() const
        {
            return _g;
        }

        NodeRange get_all_nodes() const
        {
            return vertices(_g);
        }

        AdjRange get_adj_nodes(const NodeId& id) const
        {
            return adjacent_vertices(id, _g);
        }

        int num_nodes() const
        {
            return boost::num_vertices(_g);
        }
        int num_edges( ) const{
            return boost::num_edges(_g);
        }

        int get_degree(const NodeId& id) const
        {
            return degree(id, _g);
        }
        int get_in_degree(const NodeId& id) const
        {
            return in_degree(id, _g);
        }

        int get_out_degree(const NodeId& id) const
        {
            return out_degree(id, _g);
        }

        InEdgeRange get_in_edges(const NodeId& id) const{
            return in_edges(id, _g);
        }

        OutEdgeRange get_out_edges(const NodeId& id) const{
            return out_edges(id, _g);
        }

        EdgeRange get_all_edges() const{
            return edges(_g);
        }
        void initial_dMatrix(){
            _dMatrix = new int*[num_nodes()];
            for (int i = 0; i < num_nodes(); i++) {
                _dMatrix[i] = new int[num_nodes()];
            }
        }
        void destroy_dMatrix(){
            for (int i = 0; i < num_nodes(); i++) {
                delete _dMatrix[i];
            }
            delete _dMatrix;
        }

    public:
        // won't be copy by constructor and assign operator! 
        int** _dMatrix;                        //!< distance matrix 
    protected:
            GraphContainer _g;
};

/**
 * @brief 
 *
 * @tparam GraphContainer
 * @tparam Node an property type, must be assignable and comparable
 */
template < typename GraphContainer, typename Node>
class AutoUniGraph :public BaseGraph<GraphContainer>{
    public:
        /* a bunch of graph-specific typedefs */
        typedef typename BaseGraph<GraphContainer>::NodeId NodeId;
        typedef typename BaseGraph<GraphContainer>::EdgeId EdgeId;
        typedef typename BaseGraph<GraphContainer>::NodeIter NodeIter;
        typedef typename BaseGraph<GraphContainer>::EdgeIter EdgeIter;
        typedef typename BaseGraph<GraphContainer>::AdjIter AdjIter;
        typedef typename BaseGraph<GraphContainer>::EdgeWeightsMap EdgeWeightsMap;
        typedef typename BaseGraph<GraphContainer>::OutEdgeIter OutEdgeIter;
        typedef typename BaseGraph<GraphContainer>::InEdgeIter InEdgeIter;
        typedef typename BaseGraph<GraphContainer>::DegreeSize DegreeSize;

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
        NodeId add_node(const Node& node) {
            auto i = _node2id.find(node);
            // the exist is exist, return
            if(i != _node2id.end())
                return i->second;
            // add new vertex to graph
            NodeId id = add_vertex(this->_g);
            // set extral attributes of the vertex
            _id2nodeIdPairPtr.insert(std::make_pair(id, 
                        _node2id.insert(std::make_pair(node, id)).first));
            return id;
        }
        void remove_node(const NodeId& id)
        {
            _node2id.erase(_id2nodeIdPairPtr[id]);
            _id2nodeIdPairPtr.erase(id);
            clear_vertex(id, this->_g);
            remove_vertex(id, this->_g);
        }


        void modify_node(const NodeId& id, const Node& node){
            auto i = _id2nodeIdPairPtr.find(id);
            assert(i != _id2nodeIdPairPtr.end());
            // remove old value
            _node2id.erase(i->second);
            // add new
            i->second = _node2id.insert(std::make_pair(node, id));
        }
        const Node& get_node(const NodeId& id)
        {
            return _id2nodeIdPairPtr[id]->first;
        }

        const Node& get_node(const NodeId& id) const
        {
            return _id2nodeIdPairPtr[id]->first;
        }

        bool contain_node(const Node& node){
            if(_node2id.find(node) != _node2id->end())
                return false;
            return true;
        }

    private:
        //! used in put function
        typename std::map<NodeId, typename std::map<Node, NodeId>::iterator>  
            _id2nodeIdPairPtr;
        //! map extral node attributes to id
        typename std::map<Node, NodeId> _node2id;
};

/**
 * @brief an boost based graph class, users required to make sure that the 
 *  vertexes added to the graph is unique, because this class won't check the
 *  unique of vertex, it's a bit faster than class #AutoUniGraph  
 *
 * @tparam GraphContainer
 * @tparam Node
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
        typedef typename BaseGraph<GraphContainer>::EdgeWeightsMap EdgeWeightsMap;
        typedef typename BaseGraph<GraphContainer>::OutEdgeIter OutEdgeIter;
        typedef typename BaseGraph<GraphContainer>::InEdgeIter InEdgeIter;
        typedef typename BaseGraph<GraphContainer>::DegreeSize DegreeSize;

        typedef std::pair<AdjIter, AdjIter> AdjRange;
        typedef std::pair<NodeIter, NodeIter> NodeRange;
        typedef std::pair<OutEdgeIter, OutEdgeIter> OutEdgeRange;
        typedef std::pair<InEdgeIter, InEdgeIter> InEdgeRange;
        typedef std::pair<EdgeIter, EdgeIter> EdgeRange;
        typedef std::pair<NodeId, NodeId> EdgePair;

        ManualUniGraph (){

        }

        ManualUniGraph (int numNode):BaseGraph<GraphContainer>(numNode){

        }
        virtual ~ManualUniGraph (){

        };
        NodeId add_node(const Node& node) {
            // add new vertex to graph
            NodeId id = add_vertex(this->_g);
            // set extral attributes of the vertex
            _id2node[id] = node;
            return id;
        }
        void remove_node(const NodeId& id)
        {
            _id2node.erase(id);
            clear_vertex(id, this->_g);
            remove_vertex(id, this->_g);
        }

        const Node& get_node(const NodeId& id)
        {
            return _id2node[id];
        }

        void modify_node(const NodeId& id, const Node& node){
            get_node(id) = node;
        }

        const Node& get_node(const NodeId& id)const
        {
            return _id2node[id];
        }

        virtual void clear_(){
            _id2node.clear();
        }
    private:
        //! map extral node attributes to id
        typename std::map<NodeId, Node> _id2node;
};


#endif /* end of include guard: GRAPH_H */
