#include "m_graph.h"
using namespace m_graph;
void test_VizGraph(){
    //struct ComponentH
    struct Node{
        int id;
        std::string name;
    };
    struct Edge{
        int weight;
        std::string name;
    };

    VizGraph<Node, Edge> graph;
    Node node1;
    node1.id = 1;
    node1.name = "hello world!";
    Node node2;
    node2.id = 2;
    Edge edge1;
    edge1.name = "I'm edge!";
    VizGraph<Node, Edge>::NodeH n1 = graph.add_node(node1);
    VizGraph<Node, Edge>::NodeH n2 = graph.add_node(node2);
    VizGraph<Node, Edge>::EdgeH e1 = graph.add_edge(n1, n2, edge1);
    Node node = graph.get_node_attrs(n1);
    Edge edge = graph.get_edge_attrs(e1);
    std::cout<<node.id<<std::endl;
    std::cout<<node.name<<std::endl;
    std::cout<<edge.name<<std::endl;
}
