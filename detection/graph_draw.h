#ifndef GRAPH_DRAW_H

#define GRAPH_DRAW_H
#include <string>
#include <fstream>
namespace m_graph {
template < typename Graph >
class DottyOutput {
public:
    DottyOutput (Graph *g):_fout(NULL), _g(g){ };
    virtual ~DottyOutput (){ };
    void write(const std::string &fname, bool labelEdge = false, 
                bool directed = false)
    {
        assert(_g);
        set_filename(fname);
        begin_drawing(labelEdge, directed);
        typename Graph::EdgeIter ei, ei_end;
        typename Graph::NodeIter ni, ni_end; 
        // draw nodes
        for (tie(ni, ni_end) = _g->get_all_nodes(); ni != ni_end; ni++){
            do_draw_node(*ni);
        }
        // draw edges
        for (tie(ei, ei_end) = _g->get_all_edges(); ei != ei_end; ei++)
            do_draw_edge(*ei);
        end_drawing();
    }
    void set_filename(const std::string &fname){
        // error when delete
        if(_fout)
            delete _fout;
        else
            _fout = new std::ofstream(fname);

    }
    virtual void begin_drawing(bool labelEdge = false, 
                                bool directed = false)
    {
        assert(_fout);
        _directed = directed;
        _labelEdge = labelEdge;
        if(_directed)
            *_fout << "digraph A {\n";
        else
            *_fout << "graph A {\n";
        // horizonal graph
        *_fout  << "  rankdir=LR\n"
                << "ratio=\"fill\"\n"
                << "edge[style=\"bold\"]\n" << "node[shape=\"circle\"]\n";
    }
    virtual void do_draw_node(typename Graph::NodeId id){
        //10[fillcolor="red"][shape="rect"][style="filled"][color="green"][weight=5][height=10]
        assert(_fout);
        *_fout<<id<<std::endl;
    }
    virtual void do_draw_edge(typename Graph::EdgeId eid){
        assert(_fout);
        static typename Graph::EdgeWeightsMap edgeWeights = _g->edge_weights();
        if (_directed) 
            *_fout <<_g->sourceId(eid) << " -> " << _g->targetId(eid);
        else
            *_fout <<_g->sourceId(eid) << " -- " << _g->targetId(eid);
        if (_labelEdge)
            *_fout<< "[label=" << edgeWeights[eid] << "]\n";
        else
            *_fout<<"\n";
    }
    virtual void end_drawing(){
       assert(_fout);
       * _fout << "}\n";
       delete _fout;
       _fout = NULL;
    }
protected:
    std::ofstream *_fout;
    Graph *_g;
    bool _directed;
    bool _labelEdge;
    /* data */
};
}; // end of m_graph namespace
#endif /* end of include guard: GRAPH_DRAW_H */
