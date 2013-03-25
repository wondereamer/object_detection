#include "graph.h"
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
