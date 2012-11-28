/**
 * @file vertex_color.cpp
 * @brief vertex coloring algorithm
 * @author Dingjie.Wang(dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-11-28
 */

#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <map>
using namespace std;

bool removable(vector<int> neighbor, vector<int> cover);
int max_removable(vector<vector<int> > neighbors, vector<int> cover);
vector<int> procedure_1(vector<vector<int> > neighbors, vector<int> cover);
vector<int> procedure_2(vector<vector<int> > neighbors, vector<int> cover, int k);
int cover_size(vector<int> cover);
namespace m_graph {
    
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
bool vertex_coloring(const vector< vector<int> > &Graph, map<int, string> &colorMap, map<int, string> &rst){
    int colorNum = colorMap.size();
    // number of vertex
    int N = Graph.size();
    // #info
    std::cout<<" coloring vertex... "<<std::endl
             <<"with "<< colorNum<<" color"<<std::endl
             <<"width "<< N<<" nodes"<<std::endl;
    std::map<int, int> vtx2colorIndex;
    int n, i, j, k, K, p, q, r, s, min, edge, counter=0;
    //Complete garph on colorNum verteices
    vector<vector<int> > KC;
    vector<int> row1;
    for(int i=0; i<colorNum; i++) row1.push_back(1);
    for(int i=0; i<colorNum; i++) KC.push_back(row1);
    for(int i=0; i<colorNum; i++) KC[i][i]=0;
    //Cartesian product of Graph and KC
    vector<vector<int> > graph;
    vector<int> rowind;
    for(int i=0; i<colorNum*N; i++) rowind.push_back(0);
    for(int i=0; i<colorNum*N; i++) graph.push_back(rowind);
    for(int i=0; i<colorNum*N; i++)
        for(int j=0; j<colorNum*N; j++)
        {
            int i_G=i/colorNum, i_KC=i%colorNum, j_G=j/colorNum, j_KC=j%colorNum;
            if((i_G==j_G) && (KC[i_KC][j_KC]==1)) graph[i][j]=1;
            if((Graph[i_G][j_G]==1) && (i_KC==j_KC)) graph[i][j]=1;
        }
    //Assign parameters for finding independent sets in the graph
    n=N*colorNum; K=n/colorNum; k=n-K;
    //Find Neighbors
    vector<vector<int> > neighbors;
    for(i=0; i<graph.size(); i++)
    {
        vector<int> neighbor;
        for(j=0; j<graph[i].size(); j++)
            if(graph[i][j]==1) neighbor.push_back(j);
        neighbors.push_back(neighbor);
    }

    //Find Independent Sets
    bool found=false;
    min=n+1;
    vector<vector<int> > covers;
    vector<int> allcover;
    for(i=0; i<graph.size(); i++)
        allcover.push_back(1);
    for(i=0; i<allcover.size(); i++)
    {
        if(found) break;
        counter++; 
        vector<int> cover=allcover;
        cover[i]=0;
        cover=procedure_1(neighbors,cover);
        s=cover_size(cover);
        if(s<min) min=s;
        if(s<=k)
        {

            if (n-s == N) {
                for(j=0; j<cover.size(); j++) 
                    if(cover[j]==0)
                        vtx2colorIndex[j/colorNum] = j%colorNum;
            }
            covers.push_back(cover);
            found=true;
            break;
        }
        for(j=0; j<n-k; j++)
            cover=procedure_2(neighbors,cover,j);
        s=cover_size(cover);
        if(s<min) min=s;

        if (n-s == N) {
            for(j=0; j<cover.size(); j++) 
                if(cover[j]==0)
                    vtx2colorIndex[j/colorNum] = j%colorNum;
        }
        covers.push_back(cover);
        if(s<=k){ found=true; break; }
    }
    //Pairwise Intersections
    for(p=0; p<covers.size(); p++)
    {
        if(found) break;
        for(q=p+1; q<covers.size(); q++)
        {
            if(found) break;
            counter++; 
            vector<int> cover=allcover;
            for(r=0; r<cover.size(); r++)
                if(covers[p][r]==0 && covers[q][r]==0) cover[r]=0;
            cover=procedure_1(neighbors,cover);
            s=cover_size(cover);
            if(s<min) min=s;
            if(s<=k)
            {

                if (n-s == N) {
                    for(j=0; j<cover.size(); j++) 
                        if(cover[j]==0)
                            vtx2colorIndex[j/colorNum] = j%colorNum;
                }
                found=true;
                break;
            }
            for(j=0; j<k; j++)
                cover=procedure_2(neighbors,cover,j);
            s=cover_size(cover);
            if(s<min) min=s;
            if (n-s == N) {
                for(j=0; j<cover.size(); j++) 
                    if(cover[j]==0)
                        vtx2colorIndex[j/colorNum] = j%colorNum;
            }
            if(s<=k){ found=true; break; }
        }
    }
    if(found) {
        for (int i = 0; i < Graph.size(); i++) {
            rst[i] = colorMap[vtx2colorIndex[i]];
        }
        return true;
    }
    return false;
}

} /* m_graph */
bool removable(vector<int> neighbor, vector<int> cover)
{
    bool check=true;
    for(int i=0; i<neighbor.size(); i++)
        if(cover[neighbor[i]]==0)
        {
            check=false;
            break;
        }
    return check;
}

int max_removable(vector<vector<int> > neighbors, vector<int> cover)
{
    int r=-1, max=-1;
    for(int i=0; i<cover.size(); i++)
    {
        if(cover[i]==1 && removable(neighbors[i],cover)==true)
        {
            vector<int> temp_cover=cover;
            temp_cover[i]=0;
            int sum=0;
            for(int j=0; j<temp_cover.size(); j++)
                if(temp_cover[j]==1 && removable(neighbors[j], temp_cover)==true)
                    sum++;
            if(sum>max)
            {
                if(r==-1)
                {
                    max=sum;
                    r=i;
                }
                else if(neighbors[r].size()>=neighbors[i].size())
                {
                    max=sum;
                    r=i;
                }
            }
        }
    }
    return r;
}

vector<int> procedure_1(vector<vector<int> > neighbors, vector<int> cover)
{
    vector<int> temp_cover=cover;
    int r=0;
    while(r!=-1)
    {
        r= max_removable(neighbors,temp_cover);
        if(r!=-1) temp_cover[r]=0;
    }
    return temp_cover;
}

vector<int> procedure_2(vector<vector<int> > neighbors, vector<int> cover, int k)
{
    int count=0;
    vector<int> temp_cover=cover;
    int i=0;
    for(int i=0; i<temp_cover.size(); i++)
    {
        if(temp_cover[i]==1)
        {
            int sum=0, index;
            for(int j=0; j<neighbors[i].size(); j++)
                if(temp_cover[neighbors[i][j]]==0) {index=j; sum++;}
            if(sum==1 && cover[neighbors[i][index]]==0)
            {
                temp_cover[neighbors[i][index]]=1;
                temp_cover[i]=0;
                temp_cover=procedure_1(neighbors,temp_cover);
                count++;
            }
            if(count>k) break;
        }
    }
    return temp_cover;
}

int cover_size(vector<int> cover)
{
    int count=0;
    for(int i=0; i<cover.size(); i++)
        if(cover[i]==1) count++;
    return count;
}

