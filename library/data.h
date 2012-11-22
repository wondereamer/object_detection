/**
 * @file data.h
 * @brief 
 * @author Dingjie.Wang(dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-11-22
 */
#ifndef DATA_H
#define DATA_H

#include <xmlrpc-c/base.hpp>
#include <vector>

using std::vector;
typedef std::vector<xmlrpc_c::value> AttrVector;

void _get_block_attrs(AttrVector &posV, AttrVector &sizeV, AttrVector& materialV, AttrVector &colorV, std::string method, int port = 8000);
void get_block_attrs(vector<float> &xList, vector<float> &yList, vector<float> &zList,
                    vector<int> &rList, vector<int> &gList, vector<int> &bList, vector<std::string> &materialList,
                    vector<int> &sizeList, std::string method, int port = 8000);
#endif /* end of include guard: DATA_H */
