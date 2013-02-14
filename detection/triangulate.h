/****************************************************************************
 * EfPiSoft                                                                  *
 *                                                                           *
 * Consiglio Nazionale delle Ricerche                                        *
 * Istituto di Matematica Applicata e Tecnologie Informatiche                *
 * Sezione di Genova                                                         *
 * IMATI-GE / CNR                                                            *
 *                                                                           *
 * Authors: Marco Attene                                                     *
 *                                                                           *
 * Copyright(C) 2006: IMATI-GE / CNR                                         *
 *                                                                           *
 * All rights reserved.                                                      *
 *                                                                           *
 * This program is free software; you can redistribute it and/or modify      *
 * it under the terms of the GNU General Public License as published by      *
 * the Free Software Foundation; either version 2 of the License, or         *
 * (at your option) any later version.                                       *
 *                                                                           *
 * This program is distributed in the hope that it will be useful,           *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 * GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
 * for more details.                                                         *
 *                                                                           *
 ****************************************************************************/
#ifndef TRIANGULATE_H

#define TRIANGULATE_H



//#include <Inventor/Qt/SoQt.h>
//#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
//#include <Inventor/nodes/SoVertexProperty.h>
//#include <Inventor/nodes/SoSeparator.h>
//#include <Inventor/nodes/SoCoordinate3.h>
//#include <Inventor/nodes/SoIndexedFaceSet.h>
//#include <Inventor/actions/SoWriteAction.h>
#include "jmesh.h"
#include "shape_segment.h"
#include <iostream>
#include "graph.h" 
#include <boost/graph/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <pcl/surface/gp3.h>

class MyTriangulation :public Triangulation{
    public:
        MyTriangulation (){ };
        virtual ~MyTriangulation (){ };
        int load_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PolygonMesh &meshes){
            /// @todo remove small component
            FILE *fp;
            Node *n;
            int num_mesh = meshes.polygons.size();
            int num_vertex = num_mesh * 3;
            int i,j,i1,i2,i3;
            Vertex *v;
            ExtVertex **var = (ExtVertex **)malloc(sizeof(ExtVertex *)*num_vertex);
            if ((fp = fopen("temp" ,"w")) == NULL) return IO_CANTOPEN;
            // create vertex array
            for(auto &p : cloud->points){
                // the real vertex, would be freed by parent
                V.appendTail(new Vertex(p.x, p.y, p.z));
            }
            //
            i=0; 
            FOREACHVERTEX(v, n) 
                var[i++] = new ExtVertex(v);
            JMesh::begin_progress();
            for(const pcl::Vertices &mesh : meshes.polygons){
                int i1 = mesh.vertices[0];
                int i2 = mesh.vertices[1];
                int i3 = mesh.vertices[2];
                if (i1 == i2 || i2 == i3 || i3 == i1) 
                    JMesh::warning("\nloadOFF: Coincident indexes at triangle %d! Skipping.\n",i);
                else if (!CreateIndexedTriangle(var, i1, i2, i3))
                    JMesh::warning("\nloadOFF: This shouldn't happen!!! Skipping triangle.\n");
            };
            JMesh::end_progress();
            // close #fp and free #var
            closeLoadingSession(fp, i, var, false);
            if(shells() > 1){
                removeSmallestComponents();
                std::cout<<"**small components are removed!**"<<std::endl;
            }
            // may added some new vertex
            std::cout<<V.numels()<<" *"<<cloud->points.size()<<std::endl;
            saveOFF("temp.off");
            return 0;
        }

        void reset(){
            ClusterNode::reset();
        }




};


#endif /* end of include guard: TRIANGULATE_H */
