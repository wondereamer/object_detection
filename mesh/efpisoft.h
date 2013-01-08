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

#ifndef _EFPISOFT_H
#define _EFPISOFT_H

#ifdef WIN32
#define COIN_NOT_DLL
#define SOQT_NOT_DLL
#endif

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <QMainWindow>
#include <list.h>

#define HFP_PAPER_CITATION "M. Attene, B. Falcidieno and M. Spagnuolo.\nHierarchical Mesh Segmentation Based on Fitting Primitives.\nThe Visual Computer, 22(3), pp. 181-193, 2006."

class Triangulation;
class QSlider;
class QMenu;
class QAction;

class MainCanvas : public QMainWindow
{
    Q_OBJECT
public:
    MainCanvas( QWidget *parent=0);

    void setMesh(Triangulation *);
    void setHFP(List *, int =0);
    static unsigned int random_color();
    void openFile(QString);

public slots:
    void open();
    void save_model();
    void save_scene();
    void closeMesh();
    void properties();

    void runhfp();
    void shuffleColors();
    void increaseNumClusters();
    void decreaseNumClusters();
    void showAllClusters();
    void showOneCluster();

    void about();

    void setSceneGraph();
    void checkNoTIN();
    void checkHFPStatus();

    void updateMatIndexes();
    void setNumClusters(int =0);

signals:
    void tinChanged();
    void hfpChanged();
    void numclustersChanged(int);

private:
    SoQtExaminerViewer *viewer;
    // mesh
    Triangulation *tin;
    List *hfpCollapses;
    int numclusters;

    QAction *openAct;
    QAction *save_modelAct;
    QAction *save_sceneAct;
    QAction *closeMeshAct;
    QAction *propertiesAct;

    QAction *fit_planesAct;
    QAction *fit_spheresAct;
    QAction *fit_cylindersAct;
    QAction *runhfpAct;
    QAction *shuffleColorsAct;
    QAction *increaseNumClustersAct;
    QAction *decreaseNumClustersAct;
    QAction *showAllClustersAct;
    QAction *showOneClusterAct;

    QAction *aboutAct;

    QSlider *cslider;
};

#endif // _EFPISOFT_H

