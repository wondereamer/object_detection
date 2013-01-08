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

#include "efpisoft.h"

#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoVertexProperty.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/actions/SoWriteAction.h>
#include <QtGui>
#include <QApplication>
#include "jmesh.h"
#include "fittingPrimitives.h"





MainCanvas::MainCanvas(QWidget *parent) : QMainWindow(parent)
{
 QMenu *fileMenu = menuBar()->addMenu("&File");
  openAct       = fileMenu->addAction("&Open ...",       this, SLOT(open()),       QString("CTRL+O"));
  save_modelAct = fileMenu->addAction("&Save Model ...", this, SLOT(save_model()), QString("CTRL+S"));
  save_sceneAct = fileMenu->addAction("Save Scene ...",  this, SLOT(save_scene())                   );
  closeMeshAct  = fileMenu->addAction("Close",           this, SLOT(closeMesh())                    );
  propertiesAct = fileMenu->addAction("Properties ...",  this, SLOT(properties())                   );
  fileMenu->addSeparator();
  fileMenu->addAction("E&xit",  qApp, SLOT(quit()), QString("CTRL+Q"));

 QMenu *hfpMenu = menuBar()->addMenu("HF&P");
  fit_planesAct    = hfpMenu->addAction( "Fit Planes" );
  fit_spheresAct   = hfpMenu->addAction( "Fit Spheres");
  fit_cylindersAct = hfpMenu->addAction( "Fit Cylinders");
  runhfpAct        = hfpMenu->addAction( "&Run HFP",         this, SLOT(runhfp()), QString("CTRL+R"));
  shuffleColorsAct = hfpMenu->addAction( "Shuffle Colors",   this, SLOT(shuffleColors()));
  increaseNumClustersAct = hfpMenu->addAction( "Clusters++", this, SLOT(increaseNumClusters()), QString("CTRL+Up"));
  decreaseNumClustersAct = hfpMenu->addAction( "Clusters--", this, SLOT(decreaseNumClusters()), QString("CTRL+Down"));
  showAllClustersAct = hfpMenu->addAction( "All Clusters",   this, SLOT(showAllClusters()), QString("CTRL+End"));
  showOneClusterAct  = hfpMenu->addAction( "One Cluster",    this, SLOT(showOneCluster()), QString("CTRL+Home"));

  fit_planesAct->setCheckable(1);    fit_planesAct->setChecked(1);
  fit_spheresAct->setCheckable(1);   fit_spheresAct->setChecked(1);
  fit_cylindersAct->setCheckable(1); fit_cylindersAct->setChecked(1);

 QMenu *helpMenu = menuBar()->addMenu("&Help");
  aboutAct = helpMenu->addAction( "&About", this, SLOT(about()), QString("CTRL+H"));

 statusBar();

 viewer = new SoQtExaminerViewer(this);
 viewer->setTitle("Hierarchical Fitting Primitives");
 viewer->setDecoration(0);
 setCentralWidget(viewer->getWidget());

 setMinimumSize( 600, 400 );

 tin = NULL;
 hfpCollapses = NULL;
 numclusters = 0;
 cslider = NULL;

 connect (this, SIGNAL(tinChanged()),            this, SLOT(setSceneGraph()));
 connect (this, SIGNAL(tinChanged()),            this, SLOT(checkNoTIN()));
 connect (this, SIGNAL(hfpChanged()),            this, SLOT(checkHFPStatus()));
 connect (this, SIGNAL(numclustersChanged(int)), this, SLOT(updateMatIndexes()));
 connect (this, SIGNAL(numclustersChanged(int)), this, SLOT(checkHFPStatus()));

 checkNoTIN();
}


void MainCanvas::checkNoTIN()
{
 save_modelAct->setEnabled((tin != NULL));
 propertiesAct->setEnabled((tin != NULL));
 closeMeshAct->setEnabled((tin != NULL));
 runhfpAct->setEnabled((tin != NULL));
 checkHFPStatus();
}

void MainCanvas::checkHFPStatus()
{
 save_sceneAct->setEnabled((hfpCollapses != NULL));
 shuffleColorsAct->setEnabled((hfpCollapses != NULL));
 increaseNumClustersAct->setEnabled((hfpCollapses != NULL && numclusters < tin->T.numels()));
 decreaseNumClustersAct->setEnabled((hfpCollapses != NULL && numclusters > 1));
 showAllClustersAct->setEnabled((hfpCollapses != NULL && numclusters < tin->T.numels()));
 showOneClusterAct->setEnabled((hfpCollapses != NULL && numclusters > 1));
 
 if (hfpCollapses != NULL)
 {
  if (cslider == NULL)
  {
   cslider = new QSlider(Qt::Horizontal, statusBar());
   cslider->setRange(1, tin->T.numels());
   cslider->setValue(numclusters);
   connect(cslider, SIGNAL(valueChanged(int)), this, SLOT(setNumClusters(int)));
   connect(this, SIGNAL(numclustersChanged(int)), cslider, SLOT(setValue(int)));
   statusBar()->addPermanentWidget(cslider, 0);
  }
  char msg[1024];
  sprintf(msg, "Num. Clusters: %d", numclusters);
  statusBar()->showMessage(msg);
 }
 else
 {
  statusBar()->clearMessage();
  if (cslider != NULL)
  {
   statusBar()->removeWidget(cslider);
   delete cslider;
   cslider = NULL;
  }
 }
}

void MainCanvas::runhfp()
{
 unsigned char wtf = 0;
 if (fit_planesAct->isChecked()) wtf |= HFP_FIT_PLANES;
 if (fit_spheresAct->isChecked()) wtf |= HFP_FIT_SPHERES;
 if (fit_cylindersAct->isChecked()) wtf |= HFP_FIT_CYLINDERS;
 QProgressBar pb;
 pb.setGeometry(x()+width()/3, y()+height()/3, width()/3, 40);
 pb.show();
 QApplication::setOverrideCursor(QCursor(Qt::BusyCursor));
 HFP_Action hfp(tin, &pb);
 List *coll = hfp.fit(wtf);
 setHFP(coll, 1);
 Triangle *f;
 Node *n;
 int i=0; FOREACHVTTRIANGLE((&(tin->T)), f, n) f->info = (void *)i++;
 setSceneGraph();
 updateMatIndexes();
 QApplication::restoreOverrideCursor();
}

void MainCanvas::open()
{
 openFile(QFileDialog::getOpenFileName(this,
                    QString("Open a new Mesh Model"),
                    QString(),
                    QString("Meshes (*.wrl *.off *.iv *.ply *.obj *.stl *.tri *.swm)")));
}

void MainCanvas::openFile(QString s)
{
 if (s != NULL)
 {
  Triangulation *nt = new Triangulation;
  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  int ioerr = nt->load(s.toStdString().c_str());
  QApplication::restoreOverrideCursor();

  if (ioerr == IO_CANTOPEN) JMesh::warning("Couldn't open '%s'\n",s.toStdString().c_str());
  else if (ioerr == IO_UNKNOWN) JMesh::warning("'%s': Unknown file format.\n",s.toStdString().c_str());
  else if (nt->shells() == 0)
  {
   JMesh::warning("No triangles loaded!\n");
   delete nt;
  }
  else if (nt->shells() > 1)
  {
   int ans = QMessageBox::warning(0, "Only single component meshes are supported!\n",
	"Do you want me to remove all the smallest components ?", QMessageBox::Yes, QMessageBox::Cancel);
   if (ans == QMessageBox::Yes) {nt->removeSmallestComponents(); setMesh(nt);}
   else setMesh(nt);
  }
  else setMesh(nt);
 }
}

void MainCanvas::save_model()
{
 QString s = QFileDialog::getSaveFileName(this,
                     "Save Current Mesh", QString(),
                    "Meshes (*.wrl *.off *.iv *.ply *.obj *.stl *.tri)"
                    );

 if (s != NULL) tin->save(s.toStdString().c_str(), 0);
}


void MainCanvas::save_scene()
{
 QString s = QFileDialog::getSaveFileName(this,
                    "Save Current Inventor Scene", QString(),
                    "Inventor Scenes (*.iv)");

 if (s != NULL)
 {
  SoWriteAction wa;
  if (!wa.getOutput()->openFile(s.toStdString().c_str()))
   JMesh::warning("Couldn't open '%s'\n", s.toStdString().c_str());
  else
  {
   wa.apply(viewer->getSceneGraph());
   wa.getOutput()->closeFile();
  }
 }
}


void MainCanvas::about()
{
 char msg1[2048];
 sprintf(msg1,"%s\nv%s (%s)\nAuthors: %s\n",
	JMesh::app_name, JMesh::app_version, JMesh::app_year,
	JMesh::app_authors);

 char msg2[2048];
 sprintf(msg2,"\nBased on paper:\n%s\n", HFP_PAPER_CITATION);

 QString abmsg = QString(msg1)+QString(msg2);

 QMessageBox::about(this, JMesh::app_name, abmsg);
}


void MainCanvas::properties()
{
 char msg[4096];

 sprintf(msg, "Vertices: %d\nEdges: %d\nTriangles: %d\nBoundaries: %d\nHandles: %d\nComponents: %d",
         tin->V.numels(), tin->E.numels(), tin->T.numels(), tin->boundaries(), tin->handles(), tin->shells());
 QMessageBox::information(this, "Model Info", msg);
}


void MainCanvas::closeMesh()
{
 setMesh(NULL);
}

void MainCanvas::setMesh(Triangulation *t)
{
 if (tin != NULL) {setHFP(NULL); delete tin;}
 tin = t;
 emit tinChanged();
}

void MainCanvas::setHFP(List *l, int nc)
{
 if (hfpCollapses != NULL)
 {
  for (Node *n=hfpCollapses->head(); n!=NULL; n=n->next()->next()->next()) delete((List *)n->data);
  delete hfpCollapses;
  hfpCollapses=NULL;
  numclusters = 0;
 }
 hfpCollapses = l;
 numclusters = nc;
 emit hfpChanged();
}

void MainCanvas::shuffleColors()
{
 SoIndexedFaceSet *its = (SoIndexedFaceSet *)((SoSeparator *)viewer->getSceneGraph())->getChild(0);
 SoVertexProperty *vp = (SoVertexProperty *)its->vertexProperty.getValue();
 uint32_t *rgbas = vp->orderedRGBA.startEditing();
 int i, numt = vp->orderedRGBA.getNum();
 for (i=0; i<numt; i++) rgbas[i] = random_color();
 vp->orderedRGBA.finishEditing();
}

void MainCanvas::setNumClusters(int n)
{
 if (numclusters != n)
 {
  numclusters = n;
  emit numclustersChanged(n);
 }
}

void MainCanvas::increaseNumClusters() {setNumClusters(numclusters+1);}
void MainCanvas::decreaseNumClusters() {setNumClusters(numclusters-1);}
void MainCanvas::showAllClusters() {setNumClusters(tin->T.numels());}
void MainCanvas::showOneCluster() {setNumClusters(1);}

void MainCanvas::updateMatIndexes()
{
 Node *n;
 static Node *last_node;
 static int prev_numclusters;
 Triangle *t;
 List *tlist, *collapses = hfpCollapses;
 SoIndexedFaceSet *its = (SoIndexedFaceSet *)((SoSeparator *)viewer->getSceneGraph())->getChild(0);
 int i, index, numtris = its->materialIndex.getNum();

 int32_t *idx = its->materialIndex.startEditing();

 if (numclusters == 1)
 {
  last_node = collapses->head();
  index = (int)last_node->next()->data;
  for (i=0; i<numtris; i++) idx[i] = index;
 }
 else
 {
   for (i=prev_numclusters; i<numclusters; i++)
   {
    tlist = (List *)last_node->data;
    index = (int)last_node->next()->next()->data;
    FOREACHVTTRIANGLE(tlist, t, n) idx[(int)t->info] = index;
    last_node = (last_node->next()==NULL)?(NULL):(last_node->next()->next()->next());
   }
   for (i=prev_numclusters-1; i>=numclusters; i--)
   {
    last_node = (last_node)?(last_node->prev()->prev()->prev()):(collapses->tail()->prev()->prev());
    tlist = (List *)last_node->data;
    index = (int)last_node->next()->data;
    FOREACHVTTRIANGLE(tlist, t, n) idx[(int)t->info] = index;
   }
 }

 its->materialIndex.finishEditing();
 prev_numclusters = numclusters;
 viewer->render();
}


unsigned int MainCanvas::random_color()
{
 double r, g, b, l, n;
 r = ((double)rand())/RAND_MAX;
 g = ((double)rand())/RAND_MAX;
 b = ((double)rand())/RAND_MAX;
 l=sqrt(r*r+g*g+b*b);
 n = 1.0/l;
 r *= n; g *= n; b *= n;
 unsigned int ir = (unsigned int)(r*256); if (ir>256) ir=256;
 unsigned int ig = (unsigned int)(g*256); if (ig>256) ig=256;
 unsigned int ib = (unsigned int)(b*256); if (ib>256) ib=256;
 return ((ir<<24) + (ig<<16) + (ib<<8) + 0x000000ff);
}


void MainCanvas::setSceneGraph()
{
 if (tin == NULL) {viewer->setSceneGraph(new SoSeparator()); return;}

 Node *n;
 Vertex *v;
 Triangle *t;
 int i=0;
 coord *ox;

 SoVertexProperty *vp = new SoVertexProperty;
  vp->vertex.setNum(tin->V.numels());
  SbVec3f *crds = vp->vertex.startEditing();
  FOREACHVVVERTEX((&(tin->V)), v, n) crds[i++].setValue(v->x, v->y, v->z);
  vp->vertex.finishEditing();

 if (hfpCollapses != NULL)
 {
  vp->orderedRGBA.setNum(tin->T.numels());
  uint32_t *rgbas = vp->orderedRGBA.startEditing();
  for (i=0; i<tin->T.numels(); i++) rgbas[i] = random_color();
  vp->orderedRGBA.finishEditing();
  vp->materialBinding = SoVertexProperty::PER_FACE_INDEXED;
 }

 if ((ox = (coord *)malloc(sizeof(coord)*tin->V.numels())) == NULL)
  JMesh::error("buildSceneGraph: Not enough memory.\n");

 i=0; FOREACHVVVERTEX((&(tin->V)), v, n) {ox[i] = v->x; v->x = i++;}

 SoIndexedFaceSet *its = new SoIndexedFaceSet;
  its->coordIndex.setNum(tin->T.numels()*4);
  its->vertexProperty = vp;
  if (hfpCollapses != NULL) its->materialIndex.setNum(tin->T.numels());
  int32_t *idx = its->coordIndex.startEditing();

  i=0; FOREACHVTTRIANGLE((&(tin->T)), t, n)
  {
   idx[i++] = (int)t->v1()->x;
   idx[i++] = (int)t->v2()->x;
   idx[i++] = (int)t->v3()->x;
   idx[i++] = -1;
  }
  its->coordIndex.finishEditing();

  i=0; FOREACHVVVERTEX((&(tin->V)), v, n) v->x = ox[i++];
  free (ox);

 SoSeparator *root = new SoSeparator;
  root->addChild(its);

 viewer->setSceneGraph(root);
}


void display_message(char *message, int action)
{
 static MainCanvas *mc = NULL;

 if (mc == NULL) {mc = (MainCanvas *)message; return;}

 if (action == DISPMSG_ACTION_ERRORDIALOG)
 {
  QMessageBox::critical(0, QString("Unrecoverable Error"), QString(message+8),
			QMessageBox::Abort, QMessageBox::NoButton);
  exit(-1);
 }
 else if (action == DISPMSG_ACTION_PUTPROGRESS)
 {
  mc->statusBar()->showMessage(message+1);
 }
 else if (action == DISPMSG_ACTION_PUTMESSAGE)
 {
  mc->statusBar()->showMessage(message);
 }

 qApp->flush();
}


// 首先定义图中节点和边的属性
struct VertexProperty { // 图节点中保存的信息
    bool operator==(const VertexProperty& other) const {
        return ok == other.ok;
        }
    
    bool operator!=(const VertexProperty& other) const {
        return !(*this == other);
        }
    bool operator<(const VertexProperty& r) const{
        return ok < r.ok;
    }
    unsigned int index;
    int ok;
};
struct EdgeProperty {  // 图边中保存的信息
    unsigned int index;
    float         weight;
};
int main(int argc, char **argv)
{
//using namespace boost;
//  typedef adjacency_list<vecS, vecS, undirectedS, no_property,
//    property< edge_weight_t, int, property< edge_weight2_t, int > > > Graph;
//  const int V = 6;
//  typedef std::pair < int, int >Edge;
//  Edge edge_array[] =
//    { Edge(0, 1), Edge(0, 2), Edge(0, 3), Edge(0, 4), Edge(0, 5),
//    Edge(1, 2), Edge(1, 5), Edge(1, 3), Edge(2, 4), Edge(2, 5)
//  };
//  const std::size_t E = sizeof(edge_array) / sizeof(Edge);
//#if defined(BOOST_MSVC) && BOOST_MSVC <= 1300
//  // VC++ can't handle the iterator constructor
//  Graph g(V);
//  for (std::size_t j = 0; j < E; ++j)
//    add_edge(edge_array[j].first, edge_array[j].second, g);
//#else
//  Graph g(edge_array, edge_array + E, V);
//#endif
//
//  property_map < Graph, edge_weight_t >::type w = get(edge_weight, g);
//  int weights[] = { 1,1, 1, 1, 1, 3, 1, 1, 1, 2};
//  int *wp = weights;
//
//  graph_traits < Graph >::edge_iterator e, e_end;
//  for (boost::tie(e, e_end) = edges(g); e != e_end; ++e)
//    w[*e] = *wp++;
//
//  int D[V][V];
//  johnson_all_pairs_shortest_paths(g, D);
//
//  std::cout << "       ";
//  for (int k = 0; k < V; ++k)
//    std::cout << std::setw(5) << k;
//  std::cout << std::endl;
//  for (int i = 0; i < V; ++i) {
//    std::cout << std::setw(3) << i << " -> ";
//    for (int j = 0; j < V; ++j) {
//      if (D[i][j] == (std::numeric_limits<int>::max)())
//        std::cout << std::setw(5) << "inf";
//      else
//        std::cout << std::setw(5) << D[i][j];
//    }
//    std::cout << std::endl;
//  }
//
//  std::ofstream fout("johnson-eg.dot");
//  fout << "digraph A {\n"
//    << "  rankdir=LR\n"
//    << "size=\"5,3\"\n"
//    << "ratio=\"fill\"\n"
//    << "edge[style=\"bold\"]\n" << "node[shape=\"circle\"]\n";
//
//  graph_traits < Graph >::edge_iterator ei, ei_end;
//  for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
//    fout << source(*ei, g) << " -> " << target(*ei, g)
//      << "[label=" << get(edge_weight, g)[*ei] << "]\n";
//
//  fout << "}\n";
//  return 0;
 JMesh::init();
 JMesh::app_name = "EfPiSoft";
 JMesh::app_version = "1.0";
 JMesh::app_year = "2006";
 JMesh::app_authors = "Marco Attene";
 JMesh::app_maillist = "attene@ge.imati.cnr.it";

 if (SoQt::init(argv[0]) == NULL) exit(1);

 MainCanvas *mc = new MainCanvas(NULL);
 display_message((char *)mc, 0);

 mc->show();

 JMesh::display_message = display_message;

 if (argc > 1) mc->openFile(QString(argv[1]));

 SoQt::mainLoop();
}
