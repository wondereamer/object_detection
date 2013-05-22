#ifndef _EMD_H
#define _EMD_H
/*
    emd.h

    Last update: 3/24/98

    An implementation of the Earth Movers Distance.
    Based of the solution for the Transportation problem as described in
    "Introduction to Mathematical Programming" by F. S. Hillier and 
    G. J. Lieberman, McGraw-Hill, 1990.

    Copyright (C) 1998 Yossi Rubner
    Computer Science Department, Stanford University
    E-Mail: rubner@cs.stanford.edu   URL: http://vision.stanford.edu/~rubner
*/


/* DEFINITIONS */
#define MAX_SIG_SIZE   100
#define MAX_ITERATIONS 500
#define INFINITY       1e20
#define EPSILON        1e-6
#include <library/m_geometry.h>
#include "vizblockworld.h" 
#include <pcl/common/common_headers.h>

/*****************************************************************************/
/* feature_t SHOULD BE MODIFIED BY THE USER TO REFLECT THE FEATURE TYPE      */
typedef pcl::PointXYZRGB feature_t;
/*****************************************************************************/


typedef struct
{
  int n;                /* Number of features in the signature */
  feature_t *Features;  /* Pointer to the features vector */
  float *Weights;       /* Pointer to the weights of the features */
} signature_t;


struct flow_t
{
  flow_t():from(0), to(0), amount(0){ }
  int from;             /* Feature number in signature 1 */
  int to;               /* Feature number in signature 2 */
  float amount;         /* Amount of flow from "from" to "to" */
  bool operator < (const flow_t &other) const{
//      std::cout<<"*************1****"<<std::endl;
//      other.amount;
//      std::cout<<"*************2****" <<std::endl;
      return amount < other.amount ? false : true;
  }
};



/**
 * @brief 
 *
 * @param Signature1 supplier
 * @param Signature2 demand
 * @param func
 * @param Flow
 * @param FlowSize
 *
 * @return 
 */
float emd(signature_t *Signature1, signature_t *Signature2,
	  float (*func)(feature_t *, feature_t *),
	  flow_t *Flow, int *FlowSize);

typedef std::vector<feature_t, Eigen::aligned_allocator<pcl::PointXYZRGB>> Features;
float emd_wapper(const Features &supplier, const std::vector<float> sWeight,
        const Features &demand, const std::vector<float> dWeight, flow_t *flow,
        float (*dist_fun)(feature_t *F1, feature_t *F2), VizBlockWorld *viz = NULL, float radius = 0.1
        );

#endif
