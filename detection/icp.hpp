/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: icp.hpp 5980 2012-06-24 16:07:14Z rusu $
 *
 */

#include <boost/unordered_map.hpp>
#include <algorithm>
#include <library/m_util.h>
#include "vizblockworld.h" 
#include <cmath>
float dist(feature_t *F1, feature_t *F2) { 
    return sqrt(pow((F1->x - F2->x), 2) + pow((F1->y - F2->y), 2) + 
            pow((F1->z - F2->z), 2));
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointSource, typename PointTarget> 
void IterativeClosestPoint<PointSource, PointTarget>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess)
{
    // Allocate enough space to hold the results
    nr_iterations_ = 0;
    converged_ = false;
    output_ = &output;
    assert(target_->points.size() && output.points.size());
    assert(wInput_.size() && wOutput_.size());
    if (flow_) {
        delete []flow_;
    }
    flow_ = new flow_t[wInput_.size() + wOutput_.size() - 1];
    float emd_change;
    int flowSize = wInput_.size() + wOutput_.size() - 1;
    while (!converged_)           // repeat until convergence
    {

        /// calcuate EMD distance
        float e = emd_wapper(target_->points, wInput_, output.points, wOutput_, flow_, dist);
        emd_change = abs(e - pre_EMD_);
        pre_EMD_ = e;
        /// if converged
        if (nr_iterations_ >= max_iterations_ || emd_change < 0.001)
        {
            converged_ = true;
            return;
        }
        m_util::print_list(wInput_);
        /// if calcuate response automatically
        std::cout<<"flow:"<<std::endl;
        if (autoResponse_) {
            myCorrespondence_.clear();
            std::sort(flow_, flow_ + flowSize);
            for (int i = 0; i < flowSize; i++) {
                if (flow_[i].amount > 0) {
                std::cout<<flow_[i].from<<" "<<flow_[i].to<<" "<<flow_[i].amount<<std::endl;
                myCorrespondence_.insert(BiValue(flow_[i].from, flow_[i].to));
                }
            }
        }
        std::cout<<"*************correspondences*****"<<std::endl;    
        m_util::print_map(myCorrespondence_.left);
        std::cout<<"*********************************"<<std::endl;    
        // --------------------------------------------------------------------------------------
        /// transform the points
        int cnt = myCorrespondence_.size();
        if (cnt < min_number_correspondences_)
        {
            std::cout<<"error: Not enough correspondences found." <<std::endl;
            converged_ = false;
            return;
        }
        std::vector<int> source_indices (cnt);  // output 
        std::vector<int> target_indices (cnt); // input
        int idx = 0;
        for(auto v : myCorrespondence_.left){
            source_indices[idx] = v.second;
            target_indices[idx] = v.first;
            idx++;
        }
        // Estimate the transform
        transformation_estimation_->estimateRigidTransformation (output, source_indices,
                                              *target_, target_indices, transformation_);
        // Tranform the data
        transformPointCloud (output, output, transformation_);

        // Obtain the final transformation    
        final_transformation_ = transformation_ * final_transformation_;
        nr_iterations_++;
    }
}

template <typename PointSource, typename PointTarget> void
IterativeClosestPoint<PointSource, PointTarget>::setInputWeight(const std::vector<float> &wInput){
//    std::copy(wInput.begin(), wInput.end(), back_inserter(wInput_));
    wInput_ = wInput;
}
template <typename PointSource, typename PointTarget> void 
IterativeClosestPoint<PointSource, PointTarget>::setOutputWeight(const std::vector<float>  &wOutput){
//    std::copy(wOutput.begin(), wOutput.end(), back_inserter(wOutput_));
    wOutput_ = wOutput;
}

template <typename PointSource, typename PointTarget> void 
IterativeClosestPoint<PointSource, PointTarget>::visualize_correspondence(VizBlockWorld *viz
                                                            , int view) const{
    //    for (int i=0; i < target_->points.size(); i++){
    //        auto &p = target_->points[i];
    //        // mark source
    //        viz->add_text3D(m_util::sth2string<float>(wInput_[i]) , p.x,
    //                p.y, p.z, 0, 255,0, 0.2);
    //    }
    //    for (int i=0; i < output_->points.size(); i++){
    //        auto &p = output_->points[i];
    //        // mark source
    //        viz->add_text3D(m_util::sth2string<float>(wInput_[i]) , p.x,
    //                p.y, p.z, 0, 255,0, 0.2);
    //    }
    auto &in2out = myCorrespondence_.left;
    m_util::print_map(in2out);
    for(auto &v : in2out){
        auto &pIn = target_->points[v.first];
        auto &pOut = output_->points[v.second];
        // mark flow
        viz->add_line(pIn.x, pIn.y, pIn.z,
                pOut.x, pOut.y, pOut.z, 255, 0, 0, view);
    }

}

