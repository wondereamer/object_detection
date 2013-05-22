
#include <boost/unordered_map.hpp>
#include <algorithm>
#include <library/m_util.h>
#include <library/m_math.h>
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
        assert(false);
        delete []flow_;
    }
    float emd_change;
    int flowSize = wInput_.size() + wOutput_.size() - 1;
    flow_ = new flow_t[flowSize];
    for (int i = 0; i < 100; i++) {
        /// reset condition
        converged_ = false;
        nr_iterations_ = 0;
        emd_change = 10000;
        /// random sample initial correspondences
        myCorrespondence_.clear();
        const int MINIMUM_CRSP = 3;
        std::map<float, int> sInput;
        std::map<float, int> sOutput;
        std::vector<pair<float,int>> vecOutput;
        std::vector<pair<float,int>> spOutput; // sampled first MINIMUM_CRSP elements in #output 
        m_util::sort_index(wInput_, &sInput);
        m_util::sort_index(wOutput_, &sOutput);
        // copy the first MINIMUM_CRSP elements in #output
        auto iter = sOutput.begin();
        int count = 0;
        while(iter != sOutput.end() && count++ != MINIMUM_CRSP)
            vecOutput.push_back(*iter++);
        // sample the first MINIMUM_CRSP elements in #output
        m_math::rand_sample(vecOutput, MINIMUM_CRSP, &spOutput);
        //
        count = 0;
        iter = sInput.begin();
        while(iter != sInput.end() && count != MINIMUM_CRSP){
            myCorrespondence_.insert(BiValue(iter->second, spOutput[count].second));
            count++;
            iter++;
        }
        assert(myCorrespondence_.size() == MINIMUM_CRSP);


        //            std::cout<<"*************correspondences*****"<<std::endl;    
        //            m_util::print_map(myCorrespondence_);
        //            std::cout<<"*********************************"<<std::endl;    
        /// repeat until convergence
        while (!converged_)        
        {

            /// if converged
            if (nr_iterations_ >= max_iterations_ || emd_change < 0.001)
            {
                break;
            }
            /// transform the points
            int cnt = myCorrespondence_.size();
            if (cnt < min_number_correspondences_)
            {
                std::cout<<"error: Not enough correspondences found." <<std::endl;
                converged_ = false;
                return;
            }
//            myCorrespondence_.resize(5);
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

            /// calcuate EMD distance
            for (int i = 0; i < flowSize; i++) {
                flow_[i].amount = 0;
            }
            float e = emd_wapper(target_->points, wInput_, output.points, wOutput_, flow_, dist);
            emd_change = abs(e - pre_EMD_);
            pre_EMD_ = e;

            /// set up the new correspondences
            // sort the flow according to the weight
            std::stable_sort(flow_, flow_ + flowSize);
            // I don't know why #std::sort cause a bug here,
            // It would modify #from or #to member of flow_t struct
//            std::sort(flow_, flow_ + flowSize);      
            myCorrespondence_.clear();
            for (int i = 0; i < flowSize; i++) {
                myCorrespondence_.insert(BiValue(flow_[i].from, flow_[i].to));
                // maximum number of correspondences
                if(myCorrespondence_.size() == 5)
                    break;
            }


        }
        if(pre_EMD_ < best_EMD_)
            best_EMD_ = pre_EMD_;
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

