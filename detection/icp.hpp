
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
    float emd_change;
    int flowSize = wInput_.size() + wOutput_.size() - 1;
    flow_ = new flow_t[flowSize];
    std::vector<int*> samples;
//    samples.push_back
    const int RAND_NUM = 3;
    const int RAND_TOTAL = 6;
    int a[] = { 0, 1, 2};
    int b[] = { 0, 2, 1};
    int c[] = { 1, 0, 2};
    int d[] = { 1, 2, 0};
    int e[] = { 2, 1, 0};
    int f[] = { 2, 0, 1};
    samples.push_back(a);
    samples.push_back(b);
    samples.push_back(c);
    samples.push_back(d);
    samples.push_back(e);
    samples.push_back(f);
    std::vector<pair<float,int>> vecOutput;
    std::vector<pair<float,int>> spOutput; 
    std::sort(wInput_.begin(), wInput_.end(), std::greater<float>());
    std::sort(wOutput_.begin(), wOutput_.end(), std::greater<float>());
    int flag = 0;
    for (auto &sample: samples) {
        /// @todo to interate over all possible case 
        /// rather than sample 100 times
        /// reset condition
        bool stop = false;
        nr_iterations_ = 0;
        emd_change = 10000;
        /// random sample initial correspondences
        /// sampled first MINIMUM_CRSP elements in #output 
        myCorrespondence_.clear();
        const int MINIMUM_CRSP = 3;
        // copy the first MINIMUM_CRSP elements in #output
        for (int i = 0; i < RAND_NUM; i++) {
            myCorrespondence_.insert(BiValue(i, sample[i]));
        }
        assert(myCorrespondence_.size() == MINIMUM_CRSP);


        //            std::cout<<"*************correspondences*****"<<std::endl;    
        //            m_util::print_map(myCorrespondence_);
        //            std::cout<<"*********************************"<<std::endl;    
        /// repeat until convergence
        while (!stop)        
        {

            /// if converged
            if (emd_change < 0.001)
            {
                stop = true;
                converged_ = true;
                break;
            }
            if (nr_iterations_ >= max_iterations_ ){
                stop = true;
                break;
            }
            /// transform the points
            int cnt = myCorrespondence_.size();
            // ignore this sample
            if (cnt < min_number_correspondences_)
                break;
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
                /// @todo arguments
                if(myCorrespondence_.size() == 5)
                    break;
            }
//            std::cout<<"*********************************"<<std::endl;    
//            for (int i = 0; i < flowSize; i++) {
//                std::cout<<flow_[i].from<<" "<<flow_[i].to<<" "<<flow_[i].amount<<std::endl;
//            }
//            std::cout<<"***********----------************"<<std::endl;    
//            for(auto p : myCorrespondence_.left){
//                 std::cout<<p.first<<" "<<p.second<<std::endl;
//            }
//            std::cout<<myCorrespondence_.size()<<std::endl;
//            assert(myCorrespondence_.size() >= MINIMUM_CRSP);

        
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

