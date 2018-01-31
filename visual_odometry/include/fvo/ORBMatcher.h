/**
* This file is part of FVO.
* ORBMatcher: the keypoint descriptors matching
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 28th Jan, 2018
*/

#ifndef FIRSTVO_ORBMATCHER_H
#define FIRSTVO_ORBMATCHER_H

#include "fvo/common_headers.h"

using namespace std;

namespace fvo {
    class ORBMatcher {
    public:  // definitions
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<ORBMatcher> Ptr;

    private: // properties
        // the matcher
        cv::Ptr<cv::DescriptorMatcher> mpMatcher;

    public: // methods
        ORBMatcher();
        virtual ~ORBMatcher();
        // compute the left and right original keypoints, descriptors, select the good matches
        void ComputeStereoMatches( const cv::Mat&  leftDescriptors,const cv::Mat&  rightDescriptors, vector<cv::DMatch>& matches);
    };

}// end of namespace

#endif //FIRSTVO_ORBMATCHER_H
