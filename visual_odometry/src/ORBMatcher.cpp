//
// Created by hadoop on 1/27/18.
//

#include <common/include/fvo/GlobalConfig.h>
#include "fvo/ORBMatcher.h"


namespace fvo {
    ORBMatcher::ORBMatcher() {
        mpMatcher = cv::DescriptorMatcher::create("BruteForce");

    }

    ORBMatcher::~ORBMatcher() {}

    // compute the left and right original descriptors, select the good matches
    void ORBMatcher::ComputeStereoMatches(const cv::Mat&  leftDescriptors,
                              const cv::Mat&  rightDescriptors, vector<cv::DMatch>& matches )
    {
        assert ( leftKeyPoints.size() != 0 );
        assert ( rightKeyPoints.size() != 0 );

        matches.clear();

        cv::flann::Index  flannIndex( leftDescriptors, cv::flann::LshIndexParams(12,20,2),
                        cvflann::FLANN_DIST_HAMMING);

        cv::Mat matchIndex( rightDescriptors.rows, 2, CV_32SC1 );
        cv::Mat matchDistance( rightDescriptors.rows, 2, CV_32SC1 );
        flannIndex.knnSearch( rightDescriptors, matchIndex, matchDistance,2, cv::flann::SearchParams());
        for( int i=0; i<matchDistance.rows; i++ ) {
            if ( matchDistance.at<float>(i,0) < 0.6 * matchDistance.at<float>(i,1) ) {
                cv::DMatch midDMatch( i, matchIndex.at<int>(i,0), matchDistance.at<float>(i,1) );
                matches.push_back( midDMatch );
            }
        }

        //mpMatcher->match( leftDescriptors, rightDescriptors, matches );
    }
}