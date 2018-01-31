/**
* This file is part of FVO.
* Tracker: use the ORB to fetch the features and descriptors
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/
/*
 * History:
 *    31st Jan,2018, Arthur.Chen, initial InsertStereo() funciton
 */


#include "fvo/Tracker.h"
#include "fvo/Frame.h"


namespace fvo{

    Tracker::Tracker(  Camera::Ptr pCamera  ) {
        // initial the camera and orb extractor
        mpCamera = pCamera;

        // use the default OPENCV::ORB to detect
        //    the alternative are: QUAD_TREE, SURF, GFTT
        mpORBExtractor = make_shared<fvo::ORBExtractor>(ORBExtractor::OPENCV_ORB);
        mpORBMatcher = make_shared<fvo::ORBMatcher>();
        LOG(INFO) << "Use the ORBExtractor::OPENCV_ORB to detect features." << endl;

        mState = NO_IMAGES_YES;
    }

    Tracker::~Tracker() {
    }

    SE3d Tracker::InsertStereo(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp)
    {
        //Todo : not completed
        LOG(WARNING) << "Todo : Tracker::InsertStereo() " << endl;

        // create current frame
        mpCurrentFrame = shared_ptr<Frame>( new Frame(imLeft, imRight, timestamp, mpCamera));

        // Extract and compute stereo matching in current frame
        // left & right key points , descriptors
        vector<cv::KeyPoint> leftKeyPoints, rightKeyPoints;
        cv::Mat  leftDescriptors, rightDescriptors;
        mpORBExtractor->detect( imLeft, leftKeyPoints, leftDescriptors);
        mpORBExtractor->detect( imRight, rightKeyPoints, rightDescriptors );

        vector<cv::DMatch> matches;
        mpORBMatcher->ComputeStereoMatches(leftDescriptors, rightDescriptors, matches );
        LOG(INFO) << "good/totoal :" << matches.size() << "/" << rightDescriptors.size() << endl;


        // Todo : compute the camera orientation ( facing direction ) with triangle


        return SE3d();
    }

} // end of fvo
