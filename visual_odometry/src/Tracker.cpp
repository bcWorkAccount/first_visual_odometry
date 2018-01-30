/**
* This file is part of FVO.
* Tracker: use the ORB to fetch the features and descriptors
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/


#include "fvo/Tracker.h"
#include "fvo/Frame.h"


namespace fvo{

    Tracker::Tracker(  Camera::Ptr pCamera  ) {
        // initial the camera and orb extractor
        mpCamera = pCamera;
        mpExtractor = make_shared<ORBExtractor>(ORBExtractor::OPENCV_GFTT);

        mState = NO_IMAGES_YES;
    }

    Tracker::~Tracker() {
    }

    SE3d Tracker::InsertStereo(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp)
    {
        //Todo
        LOG(WARNING) << "Todo Tracker::InsertStereo() " << endl;

        // create current frame
        mpCurrentFrame = shared_ptr<Frame>( new Frame(imLeft, imRight, timestamp, mpCamera));
        mpCurrentFrame->ComputeImagePyramid();

        // Extract and compute stereo matching in current frame
        // Extract the keypoints and compute the descriptors
        // like: mpExtractor->detect(img, keypoints, descriptors);
        mpExtractor->detect( mpCurrentFrame->mImgLeft, true );
        mpExtractor->detect( mpCurrentFrame->mImgRight, true );

        return SE3d();
    }

} // end of fvo
