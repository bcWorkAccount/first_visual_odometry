/**
* This file is part of FVO.
* Tracker: track the camera poses
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/

#ifndef FIRSTVO_TRACKER_H
#define FIRSTVO_TRACKER_H

#include "fvo/common_headers.h"
#include "fvo/Camera.h"
#include "fvo/ORBExtractor.h"
#include "fvo/Frame.h"


using namespace std;

namespace fvo{

class Tracker {
public:  // definitions
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Tracker> Ptr;

    typedef enum{
        SYSTEM_NOT_READY = -1,
        NO_IMAGES_YES = 0,
        OK = 2
    }TrackingState;

private:  // properties
    // Camera parameters
    Camera::Ptr mpCamera = nullptr;
    // ORB Extractor
    ORBExtractor::Ptr mpExtractor = nullptr;
    Frame::Ptr mpCurrentFrame = nullptr;
    // current state of tracker
    TrackingState mState = SYSTEM_NOT_READY;

public: // methods
    Tracker( Camera::Ptr pCamera );
    virtual ~Tracker();
    // deal with the stereo images
    SE3d InsertStereo(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp);
};


} // end of namespace

#endif //FIRSTVO_TRACKER_H
