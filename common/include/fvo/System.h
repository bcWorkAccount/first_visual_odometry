/**
* This file is part of FVO.
* System: System of the VO
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/

#ifndef FIRSTVO_SYSTEM_H
#define FIRSTVO_SYSTEM_H

#include "fvo/common_headers.h"
#include "fvo/Tracker.h"
#include "fvo/Viewer.h"
#include "fvo/Camera.h"

using namespace std;

namespace fvo {

class System {

public:
    Camera::Ptr mpCamera = nullptr;
    Tracker::Ptr mpTracker = nullptr;
    shared_ptr<Viewer> mpViewer = nullptr;

public:
    System(const string &configPath) ;
    virtual ~System();

    /**
     * Trace the stereo images
     * @param imLeft
     * @param imRight
     * @param timestamp
     * @return SE3
     */
    SE3d traceStereo(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp );

    /**
     * Set the true trajectory
     * @param traj
     * @return
     */
    void SetGroundTruthTrajectory(map<double, SE3d, std::less<double>, Eigen::aligned_allocator<SE3d> >& traj);

    /**
     * Stop all threads
     */
    void shutdown();
};

} // end of namespace
#endif //FIRSTVO_SYSTEM_H
