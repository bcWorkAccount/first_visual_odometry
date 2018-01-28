/**
* This file is part of FVO.
* ImageUtils: load images from kitti data set. all is static functions
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/

#ifndef FISTVO_IMAGEUTILS_H
#define FISTVO_IMAGEUTILS_H

#include "fvo/common_headers.h"
using namespace std;

namespace fvo {

class ImageUtils {
public:
    // Load the left&right image's names and timestamps
    static void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                    vector<string> &vstrImageRight, vector<double> &vTimestamps);

    // Load the ground truth data
    static void LoadGroundTruth( const string &gtFile, const vector<double> &vTimestamps,
                          map<double, SE3d, std::less<double>, Eigen::aligned_allocator<SE3d>> &traj);
};

} // end of fvo

#endif //FISTVO_IMAGEUTILS_H
