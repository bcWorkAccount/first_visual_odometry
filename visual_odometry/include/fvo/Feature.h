/**
* This file is part of FVO.
* Feature: the structure of the feature
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 29th Jan, 2018
*/

#ifndef FISTVO_FEATURE_H
#define FISTVO_FEATURE_H

#include "fvo/common_headers.h"

namespace fvo {

class Feature {
public:  // definitions
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

public:  // properties
    // the pixel position
    Vector2f mPixel = Vector2f(0,0);

    // data used in ORB
    float mAngle = 0;           // angle of oriented FAST
    size_t mLevel = 0;          // the pyramid level
    uchar mDesc[32] = {0};        // 256 bits of ORB features( 32* 8 )

    // flags
    bool mbOutlier = false;      // true if it is an outlier
};


}  // end of namespace

#endif //FISTVO_FEATURE_H
