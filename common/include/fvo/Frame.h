/**
* This file is part of FVO.
* Frame: the structure of the frame
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 28th Jan, 2018
*/

#ifndef FIRSTVO_FRAME_H
#define FIRSTVO_FRAME_H

#include "fvo/common_headers.h"
#include "fvo/Camera.h"

using namespace std;
using namespace fvo;

namespace fvo {

class Frame {
public:   // definitions
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

public:
    cv::Mat mImgLeft,  mImgRight;  // left and right image

public:   // methods
    // constructor
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight,
          const double &timestamp, fvo::Camera::Ptr pCamera);

    virtual ~Frame();

};

} // end of namespace

#endif //FIRSTVO_FRAME_H
