/**
* This file is part of FVO.
* Frame: the structure of the frame
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 28th Jan, 2018
*/

#include "fvo/Frame.h"

namespace fvo {

    Frame::Frame( const cv::Mat& imLeft, const cv::Mat& imRight,
           const double& timestamp, fvo::Camera::Ptr pCamera)
    {
        // Todo
        LOG(WARNING) << "Todo : Frame::Frame() " << endl;
    }
    Frame::~Frame(){};

    void Frame::ComputeImagePyramid()
    {
        // Todo
        LOG(WARNING) << "Todo: Frame::ComputeImagePyramid() " << endl;

    }
}
