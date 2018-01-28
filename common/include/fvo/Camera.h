/**
* This file is part of FVO.
* Camera: the parameters of camera and translation funcitons between image pixel and camera
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/


#ifndef FIRSTVO_CAMERA_H
#define FIRSTVO_CAMERA_H

#include "fvo/common_headers.h"

namespace fvo {

class Camera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    float fx = 0;
    float fy = 0;
    float fx_inv = 0;
    float fy_inv = 0;
    float cx = 0;
    float cy = 0;
    float b = 0;    // baseline in stereo
    float f = 0;    // focal length
    float bf = 0;   // baseline*focal

    Matrix3f K = Matrix3f::Identity();     // intrinsics matrix
    Matrix3f K_inv = Matrix3f::Identity();  // inverse K

    Camera ( const float &_fx, const float &_fy, const float &_cx, const float &_cy, const float _bf = 0 )
            :fx(_fx),  fy(_fy),  cx(_cx ) , cy(_cy), bf(_bf)
    {
        K <<    fx, 0,  cx,
                0,  fy, cy,
                0,  0,   1;
        fx_inv = 1 / fx;
        fy_inv = 1 / fy;
        K_inv = K.inverse();
        f = ( fx + fy ) * 0.5;
        b = bf / f ;
    }

    // from image pixel to camera point
    inline Vector3d Img2Cam( const Vector3f& px )
    {
        return Vector3d(
              fx_inv * ( px[0] - cx ),
              fy_inv * ( px[0] - cy ),
              1
        );
    }

};


} // end of namespace

#endif //FIRSTVO_CAMERA_H
