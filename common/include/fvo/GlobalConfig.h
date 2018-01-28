/**
* This file is part of FVO.
* GlobalConfig: store the global configuration items for all other class
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/


#ifndef FISTVO_GLOBALCONFIG_H
#define FISTVO_GLOBALCONFIG_H

namespace fvo {
    namespace G{
        // image oringal width and height
        extern int imageWidth;
        extern int imageHeight;

        // ORB Extractor Threshold
        extern int initTHFAST;  // initial threshold
        extern int minTHFASE;   // minimal threshold for worse situation
        extern int nFeatures;   // number of features

        // images number
        extern int images;

        // camera parameters
        extern float fx;
        extern float fy;
        extern float cx;
        extern float cy;
        extern float bf;

        // viewer parameters
        extern int viewerHeight ;
        extern int viewerWidth;
        extern int UI_WIDTH;

} // end of namespace

} // end of namespace

#endif //FISTVO_GLOBALCONFIG_H
