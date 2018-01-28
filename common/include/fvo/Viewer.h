/**
* This file is part of FVO.
* Viewer: response for camera poses , point cloud and images show
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/

#ifndef FIRSTVO_VIEWER_H
#define FIRSTVO_VIEWER_H

#include <pangolin/pangolin.h>
#include "fvo/common_headers.h"

using namespace std;

namespace fvo {

class Viewer {
public:
    // ViewThread is running or not
    bool mbRunning = false;
    // the Viewer thread
    std::thread mViewerThread;
    // bias curve log
    pangolin::DataLog mBiasLog;
    std::vector<std::string> mBiasLogLabels;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer( bool startViewer = true );
    virtual ~Viewer();

    // Viewer Thread Start Function
    void run();

    // draw the basic and fixed ui items for all frames
    void DrawOrigin();
};

} // end of namespace


#endif //FIRSTVO_VIEWER_H
