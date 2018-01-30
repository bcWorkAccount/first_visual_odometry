/**
* This file is part of FVO.
* System: System of the VO
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/

#include "fvo/System.h"
// load the seeting fraom yaml
#include "fvo/Config.h"
// for Glboal Configure Parameters
#include "fvo/GlobalConfig.h"

using namespace std;
using namespace fvo;

namespace fvo {

System::System( const string &configPath)
{
    //Read rectification parameters
    Config config( configPath );
    // read all setting from yaml and store at global items which
    //   are defined at GlobalConfig.h
    config.readAllSettings();

    // create camera
    mpCamera = make_shared<Camera>( G::fx, G::fy, G::cx, G::cy, G::bf ) ;

    // Create a tracker
    mpTracker = make_shared<Tracker>( mpCamera  );

    // Todo
    // create a backend


    // Todo
    // Create a viewer

    // Todo
    LOG(WARNING) << "Todo: System::System()" <<  std::endl;

    LOG(INFO) << "fvo system all ready, waiting for images ..." << endl;
}

System::~System() {
    // Todo
    // destroy all settings
    LOG(WARNING) << "Todo: System::~System()" <<  std::endl;
}


SE3d System::traceStereo(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp )
{
    // invoke the Tracker to track the stereo
    return mpTracker->InsertStereo(imLeft, imRight, timestamp );
}

void System::SetGroundTruthTrajectory( map<double, SE3d, std::less<double>, Eigen::aligned_allocator<SE3d>> & traj)
{
    // Todo
    // invoke the viewer to set the trajectory to the viewer
    if( mpViewer ) {
        // translation part of traj
        map<double , Vector3d, std::less<double>, Eigen::aligned_allocator<SE3d> >  trajTrans;
        for( auto& t: traj) {
            trajTrans[t.first] = t.second.translation();
        }
        //mpViewer->SetGtTraj( trajTrans );
    }

    LOG(WARNING) << "Todo: System::SetGroundTruthTrajectory()" <<  std::endl;
}

void System::shutdown() {
    LOG(INFO) << "System shutdown" << endl;

    // Todo
    // Close the Viewer thread
    //if (mpViewer) {
    //    LOG(INFO) << "Please close the GUI to shutdown all the system" << endl;
    //    mpViewer->WaitToFinish();
    // }
    LOG(WARNING) << "Todo: System::Shutdown()" <<  std::endl;

}


} // end of namespace
