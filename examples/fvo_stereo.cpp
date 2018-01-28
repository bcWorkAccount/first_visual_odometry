/**
* This file is part of FVO.
* FVO: Fist Visual Odometry
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/


#include "fvo/common_headers.h"
#include "fvo/System.h"
#include "fvo/ImageUtils.h"


using namespace std;
using namespace fvo;



int main( int argc, char ** argv ) {
    // Check the arguments numbers
    if ( argc != 4) {
        std::cerr << std::endl << "Usage: ./fvo  path_to_setting_file path_to_groudtruth_file  path_to_sequence_dir  " << std::endl;
        return 1;
    }
    // Check the directories existing
    string sSettingFile = string( argv[1] );
    string sGroudtruthFile = string( argv[2] );
    string sSequenceDir = string( argv[3] );
    bool isExist = fvo::FileState::isFileExist(sSettingFile) &&
            fvo::FileState::isFileExist(sGroudtruthFile) &&
            fvo::FileState::isDirExist(sSequenceDir) ;
    if ( !isExist ) {
        std::cerr << std::endl << "Can't find the input directories." << std::endl;
        return 1;
    }
    // Retrieve paths to images
    vector<string> vsImgLeft;
    vector<string> vsImgRight;
    vector<double> vTimestamps;
    ImageUtils::LoadImages( sSequenceDir, vsImgLeft,vsImgRight, vTimestamps );

    // Get the total image number
    const long nImages = vsImgLeft.size();

    // Create SLAM System.
    // It initialize all system threads and get ready to process frames.
    fvo::System system( sSettingFile );

    // Load ground truth trajectory
    //map<double, SE3d, std::less<double>, Eigen::aligned_allocator<SE3d>> mapTraj;
    //ImageUtils::LoadGroundTruth( sGroudtruthFile , vTimestamps, mapTraj );

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize( nImages );


    LOG(INFO) << "Start processing sequence ..." << endl;
    LOG(INFO) << "Images # in the sequence: " << nImages << endl;


    // Main loop
    // the left and right image matrix
    cv::Mat imLeft, imRight;
    for( int i=0; i<nImages; i++ ) {
        // Read left and right images from file
        imLeft = cv::imread( vsImgLeft[i], CV_LOAD_IMAGE_GRAYSCALE );
        imRight = cv::imread( vsImgRight[i], CV_LOAD_IMAGE_GRAYSCALE );
        double tframe = vTimestamps[i];

        if( imLeft.empty() || imRight.empty() ) {
            LOG(WARNING) << "Can't load image " << i << endl;
            continue;
        }
        // Pass the images to the SLAM system
        //system.traceStereo(imLeft, imRight, tframe );
        LOG(INFO) << "stereo " << vsImgLeft[i] << " , " << vsImgRight[i] << endl;

    }

    // Stop all threads
    //system.shutdown();
    LOG(INFO) << "end of program" << endl;

    return 0;
}



