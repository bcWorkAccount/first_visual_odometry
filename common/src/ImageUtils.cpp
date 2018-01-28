//
// Created by hadoop on 1/27/18.
//

#include "fvo/ImageUtils.h"

namespace fvo {

// Load the left&right image's names and timestamps
void ImageUtils::LoadImages(const string &strPathToSequence, vector <string> &vstrImageLeft,
                                   vector <string> &vstrImageRight, vector<double> &vTimestamps) {
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for (int i = 0; i < nTimes; i++) {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

// Load the ground truth data
void ImageUtils::LoadGroundTruth(const string &gtFile, const vector<double> &vTimestamps,
                                        map<double, SE3d, std::less<double>, Eigen::aligned_allocator<SE3d>> &traj) {
    ifstream fin(gtFile);
    if (fin) {
        int index = 0;
        while (!fin.eof()) {
            Matrix3d R;
            Vector3d t;
            fin >> R(0, 0) >> R(0, 1) >> R(0, 2) >> t(0, 0) >>
                R(1, 0) >> R(1, 1) >> R(1, 2) >> t(1, 0) >>
                R(2, 0) >> R(2, 1) >> R(2, 2) >> t(2, 0);
            traj[vTimestamps[index++]] = SE3d(R, t);
            if (index >= vTimestamps.size())
                break;
            if (!fin.good())
                break;
        }
    }
}

} // end of namespace