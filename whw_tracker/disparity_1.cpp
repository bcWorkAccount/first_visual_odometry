//
// Created by 高翔 on 2017/12/15.
//

#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <sophus/se3.h>

using namespace std;
using namespace Eigen;
using namespace cv;
// 文件路径，如果不对，请调整
string left_file = "./left.png";
string right_file = "./right.png";
string left2_file = "./left2.png";
typedef Matrix< double, 6, 1 > Vector6d;
int main(int argc, char **argv) {

    // 内参
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    // 间距
    double d = 0.573;
    Matrix3d K;
      K<< fx , 0 , cx,
          0 , fy , cy,
          0 , 0 , 1;
    cv::Mat left = cv::imread(left_file, 0);
    cv::Mat right = cv::imread(right_file, 0);
    cv::Mat left2 = cv::imread(left2_file, 0);
   //get keypoint use opencv
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(left, kp1);

   vector<KeyPoint> kp2;
    Ptr<GFTTDetector> detector2 = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector2->detect(left2, kp2);
   //match use opencv
    vector<Point2f> leftKPs, rightKPs,left2KPs;
    for (auto &kp: kp1){ 
    leftKPs.push_back(kp.pt);
    }
    for (auto &kp: kp2){
    left2KPs.push_back(kp.pt);
    }
    vector<uchar> status;
    vector<float> error;
    cv::calcOpticalFlowPyrLK(left, right, leftKPs, rightKPs, status, error, cv::Size(31, 31),3);
   //imshow the picture
    for (int i = 0; i < rightKPs.size(); i++) {
        if (status[i]) {
            cv::circle(right, rightKPs[i], 2, cv::Scalar(0, 250, 255), 2);
            cv::line(right,leftKPs[i], rightKPs[i], cv::Scalar(0, 250, 255));
        }
    }

    cv::imshow("tracked by opencv", right);
    //compute 3Dpoint 
    Vector3d point3D;
    vector<Vector3d, Eigen::aligned_allocator<Vector3d>> pointcloud;

    for (int i = 0; i < leftKPs.size(); i++)
       {
           // start your code here (~6 lines)
            // 根据双目模型计算 point 的位置
            
            unsigned int ulr=leftKPs[i].x-rightKPs[i].x;
            if(ulr==0) continue;
           point3D[2]=fx*d/ulr;
           point3D[0]=(leftKPs[i].x-cx)*point3D[2]/fx;
           point3D[1]=(leftKPs[i].y-cy)*point3D[2]/fy;
           pointcloud.push_back(point3D);
            }
    //BA compute T
    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = pointcloud.size();
    cout << "points: " << nPoints << endl;
    Sophus::SE3 T_esti;
    // estimated pose

    for (int iter = 0; iter < iterations; iter++) {
        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
             if(left2KPs[i].y-8 > 0 
              && left2KPs[i].x-8 > 0 
              && 8+left2KPs[i].y<left2.rows 
              && 8+left2KPs[i].x<left2.cols){
            
           Vector3d p_camera=T_esti*pointcloud[i];//相機坐標系（x、y、z）
           Vector3d uvs=K*p_camera;//投影模型
           Vector3d uv=K*p_camera/p_camera(2);//歸一化

           Vector2d e;
            e={left2KPs[i].x-uv(0),left2KPs[i].y-uv(1)};
           cost +=sqrt(e.transpose()*e);

            // compute jacobian
            Matrix<double, 2, 6> J;
 
            double X=p_camera(0);
            double Y=p_camera(1);
            double Z=p_camera(2);
            double Z2=p_camera(2)*p_camera(2);
           J<<-fx/Z,   0,  fx*X/Z2, fx*X*Y/Z2, -fx-fx*X*X/Z2,  fx*Y/Z,
                0, -fy/Z, fy*Y/Z2, fy+fy*Y*Y/Z2, -fy*X*Y/Z2,  -fy*X/Z;
            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        // solve dx 
        Vector6d dx;
         dx=H.ldlt().solve(b);
       if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
         T_esti=Sophus::SE3::exp(dx)*T_esti;
        lastCost = cost;
      cout<<"T_esti: "<<T_esti<<endl;
    }
}
    waitKey(0);
    return 0;
}
