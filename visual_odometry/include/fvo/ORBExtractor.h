/**
* This file is part of FVO.
* ORBExtractor: fetch the features and descriptors
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/

#ifndef FIRSTVO_ORBEXTRACTOR_H
#define FIRSTVO_ORBEXTRACTOR_H

#include "fvo/common_headers.h"
#include "fvo/Feature.h"

using namespace std;

namespace fvo {


    /* 一个图像区域的象限划分：:
    UL_(1)   |    UR_(0)
    ---------|-----------
    BL_(2)   |    BR_(3)
    */
    class  ExtractorNode
    {
    public:
        ExtractorNode() :is_no_more_(false){}

        // 划分四叉树
        void divideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    public:
        std::vector<cv::KeyPoint> vec_keys_;
        cv::Point2i UL_, UR_, BL_, BR_;
        std::list<ExtractorNode>::iterator node_iter_;
        bool is_no_more_;
    };

    class ORBExtractor {
    public:  // definitions
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<ORBExtractor> Ptr;

        // the method to fetch the features
        typedef enum{
            OPENCV_ORB,     // OpenCV ORB
            OPENCV_SURF,    // OPenCV SURF
            QUAD_TREE_ORB,  // Quad Tree ORB
            OPENCV_GFTT,    // OpenCV's Good Feature to Track
        }KeyPointMethod;


    public: // properties
        //  pyramid vector for image to store every level image
        std::vector<cv::Mat> mvecImagePyramid;

    private:   // properties
        // the features fetch method
        KeyPointMethod  mMethod;
        // training pattern
        std::vector<cv::Point> mvecPattern;
        // the max features number
        int mnFeatures;
        // the pyramid level number
        int mnLevel;
        // the scale factor between two level
        float mfScaleFactor;
        // the scale factor of each level
        std::vector<float> mvecScaleFactor;
        // the default fast corner number threshold
        int mnDefaultFastTh;
        // the min fast corner number threshold
        int mnMinFastTh;

        // the features number of each level
        std::vector<int> mvecFeatureNumPerLevel;
        // 用于存储计算特征方向时，图像每个v坐标对应最大的u坐标
        std::vector<int> mvecUMax;

    public:   // methods
        // Constructor with method
        ORBExtractor(const KeyPointMethod& method );
        virtual ~ORBExtractor();

        // detect features and descriptors for frame
        void detect( cv::InputArray, vector<cv::KeyPoint>& output_keypoints,
                     cv::OutputArray descriptors_array );

    private:  // methods
        // ORB detector with Quad Tree detect
        void computeKeyPointsQuadTree( const cv::Mat& image, vector<vector<cv::KeyPoint>>& allKeyPoints);
        // OpenCV's good Fast detector
        void ComputeKeyPointsGFTT( cv::Mat image, vector<vector<Feature::Ptr>>& allKeyPoints);
        // compute and build the image pyramid of each level
        void computeImagePyramid(cv::Mat image);
        // quad tree distribute to key points
        vector<cv::KeyPoint> distributeQuadTree(const vector<cv::KeyPoint>& vec_to_distribute_keys,
                 const int &min_x, const int &max_x,
                 const int &min_y, const int &max_y,
                 const int &feature_num, const int &level);
        // compute the available keypoints and descriptors from all keypoints in pyramid .
        void computeDescriptors(vector<vector<cv::KeyPoint>>& all_keypoints, vector<cv::KeyPoint>& output_keypoints, cv::OutputArray descriptors_array);
    };


} // end of namespace
#endif //FIRSTVO_ORBEXTRACTOR_H
