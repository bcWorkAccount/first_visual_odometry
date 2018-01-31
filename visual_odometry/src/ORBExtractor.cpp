/**
* This file is part of FVO.
* ORBExtractor: fetch the features and descriptors
 *    Using the QuadTree and Pyramid to do the features detect and descriptor computing
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 26th Jan, 2018
*/
/*
 * History:
 *    31st Jan,2018, Arthur.Chen, complete the basic features detect and descriptors computing function
 */


#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "fvo/ORBExtractor.h"
// for Glboal Configure Parameters
#include "fvo/GlobalConfig.h"

namespace fvo {

    const int PATCH_SIZE = 31;//用于计算BIREF描述子的特征点邻域大小
    const int HALF_PATCH_SIZE = 15;//用于计算BIREF描述子的特征点邻域大小的一半
    const int EDGE_THRESHOLD = 19;//边缘阈值,靠近边缘阈值以内的像素是不检测特征点的。

    // trainging pattern
    static int bit_pattern_31_[256 * 4] =
    {
        8, -3, 9, 5/*mean (0), correlation (0)*/,
        4, 2, 7, -12/*mean (1.12461e-05), correlation (0.0437584)*/,
        -11, 9, -8, 2/*mean (3.37382e-05), correlation (0.0617409)*/,
        7, -12, 12, -13/*mean (5.62303e-05), correlation (0.0636977)*/,
        2, -13, 2, 12/*mean (0.000134953), correlation (0.085099)*/,
        1, -7, 1, 6/*mean (0.000528565), correlation (0.0857175)*/,
        -2, -10, -2, -4/*mean (0.0188821), correlation (0.0985774)*/,
        -13, -13, -11, -8/*mean (0.0363135), correlation (0.0899616)*/,
        -13, -3, -12, -9/*mean (0.121806), correlation (0.099849)*/,
        10, 4, 11, 9/*mean (0.122065), correlation (0.093285)*/,
        -13, -8, -8, -9/*mean (0.162787), correlation (0.0942748)*/,
        -11, 7, -9, 12/*mean (0.21561), correlation (0.0974438)*/,
        7, 7, 12, 6/*mean (0.160583), correlation (0.130064)*/,
        -4, -5, -3, 0/*mean (0.228171), correlation (0.132998)*/,
        -13, 2, -12, -3/*mean (0.00997526), correlation (0.145926)*/,
        -9, 0, -7, 5/*mean (0.198234), correlation (0.143636)*/,
        12, -6, 12, -1/*mean (0.0676226), correlation (0.16689)*/,
        -3, 6, -2, 12/*mean (0.166847), correlation (0.171682)*/,
        -6, -13, -4, -8/*mean (0.101215), correlation (0.179716)*/,
        11, -13, 12, -8/*mean (0.200641), correlation (0.192279)*/,
        4, 7, 5, 1/*mean (0.205106), correlation (0.186848)*/,
        5, -3, 10, -3/*mean (0.234908), correlation (0.192319)*/,
        3, -7, 6, 12/*mean (0.0709964), correlation (0.210872)*/,
        -8, -7, -6, -2/*mean (0.0939834), correlation (0.212589)*/,
        -2, 11, -1, -10/*mean (0.127778), correlation (0.20866)*/,
        -13, 12, -8, 10/*mean (0.14783), correlation (0.206356)*/,
        -7, 3, -5, -3/*mean (0.182141), correlation (0.198942)*/,
        -4, 2, -3, 7/*mean (0.188237), correlation (0.21384)*/,
        -10, -12, -6, 11/*mean (0.14865), correlation (0.23571)*/,
        5, -12, 6, -7/*mean (0.222312), correlation (0.23324)*/,
        5, -6, 7, -1/*mean (0.229082), correlation (0.23389)*/,
        1, 0, 4, -5/*mean (0.241577), correlation (0.215286)*/,
        9, 11, 11, -13/*mean (0.00338507), correlation (0.251373)*/,
        4, 7, 4, 12/*mean (0.131005), correlation (0.257622)*/,
        2, -1, 4, 4/*mean (0.152755), correlation (0.255205)*/,
        -4, -12, -2, 7/*mean (0.182771), correlation (0.244867)*/,
        -8, -5, -7, -10/*mean (0.186898), correlation (0.23901)*/,
        4, 11, 9, 12/*mean (0.226226), correlation (0.258255)*/,
        0, -8, 1, -13/*mean (0.0897886), correlation (0.274827)*/,
        -13, -2, -8, 2/*mean (0.148774), correlation (0.28065)*/,
        -3, -2, -2, 3/*mean (0.153048), correlation (0.283063)*/,
        -6, 9, -4, -9/*mean (0.169523), correlation (0.278248)*/,
        8, 12, 10, 7/*mean (0.225337), correlation (0.282851)*/,
        0, 9, 1, 3/*mean (0.226687), correlation (0.278734)*/,
        7, -5, 11, -10/*mean (0.00693882), correlation (0.305161)*/,
        -13, -6, -11, 0/*mean (0.0227283), correlation (0.300181)*/,
        10, 7, 12, 1/*mean (0.125517), correlation (0.31089)*/,
        -6, -3, -6, 12/*mean (0.131748), correlation (0.312779)*/,
        10, -9, 12, -4/*mean (0.144827), correlation (0.292797)*/,
        -13, 8, -8, -12/*mean (0.149202), correlation (0.308918)*/,
        -13, 0, -8, -4/*mean (0.160909), correlation (0.310013)*/,
        3, 3, 7, 8/*mean (0.177755), correlation (0.309394)*/,
        5, 7, 10, -7/*mean (0.212337), correlation (0.310315)*/,
        -1, 7, 1, -12/*mean (0.214429), correlation (0.311933)*/,
        3, -10, 5, 6/*mean (0.235807), correlation (0.313104)*/,
        2, -4, 3, -10/*mean (0.00494827), correlation (0.344948)*/,
        -13, 0, -13, 5/*mean (0.0549145), correlation (0.344675)*/,
        -13, -7, -12, 12/*mean (0.103385), correlation (0.342715)*/,
        -13, 3, -11, 8/*mean (0.134222), correlation (0.322922)*/,
        -7, 12, -4, 7/*mean (0.153284), correlation (0.337061)*/,
        6, -10, 12, 8/*mean (0.154881), correlation (0.329257)*/,
        -9, -1, -7, -6/*mean (0.200967), correlation (0.33312)*/,
        -2, -5, 0, 12/*mean (0.201518), correlation (0.340635)*/,
        -12, 5, -7, 5/*mean (0.207805), correlation (0.335631)*/,
        3, -10, 8, -13/*mean (0.224438), correlation (0.34504)*/,
        -7, -7, -4, 5/*mean (0.239361), correlation (0.338053)*/,
        -3, -2, -1, -7/*mean (0.240744), correlation (0.344322)*/,
        2, 9, 5, -11/*mean (0.242949), correlation (0.34145)*/,
        -11, -13, -5, -13/*mean (0.244028), correlation (0.336861)*/,
        -1, 6, 0, -1/*mean (0.247571), correlation (0.343684)*/,
        5, -3, 5, 2/*mean (0.000697256), correlation (0.357265)*/,
        -4, -13, -4, 12/*mean (0.00213675), correlation (0.373827)*/,
        -9, -6, -9, 6/*mean (0.0126856), correlation (0.373938)*/,
        -12, -10, -8, -4/*mean (0.0152497), correlation (0.364237)*/,
        10, 2, 12, -3/*mean (0.0299933), correlation (0.345292)*/,
        7, 12, 12, 12/*mean (0.0307242), correlation (0.366299)*/,
        -7, -13, -6, 5/*mean (0.0534975), correlation (0.368357)*/,
        -4, 9, -3, 4/*mean (0.099865), correlation (0.372276)*/,
        7, -1, 12, 2/*mean (0.117083), correlation (0.364529)*/,
        -7, 6, -5, 1/*mean (0.126125), correlation (0.369606)*/,
        -13, 11, -12, 5/*mean (0.130364), correlation (0.358502)*/,
        -3, 7, -2, -6/*mean (0.131691), correlation (0.375531)*/,
        7, -8, 12, -7/*mean (0.160166), correlation (0.379508)*/,
        -13, -7, -11, -12/*mean (0.167848), correlation (0.353343)*/,
        1, -3, 12, 12/*mean (0.183378), correlation (0.371916)*/,
        2, -6, 3, 0/*mean (0.228711), correlation (0.371761)*/,
        -4, 3, -2, -13/*mean (0.247211), correlation (0.364063)*/,
        -1, -13, 1, 9/*mean (0.249325), correlation (0.378139)*/,
        7, 1, 8, -6/*mean (0.000652272), correlation (0.411682)*/,
        1, -1, 3, 12/*mean (0.00248538), correlation (0.392988)*/,
        9, 1, 12, 6/*mean (0.0206815), correlation (0.386106)*/,
        -1, -9, -1, 3/*mean (0.0364485), correlation (0.410752)*/,
        -13, -13, -10, 5/*mean (0.0376068), correlation (0.398374)*/,
        7, 7, 10, 12/*mean (0.0424202), correlation (0.405663)*/,
        12, -5, 12, 9/*mean (0.0942645), correlation (0.410422)*/,
        6, 3, 7, 11/*mean (0.1074), correlation (0.413224)*/,
        5, -13, 6, 10/*mean (0.109256), correlation (0.408646)*/,
        2, -12, 2, 3/*mean (0.131691), correlation (0.416076)*/,
        3, 8, 4, -6/*mean (0.165081), correlation (0.417569)*/,
        2, 6, 12, -13/*mean (0.171874), correlation (0.408471)*/,
        9, -12, 10, 3/*mean (0.175146), correlation (0.41296)*/,
        -8, 4, -7, 9/*mean (0.183682), correlation (0.402956)*/,
        -11, 12, -4, -6/*mean (0.184672), correlation (0.416125)*/,
        1, 12, 2, -8/*mean (0.191487), correlation (0.386696)*/,
        6, -9, 7, -4/*mean (0.192668), correlation (0.394771)*/,
        2, 3, 3, -2/*mean (0.200157), correlation (0.408303)*/,
        6, 3, 11, 0/*mean (0.204588), correlation (0.411762)*/,
        3, -3, 8, -8/*mean (0.205904), correlation (0.416294)*/,
        7, 8, 9, 3/*mean (0.213237), correlation (0.409306)*/,
        -11, -5, -6, -4/*mean (0.243444), correlation (0.395069)*/,
        -10, 11, -5, 10/*mean (0.247672), correlation (0.413392)*/,
        -5, -8, -3, 12/*mean (0.24774), correlation (0.411416)*/,
        -10, 5, -9, 0/*mean (0.00213675), correlation (0.454003)*/,
        8, -1, 12, -6/*mean (0.0293635), correlation (0.455368)*/,
        4, -6, 6, -11/*mean (0.0404971), correlation (0.457393)*/,
        -10, 12, -8, 7/*mean (0.0481107), correlation (0.448364)*/,
        4, -2, 6, 7/*mean (0.050641), correlation (0.455019)*/,
        -2, 0, -2, 12/*mean (0.0525978), correlation (0.44338)*/,
        -5, -8, -5, 2/*mean (0.0629667), correlation (0.457096)*/,
        7, -6, 10, 12/*mean (0.0653846), correlation (0.445623)*/,
        -9, -13, -8, -8/*mean (0.0858749), correlation (0.449789)*/,
        -5, -13, -5, -2/*mean (0.122402), correlation (0.450201)*/,
        8, -8, 9, -13/*mean (0.125416), correlation (0.453224)*/,
        -9, -11, -9, 0/*mean (0.130128), correlation (0.458724)*/,
        1, -8, 1, -2/*mean (0.132467), correlation (0.440133)*/,
        7, -4, 9, 1/*mean (0.132692), correlation (0.454)*/,
        -2, 1, -1, -4/*mean (0.135695), correlation (0.455739)*/,
        11, -6, 12, -11/*mean (0.142904), correlation (0.446114)*/,
        -12, -9, -6, 4/*mean (0.146165), correlation (0.451473)*/,
        3, 7, 7, 12/*mean (0.147627), correlation (0.456643)*/,
        5, 5, 10, 8/*mean (0.152901), correlation (0.455036)*/,
        0, -4, 2, 8/*mean (0.167083), correlation (0.459315)*/,
        -9, 12, -5, -13/*mean (0.173234), correlation (0.454706)*/,
        0, 7, 2, 12/*mean (0.18312), correlation (0.433855)*/,
        -1, 2, 1, 7/*mean (0.185504), correlation (0.443838)*/,
        5, 11, 7, -9/*mean (0.185706), correlation (0.451123)*/,
        3, 5, 6, -8/*mean (0.188968), correlation (0.455808)*/,
        -13, -4, -8, 9/*mean (0.191667), correlation (0.459128)*/,
        -5, 9, -3, -3/*mean (0.193196), correlation (0.458364)*/,
        -4, -7, -3, -12/*mean (0.196536), correlation (0.455782)*/,
        6, 5, 8, 0/*mean (0.1972), correlation (0.450481)*/,
        -7, 6, -6, 12/*mean (0.199438), correlation (0.458156)*/,
        -13, 6, -5, -2/*mean (0.211224), correlation (0.449548)*/,
        1, -10, 3, 10/*mean (0.211718), correlation (0.440606)*/,
        4, 1, 8, -4/*mean (0.213034), correlation (0.443177)*/,
        -2, -2, 2, -13/*mean (0.234334), correlation (0.455304)*/,
        2, -12, 12, 12/*mean (0.235684), correlation (0.443436)*/,
        -2, -13, 0, -6/*mean (0.237674), correlation (0.452525)*/,
        4, 1, 9, 3/*mean (0.23962), correlation (0.444824)*/,
        -6, -10, -3, -5/*mean (0.248459), correlation (0.439621)*/,
        -3, -13, -1, 1/*mean (0.249505), correlation (0.456666)*/,
        7, 5, 12, -11/*mean (0.00119208), correlation (0.495466)*/,
        4, -2, 5, -7/*mean (0.00372245), correlation (0.484214)*/,
        -13, 9, -9, -5/*mean (0.00741116), correlation (0.499854)*/,
        7, 1, 8, 6/*mean (0.0208952), correlation (0.499773)*/,
        7, -8, 7, 6/*mean (0.0220085), correlation (0.501609)*/,
        -7, -4, -7, 1/*mean (0.0233806), correlation (0.496568)*/,
        -8, 11, -7, -8/*mean (0.0236505), correlation (0.489719)*/,
        -13, 6, -12, -8/*mean (0.0268781), correlation (0.503487)*/,
        2, 4, 3, 9/*mean (0.0323324), correlation (0.501938)*/,
        10, -5, 12, 3/*mean (0.0399235), correlation (0.494029)*/,
        -6, -5, -6, 7/*mean (0.0420153), correlation (0.486579)*/,
        8, -3, 9, -8/*mean (0.0548021), correlation (0.484237)*/,
        2, -12, 2, 8/*mean (0.0616622), correlation (0.496642)*/,
        -11, -2, -10, 3/*mean (0.0627755), correlation (0.498563)*/,
        -12, -13, -7, -9/*mean (0.0829622), correlation (0.495491)*/,
        -11, 0, -10, -5/*mean (0.0843342), correlation (0.487146)*/,
        5, -3, 11, 8/*mean (0.0929937), correlation (0.502315)*/,
        -2, -13, -1, 12/*mean (0.113327), correlation (0.48941)*/,
        -1, -8, 0, 9/*mean (0.132119), correlation (0.467268)*/,
        -13, -11, -12, -5/*mean (0.136269), correlation (0.498771)*/,
        -10, -2, -10, 11/*mean (0.142173), correlation (0.498714)*/,
        -3, 9, -2, -13/*mean (0.144141), correlation (0.491973)*/,
        2, -3, 3, 2/*mean (0.14892), correlation (0.500782)*/,
        -9, -13, -4, 0/*mean (0.150371), correlation (0.498211)*/,
        -4, 6, -3, -10/*mean (0.152159), correlation (0.495547)*/,
        -4, 12, -2, -7/*mean (0.156152), correlation (0.496925)*/,
        -6, -11, -4, 9/*mean (0.15749), correlation (0.499222)*/,
        6, -3, 6, 11/*mean (0.159211), correlation (0.503821)*/,
        -13, 11, -5, 5/*mean (0.162427), correlation (0.501907)*/,
        11, 11, 12, 6/*mean (0.16652), correlation (0.497632)*/,
        7, -5, 12, -2/*mean (0.169141), correlation (0.484474)*/,
        -1, 12, 0, 7/*mean (0.169456), correlation (0.495339)*/,
        -4, -8, -3, -2/*mean (0.171457), correlation (0.487251)*/,
        -7, 1, -6, 7/*mean (0.175), correlation (0.500024)*/,
        -13, -12, -8, -13/*mean (0.175866), correlation (0.497523)*/,
        -7, -2, -6, -8/*mean (0.178273), correlation (0.501854)*/,
        -8, 5, -6, -9/*mean (0.181107), correlation (0.494888)*/,
        -5, -1, -4, 5/*mean (0.190227), correlation (0.482557)*/,
        -13, 7, -8, 10/*mean (0.196739), correlation (0.496503)*/,
        1, 5, 5, -13/*mean (0.19973), correlation (0.499759)*/,
        1, 0, 10, -13/*mean (0.204465), correlation (0.49873)*/,
        9, 12, 10, -1/*mean (0.209334), correlation (0.49063)*/,
        5, -8, 10, -9/*mean (0.211134), correlation (0.503011)*/,
        -1, 11, 1, -13/*mean (0.212), correlation (0.499414)*/,
        -9, -3, -6, 2/*mean (0.212168), correlation (0.480739)*/,
        -1, -10, 1, 12/*mean (0.212731), correlation (0.502523)*/,
        -13, 1, -8, -10/*mean (0.21327), correlation (0.489786)*/,
        8, -11, 10, -6/*mean (0.214159), correlation (0.488246)*/,
        2, -13, 3, -6/*mean (0.216993), correlation (0.50287)*/,
        7, -13, 12, -9/*mean (0.223639), correlation (0.470502)*/,
        -10, -10, -5, -7/*mean (0.224089), correlation (0.500852)*/,
        -10, -8, -8, -13/*mean (0.228666), correlation (0.502629)*/,
        4, -6, 8, 5/*mean (0.22906), correlation (0.498305)*/,
        3, 12, 8, -13/*mean (0.233378), correlation (0.503825)*/,
        -4, 2, -3, -3/*mean (0.234323), correlation (0.476692)*/,
        5, -13, 10, -12/*mean (0.236392), correlation (0.475462)*/,
        4, -13, 5, -1/*mean (0.236842), correlation (0.504132)*/,
        -9, 9, -4, 3/*mean (0.236977), correlation (0.497739)*/,
        0, 3, 3, -9/*mean (0.24314), correlation (0.499398)*/,
        -12, 1, -6, 1/*mean (0.243297), correlation (0.489447)*/,
        3, 2, 4, -8/*mean (0.00155196), correlation (0.553496)*/,
        -10, -10, -10, 9/*mean (0.00239541), correlation (0.54297)*/,
        8, -13, 12, 12/*mean (0.0034413), correlation (0.544361)*/,
        -8, -12, -6, -5/*mean (0.003565), correlation (0.551225)*/,
        2, 2, 3, 7/*mean (0.00835583), correlation (0.55285)*/,
        10, 6, 11, -8/*mean (0.00885065), correlation (0.540913)*/,
        6, 8, 8, -12/*mean (0.0101552), correlation (0.551085)*/,
        -7, 10, -6, 5/*mean (0.0102227), correlation (0.533635)*/,
        -3, -9, -3, 9/*mean (0.0110211), correlation (0.543121)*/,
        -1, -13, -1, 5/*mean (0.0113473), correlation (0.550173)*/,
        -3, -7, -3, 4/*mean (0.0140913), correlation (0.554774)*/,
        -8, -2, -8, 3/*mean (0.017049), correlation (0.55461)*/,
        4, 2, 12, 12/*mean (0.01778), correlation (0.546921)*/,
        2, -5, 3, 11/*mean (0.0224022), correlation (0.549667)*/,
        6, -9, 11, -13/*mean (0.029161), correlation (0.546295)*/,
        3, -1, 7, 12/*mean (0.0303081), correlation (0.548599)*/,
        11, -1, 12, 4/*mean (0.0355151), correlation (0.523943)*/,
        -3, 0, -3, 6/*mean (0.0417904), correlation (0.543395)*/,
        4, -11, 4, 12/*mean (0.0487292), correlation (0.542818)*/,
        2, -4, 2, 1/*mean (0.0575124), correlation (0.554888)*/,
        -10, -6, -8, 1/*mean (0.0594242), correlation (0.544026)*/,
        -13, 7, -11, 1/*mean (0.0597391), correlation (0.550524)*/,
        -13, 12, -11, -13/*mean (0.0608974), correlation (0.55383)*/,
        6, 0, 11, -13/*mean (0.065126), correlation (0.552006)*/,
        0, -1, 1, 4/*mean (0.074224), correlation (0.546372)*/,
        -13, 3, -9, -2/*mean (0.0808592), correlation (0.554875)*/,
        -9, 8, -6, -3/*mean (0.0883378), correlation (0.551178)*/,
        -13, -6, -8, -2/*mean (0.0901035), correlation (0.548446)*/,
        5, -9, 8, 10/*mean (0.0949843), correlation (0.554694)*/,
        2, 7, 3, -9/*mean (0.0994152), correlation (0.550979)*/,
        -1, -6, -1, -1/*mean (0.10045), correlation (0.552714)*/,
        9, 5, 11, -2/*mean (0.100686), correlation (0.552594)*/,
        11, -3, 12, -8/*mean (0.101091), correlation (0.532394)*/,
        3, 0, 3, 5/*mean (0.101147), correlation (0.525576)*/,
        -1, 4, 0, 10/*mean (0.105263), correlation (0.531498)*/,
        3, -6, 4, 5/*mean (0.110785), correlation (0.540491)*/,
        -13, 0, -10, 5/*mean (0.112798), correlation (0.536582)*/,
        5, 8, 12, 11/*mean (0.114181), correlation (0.555793)*/,
        8, 9, 9, -6/*mean (0.117431), correlation (0.553763)*/,
        7, -4, 8, -12/*mean (0.118522), correlation (0.553452)*/,
        -10, 4, -10, 9/*mean (0.12094), correlation (0.554785)*/,
        7, 3, 12, 4/*mean (0.122582), correlation (0.555825)*/,
        9, -7, 10, -2/*mean (0.124978), correlation (0.549846)*/,
        7, 0, 12, -2/*mean (0.127002), correlation (0.537452)*/,
        -1, -6, 0, -11/*mean (0.127148), correlation (0.547401)*/
    };

    // 灰度质心法计算特征点方向
    static float IC_Angle(const cv::Mat& image, cv::Point2f pt, const std::vector<int> & u_max)
    {
        int m_01 = 0, m_10 = 0;
        // 得到中心位置
        const uchar* center = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));

        // 对 v=0 这一行单独计算
        for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
            m_10 += u * center[u];

        // 这边要注意图像的step不一定是图像的宽度
        int step = (int)image.step1();

        for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
        {
            // 上下和左右两条线同时计算
            int v_sum = 0;
            int d = u_max[v];
            for (int u = -d; u <= d; ++u)
            {
                int val_plus = center[u + v*step], val_minus = center[u - v*step];
                v_sum += (val_plus - val_minus);//计算上下的时候是有符号的，所以这边是减
                m_10 += u * (val_plus + val_minus);//这边加是由于u已经确定好了符号
            }
            m_01 += v * v_sum;
        }

        return cv::fastAtan2((float)m_01, (float)m_10);
    }


    // compute the ORB descriptor
    const float factor_pi = (float)(CV_PI / 180.f);
    static void computeOrbDescriptor(const cv::KeyPoint& kpt,
                                     const cv::Mat& img, const cv::Point* pattern,
                                     uchar* desc)
    {
        float angle = (float)kpt.angle*factor_pi;
        auto a = (float)cos(angle), b = (float)sin(angle);

        const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
        const auto step = (int)img.step;
#define GET_VALUE(idx) \
        center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
               cvRound(pattern[idx].x*a - pattern[idx].y*b)]


        for (int i = 0; i < 32; ++i, pattern += 16)
        {
            int t0, t1, val;
            t0 = GET_VALUE(0); t1 = GET_VALUE(1);
            val = t0 < t1;
            t0 = GET_VALUE(2); t1 = GET_VALUE(3);
            val |= (t0 < t1) << 1;
            t0 = GET_VALUE(4); t1 = GET_VALUE(5);
            val |= (t0 < t1) << 2;
            t0 = GET_VALUE(6); t1 = GET_VALUE(7);
            val |= (t0 < t1) << 3;
            t0 = GET_VALUE(8); t1 = GET_VALUE(9);
            val |= (t0 < t1) << 4;
            t0 = GET_VALUE(10); t1 = GET_VALUE(11);
            val |= (t0 < t1) << 5;
            t0 = GET_VALUE(12); t1 = GET_VALUE(13);
            val |= (t0 < t1) << 6;
            t0 = GET_VALUE(14); t1 = GET_VALUE(15);
            val |= (t0 < t1) << 7;

            desc[i] = (uchar)val;
        }

#undef GET_VALUE
    }

    // calculate the keypoint orientation
    static void computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& umax)
    {
        for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
                     keypoint_end = keypoints.end(); keypoint != keypoint_end; ++keypoint)
        {
            keypoint->angle = IC_Angle(image, keypoint->pt, umax);
        }
    }

    // calculate this level descriptors for this level image
    static void computeLevelDescriptors(const cv::Mat& level_image,const vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                const vector<cv::Point>& pattern ) {
        descriptors = cv::Mat::zeros( (int)keypoints.size(), 32, CV_8UC1 );
        for( int i=0; i<keypoints.size(); i++ ) {
            computeOrbDescriptor(keypoints[i], level_image, &pattern[0], descriptors.ptr( i ) );
        }
    }
    void ExtractorNode::divideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
    {
        const int half_x = ceil(static_cast<float>(UR_.x - UL_.x) / 2);
        const int half_y = ceil(static_cast<float>(BR_.y - UL_.y) / 2);

        //定义节点边界
        n1.UL_ = UL_;
        n1.UR_ = cv::Point2i(UL_.x + half_x, UL_.y);
        n1.BL_ = cv::Point2i(UL_.x, UL_.y + half_y);
        n1.BR_ = cv::Point2i(UL_.x + half_x, UL_.y + half_y);
        n1.vec_keys_.reserve(vec_keys_.size());

        n2.UL_ = n1.UR_;
        n2.UR_ = UR_;
        n2.BL_ = n1.BR_;
        n2.BR_ = cv::Point2i(UR_.x, UL_.y + half_y);
        n2.vec_keys_.reserve(vec_keys_.size());

        n3.UL_ = n1.BL_;
        n3.UR_ = n1.BR_;
        n3.BL_ = BL_;
        n3.BR_ = cv::Point2i(n1.BR_.x, BL_.y);
        n3.vec_keys_.reserve(vec_keys_.size());

        n4.UL_ = n3.UR_;
        n4.UR_ = n2.BR_;
        n4.BL_ = n3.BR_;
        n4.BR_ = BR_;
        n4.vec_keys_.reserve(vec_keys_.size());

        //将特征点按象限进行划分
        for (size_t i = 0; i < vec_keys_.size(); i++)
        {
            const cv::KeyPoint &kp = vec_keys_[i];
            if (kp.pt.x < n1.UR_.x)
            {
                if (kp.pt.y < n1.BR_.y)
                    n1.vec_keys_.push_back(kp);
                else
                    n3.vec_keys_.push_back(kp);
            }
            else if (kp.pt.y < n1.BR_.y)
                n2.vec_keys_.push_back(kp);
            else
                n4.vec_keys_.push_back(kp);
        }

        if (n1.vec_keys_.size() == 1)
            n1.is_no_more_ = true;
        if (n2.vec_keys_.size() == 1)
            n2.is_no_more_ = true;
        if (n3.vec_keys_.size() == 1)
            n3.is_no_more_ = true;
        if (n4.vec_keys_.size() == 1)
            n4.is_no_more_ = true;

    }


    //Consructor with method
    ORBExtractor::ORBExtractor(const KeyPointMethod &method) {
        mMethod = method;

        // initial the ORB parameters from config file(GlobalCfongi)
        //   the GlobalConfig has been assigned when System contructor
        mnFeatures = G::nFeatures;
        mnLevel = G::nPyramidLevel;
        mfScaleFactor = G::fScaleFactor;
        mnDefaultFastTh = G::nInitTHFAST;
        mnMinFastTh = G::nMinTHFAST;

        // Initial pyramid

        // initial the pyramid scale factor vector
        mvecScaleFactor.resize(mnLevel);
        // set the level 0 scale factor to 1.0
        mvecScaleFactor[0] = 1.0f;
        // set other level by scale factor
        for (int i = 1; i < mnLevel; i++) {
            mvecScaleFactor[i] = mvecScaleFactor[i - 1] * mfScaleFactor;
        }

        // initial the image pyramid
        mvecImagePyramid.resize(mnLevel);
        // initial the vector of each level feature number
        mvecFeatureNumPerLevel.resize(mnLevel);
        float fScaleInv = 1.0f / mfScaleFactor;
        const float EPSINON = 0.000001;
        float x = 1 - (float) pow((double) fScaleInv, (double) mnLevel);
        float fDesiredFeaturesPerScale = mnFeatures / mnLevel;
        // if x != 0 , set the scale
        if (abs(x) > EPSINON) {
            fDesiredFeaturesPerScale = mnFeatures * (1 - fScaleInv) / x;
        }

        // set the features number of every level in pyramid
        int nSumFeatures = 0;
        for (int level = 0; level < mnLevel; level++) {
            mvecFeatureNumPerLevel[level] = cvRound(fDesiredFeaturesPerScale);
            nSumFeatures += mvecFeatureNumPerLevel[level];
            fDesiredFeaturesPerScale *= fScaleInv;
        }
        mvecFeatureNumPerLevel[mnLevel - 1] = std::max(mnFeatures - nSumFeatures, 0);

        //  复制训练的模板
        const int npoints = 512;
        const cv::Point *pattern0 = (const cv::Point *) bit_pattern_31_;
        std::copy(pattern0, pattern0 + npoints, std::back_inserter(mvecPattern));

        // the max u coordinate of per v when calculating the direction of feature
        mvecUMax.resize(HALF_PATCH_SIZE + 1);
        // 将v坐标划分为两部分进行计算，主要为了确保计算特征主方向的时候，x,y方向对称
        int v, v0;
        int vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.0f) / 2 + 1);
        int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.0f) / 2);
        // 通过勾股定理计算
        const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
        for (v = 0; v <= vmax; ++v) {
            mvecUMax[v] = cvRound(sqrt(hp2 - v * v));
        }

        // 确保对称，即保证是一个圆
        for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v) {
            while (mvecUMax[v0] == mvecUMax[v0 + 1]) {
                ++v0;
            }
            mvecUMax[v] = v0;
            ++v0;
        }
    }

    ORBExtractor::~ORBExtractor() {}

    // detect features and descriptors for image
    void ORBExtractor::detect(cv::InputArray image_array, vector<cv::KeyPoint>& output_keypoints,
                              cv::OutputArray descriptors_array )
    {
        if ( image_array.empty() )
            return;
        cv::Mat image = image_array.getMat();
        // gray image
        assert( image.type() == CV_8UC1);


        if ( mMethod == KeyPointMethod::QUAD_TREE_ORB) {
            std::vector<std::vector<cv::KeyPoint> > all_keypoints;
            // build the image pyramid
            computeImagePyramid(image);
            // compute keypoints with quad tree
            computeKeyPointsQuadTree(image, all_keypoints);
            // compute the descriptors for all keypoints
            computeDescriptors( all_keypoints, output_keypoints, descriptors_array  );



        } else if (mMethod == KeyPointMethod::OPENCV_GFTT) {
            // Todo
            LOG(WARNING) << "Todo : ORBExtractor::detect()::OPENCV_GFTT" << endl;

        } else if ( mMethod == KeyPointMethod::OPENCV_ORB ) {
            // Todo
            cv::Ptr<cv::ORB> detector = cv::ORB::create(mnFeatures,
                                                        G::fScaleFactor,
                                                        G::nPyramidLevel
            );
            detector->detectAndCompute( image, cv::Mat(), output_keypoints,descriptors_array );

        } else if ( mMethod == KeyPointMethod::OPENCV_SURF ) {
            // Todo
            LOG(WARNING) << "Todo : ORBExtractor::detect()::OPENCV_SURF" << endl;

        } else {
           LOG(FATAL) << "not right method " << endl;
        }
    }

    void ORBExtractor::computeKeyPointsQuadTree(const cv::Mat &image,
                                                vector<vector<cv::KeyPoint>> &allKeyPoints) {
        // Todo
        LOG(WARNING) << "Todo : ORBExtractor::computeKeyPointsQuadTree()" << endl;
        // resize the allpoints level to meet the pyramid level
        allKeyPoints.resize(mnLevel);

        // set the grid size
        const float border_width = 30;

        for (int level = 0; level < mnLevel; level++) {
            // 得到每一层图像进行特征检测区域上下两个坐标
            const int min_border_x = EDGE_THRESHOLD - 3;
            const int min_border_y = min_border_x;
            const int max_border_x = mvecImagePyramid[level].cols - EDGE_THRESHOLD + 3;
            const int max_border_y = mvecImagePyramid[level].rows - EDGE_THRESHOLD + 3;
            // 用于分配的关键点
            vector<cv::KeyPoint> vec_to_distribute_kps;
            vec_to_distribute_kps.reserve(mnFeatures * 10);

            const float width = (max_border_x - min_border_x);
            const float height = (max_border_y - min_border_y);
            // 将待检测区域划分为格子的行列数
            const int cols = width / border_width;
            const int rows = height / border_width;
            // 重新计算每个格子的大小
            const int width_cell = ceil(width / cols);
            const int height_cell = ceil(height / rows);
            // 在每个格子内进行fast特征检测
            for (int i = 0; i < rows; i++) {
                const float ini_y = min_border_y + i * height_cell;
                float max_y = ini_y + height_cell + 6;
                if (ini_y >= max_border_y - 3) {
                    continue;
                }
                if (max_y > max_border_y) {
                    max_y = max_border_y;
                }
                for (int j = 0; j < cols; j++) {
                    const float ini_x = min_border_x + j * width_cell;
                    float max_x = ini_x + width_cell + 6;
                    if (ini_x >= max_border_x - 6) {
                        continue;
                    }
                    if (max_x > max_border_x) {
                        max_x = max_border_x;
                    }

                    std::vector<cv::KeyPoint> vec_keys_cell;
                    // use the default FAST threshold to detect
                    cv::FAST(mvecImagePyramid[level]
                                     .rowRange(ini_y, max_y)
                                     .colRange(ini_x, max_x),
                             vec_keys_cell, mnDefaultFastTh, true);
                    // retry with min FASt threshold when default is unavailable
                    if (vec_keys_cell.empty()) {
                        cv::FAST(mvecImagePyramid[level]
                                         .rowRange(ini_y, max_y)
                                         .colRange(ini_x, max_x),
                                 vec_keys_cell, mnMinFastTh, true);
                    }
                    //  计算实际特征点的位置, and add to distribute keypoints
                    if (vec_keys_cell.empty()) {
                        continue;
                    }
                    for (auto &vit : vec_keys_cell) {
                        vit.pt.x += j * width_cell;
                        vit.pt.y += i * height_cell;
                        vec_to_distribute_kps.push_back(vit);
                    }
                } // end of for ( j )
            } // end of for ( i )

            //
            std::vector<cv::KeyPoint> &keypoints = allKeyPoints[level];
            keypoints.reserve(mnFeatures);
            // 将特征点进行四叉树划分
            keypoints = distributeQuadTree(vec_to_distribute_kps, min_border_x, max_border_x,
                                           min_border_y, max_border_y, mvecFeatureNumPerLevel[level], level);
            const int scaled_patch_size = PATCH_SIZE*mvecScaleFactor[level];

            // 换算特征点真实位置（添加边界值），添加特征点的尺度信息
            const int nkps = keypoints.size();
            for (int i = 0; i < nkps; i++)
            {
                keypoints[i].pt.x += min_border_x;
                keypoints[i].pt.y += min_border_y;
                keypoints[i].octave = level;
                keypoints[i].size = scaled_patch_size;
            }
        } // end of for ( level )

        // 计算特征点的方向
        for (int level = 0; level < mnLevel; ++level)
            computeOrientation(mvecImagePyramid[level], allKeyPoints[level], mvecUMax);

    }

    // OpenCV's good Fast detector
    void ORBExtractor::ComputeKeyPointsGFTT(cv::Mat image, vector<vector<Feature::Ptr>> &allKeyPoints) {
        // Todo
        LOG(WARNING) << "Todo : ORBExtractor::ComputeKeyPointsGFTT()" << endl;
    }

    // compute and build the image pyramid of each level
    void ORBExtractor::computeImagePyramid(cv::Mat image) {
        for (int level = 0; level < mnLevel; level++) {
            // get the scale factor of this level
            float scale = 1.0f / mvecScaleFactor[level];
            // resize the image with the scale factor
            int newCols = cvRound((float) image.cols * scale);
            int newRows = cvRound((float) image.rows * scale);
            cv::Size sz(newCols, newRows);
            if (level == 0) {
                // the first level pyramid is image itself
                mvecImagePyramid[level] = image;
            } else {
                // resize image from upper level image to this level
                //   with the scale factor size
                cv::resize(mvecImagePyramid[level - 1],
                           mvecImagePyramid[level],
                           sz, 0, 0, cv::INTER_LINEAR);
            }
        }
    }

    // quad tree distribute to key points
    vector<cv::KeyPoint> ORBExtractor::distributeQuadTree(const vector<cv::KeyPoint> &vec_to_distribute_keys,
                                                          const int &min_x, const int &max_x,
                                                          const int &min_y, const int &max_y,
                                                          const int &feature_num, const int &level)
    {
        // 计算初始时有几个节点
        const int init_node_num = round(static_cast<float>(max_x - min_x) / (max_y - min_y));
        // 得到节点之间的间隔
        const float interval_x = static_cast<float>(max_x - min_x) / init_node_num;

        std::vector<ExtractorNode*> init_nodes;
        init_nodes.resize(init_node_num);
        // 划分之后包含的节点
        std::list<ExtractorNode> list_nodes;
        for (int i = 0; i < init_node_num; i++)
        {
            ExtractorNode ni;
            ni.UL_ = cv::Point2i(interval_x*static_cast<float>(i), 0);
            ni.UR_ = cv::Point2i(interval_x*static_cast<float>(i + 1), 0);
            ni.BL_ = cv::Point2i(ni.UL_.x, max_y - min_y);
            ni.BR_ = cv::Point2i(ni.UR_.x, max_y - min_y);
            ni.vec_keys_.reserve(vec_to_distribute_keys.size());

            list_nodes.push_back(ni);
            init_nodes[i] = &list_nodes.back();
        }

        //将点分配给子节点
        for (size_t i = 0; i < vec_to_distribute_keys.size(); i++)
        {
            const cv::KeyPoint &kp = vec_to_distribute_keys[i];
            init_nodes[kp.pt.x / interval_x]->vec_keys_.push_back(kp);
        }

        std::list<ExtractorNode>::iterator lit = list_nodes.begin();

        while (lit != list_nodes.end())
        {
            // 如果只含一个特征点的时候，则不再划分
            if (lit->vec_keys_.size() == 1)
            {
                lit->is_no_more_ = true;
                lit++;
            }
            else if (lit->vec_keys_.empty())
                lit = list_nodes.erase(lit);
            else
                lit++;
        }

        bool is_finish = false;

        int iteration = 0;

        std::vector<std::pair<int, ExtractorNode*> > keys_size_and_node;//节点及对应包含的特征数
        keys_size_and_node.reserve(list_nodes.size() * 4);

        while (!is_finish)
        {
            iteration++;
            // 初始节点个数，用于判断是否节点再一次进行了划分
            int prev_size = list_nodes.size();

            lit = list_nodes.begin();
            // 表示节点分解次数
            int to_expand_num = 0;

            keys_size_and_node.clear();

            while (lit != list_nodes.end())
            {
                if (lit->is_no_more_)
                {
                    // 表面只有一个特征点，则不再划分
                    lit++;
                    continue;
                }
                else
                {
                    // 如果超过一个特征点，则继续划分
                    ExtractorNode n1, n2, n3, n4;
                    lit->divideNode(n1, n2, n3, n4);

                    // 对划分之后的节点进行判断，是否含有特征点，含有特征点则添加节点
                    if (n1.vec_keys_.size() > 0)
                    {
                        list_nodes.push_front(n1);
                        if (n1.vec_keys_.size() > 1)
                        {
                            to_expand_num++;
                            keys_size_and_node.push_back(std::make_pair(n1.vec_keys_.size(), &list_nodes.front()));
                            list_nodes.front().node_iter_ = list_nodes.begin();
                        }
                    }
                    if (n2.vec_keys_.size() > 0)
                    {
                        list_nodes.push_front(n2);
                        if (n2.vec_keys_.size() > 1)
                        {
                            to_expand_num++;
                            keys_size_and_node.push_back(std::make_pair(n2.vec_keys_.size(), &list_nodes.front()));
                            list_nodes.front().node_iter_ = list_nodes.begin();
                        }
                    }
                    if (n3.vec_keys_.size() > 0)
                    {
                        list_nodes.push_front(n3);
                        if (n3.vec_keys_.size() > 1)
                        {
                            to_expand_num++;
                            keys_size_and_node.push_back(std::make_pair(n3.vec_keys_.size(), &list_nodes.front()));
                            list_nodes.front().node_iter_ = list_nodes.begin();
                        }
                    }
                    if (n4.vec_keys_.size() > 0)
                    {
                        list_nodes.push_front(n4);
                        if (n4.vec_keys_.size() > 1)
                        {
                            to_expand_num++;
                            keys_size_and_node.push_back(std::make_pair(n4.vec_keys_.size(), &list_nodes.front()));
                            list_nodes.front().node_iter_ = list_nodes.begin();
                        }
                    }

                    lit = list_nodes.erase(lit);
                    continue;
                }
            }

            // 当节点个数大于需分配的特征数或者所有的节点只有一个特征点（节点不能划分）的时候，则结束。
            if ((int)list_nodes.size() >= feature_num || (int)list_nodes.size() == prev_size)
            {
                is_finish = true;
            }
            else if (((int)list_nodes.size() + to_expand_num * 3) > feature_num)//节点展开次数乘以3用于表明下一次的节点分解可能超过特征数，即为最后一次分解
            {
                while (!is_finish)
                {
                    prev_size = list_nodes.size();

                    std::vector<std::pair<int, ExtractorNode*> > prev_size_and_node = keys_size_and_node;
                    keys_size_and_node.clear();

                    sort(prev_size_and_node.begin(), prev_size_and_node.end());
                    for (int j = prev_size_and_node.size() - 1; j >= 0; j--)
                    {
                        ExtractorNode n1, n2, n3, n4;
                        prev_size_and_node[j].second->divideNode(n1, n2, n3, n4);

                        // 划分之后进一步的判断
                        if (n1.vec_keys_.size() > 0)
                        {
                            list_nodes.push_front(n1);
                            if (n1.vec_keys_.size() > 1)
                            {
                                keys_size_and_node.push_back(std::make_pair(n1.vec_keys_.size(), &list_nodes.front()));
                                list_nodes.front().node_iter_ = list_nodes.begin();
                            }
                        }
                        if (n2.vec_keys_.size() > 0)
                        {
                            list_nodes.push_front(n2);
                            if (n2.vec_keys_.size() > 1)
                            {
                                keys_size_and_node.push_back(std::make_pair(n2.vec_keys_.size(), &list_nodes.front()));
                                list_nodes.front().node_iter_ = list_nodes.begin();
                            }
                        }
                        if (n3.vec_keys_.size() > 0)
                        {
                            list_nodes.push_front(n3);
                            if (n3.vec_keys_.size() > 1)
                            {
                                keys_size_and_node.push_back(std::make_pair(n3.vec_keys_.size(), &list_nodes.front()));
                                list_nodes.front().node_iter_ = list_nodes.begin();
                            }
                        }
                        if (n4.vec_keys_.size() > 0)
                        {
                            list_nodes.push_front(n4);
                            if (n4.vec_keys_.size() > 1)
                            {
                                keys_size_and_node.push_back(std::make_pair(n4.vec_keys_.size(), &list_nodes.front()));
                                list_nodes.front().node_iter_ = list_nodes.begin();
                            }
                        }

                        list_nodes.erase(prev_size_and_node[j].second->node_iter_);

                        if ((int)list_nodes.size() >= feature_num)
                            break;
                    }

                    if ((int)list_nodes.size() >= feature_num || (int)list_nodes.size() == prev_size)
                        is_finish = true;

                }
            }
        }

        // 保留每个节点下最好的特征点
        std::vector<cv::KeyPoint> result_keys;
        result_keys.reserve(mnFeatures);
        for (std::list<ExtractorNode>::iterator lit = list_nodes.begin(); lit != list_nodes.end(); lit++)
        {
            std::vector<cv::KeyPoint> &node_keys = lit->vec_keys_;
            cv::KeyPoint* keypoint = &node_keys[0];
            float max_response = keypoint->response;

            for (size_t k = 1; k < node_keys.size(); k++)
            {
                if (node_keys[k].response > max_response)
                {
                    keypoint = &node_keys[k];
                    max_response = node_keys[k].response;
                }
            }

            result_keys.push_back(*keypoint);
        }

        return result_keys;
    }

    // compute descriptors for keypoints
    void ORBExtractor::computeDescriptors(vector<vector<cv::KeyPoint>>& all_keypoints,
                                          vector<cv::KeyPoint>& output_keypoints,
                                          cv::OutputArray descriptors_array){
        int keypoints_num = 0;
        // the descriptors for all keypoints
        cv::Mat descriptors;

        // total keypoints in pyramid
        for( int level=0; level < mnLevel; level++ ) {
            keypoints_num += all_keypoints[level].size();
        }
        if ( keypoints_num == 0 ) {
           descriptors_array.release();
        }else{
            descriptors_array.create(keypoints_num, 32, CV_8U );
            descriptors = descriptors_array.getMat();
        }


        // 存储描述子的偏移量，用于分割不同尺度层
        // the offset of each level descriptors
        //   to add different level descriptor together
        int offset = 0;
        for( int level = 0; level < mnLevel; level++ ) {
            vector<cv::KeyPoint>& keypoints = all_keypoints[level];
            //当前尺度下图像特征点的数目
            int kps_num_level = (int)keypoints.size();
            if( kps_num_level == 0 ) {
                continue;
            }
            // deal with current level image
            cv::Mat image_level = mvecImagePyramid[level].clone();
            GaussianBlur( image_level, image_level, cv::Size(7,7), 2,2 ,cv::BORDER_REFLECT_101);
            // compute descriptors
            cv::Mat desc = descriptors.rowRange( offset, offset + kps_num_level );
            computeLevelDescriptors( image_level, keypoints, desc, mvecPattern );

            offset += kps_num_level;

            // 特征点坐标进行尺度处理，换算到当前图像中
            if( level != 0 ) {
                float scale = mvecScaleFactor[level];
                for( auto &kp : keypoints ){
                    kp.pt *= scale;
                }
            }
            // 将完成好的特征点添加到输出vector中
            output_keypoints.insert(output_keypoints.end(), keypoints.begin(), keypoints.end());
        }
    }
}// end of namespace