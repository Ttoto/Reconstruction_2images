#ifndef MATCHING_H
#define MATCHING_H

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <iostream>
#include <set>

using namespace std;
using namespace cv;

//class Matching
//{
//public:
//    Matching(const Mat& img_1, const Mat& img_1_orig,
//             const Mat& img_2, const Mat& img_2_orig,
//             const vector<KeyPoint>& imgpts1,
//             const vector<KeyPoint>& imgpts2,
//             const Mat& descriptors_1,
//             const Mat& descriptors_2,
//             vector<KeyPoint>& fullpts1,
//             vector<KeyPoint>& fullpts2,
//             vector<DMatch>* matches);
//};

void matching_feature_detector(const Mat& img1,
                               const Mat& img2,
                               vector<KeyPoint>& keypoint1,
                               vector<KeyPoint>& keypoint2
                               );
void matching_descriptor_extractor(const Mat& img1,
                                   const Mat& img2,
                                   vector<KeyPoint>& keypoint1,
                                   vector<KeyPoint>& keypoint2,
                                   Mat& descriptors_1,
                                   Mat& descriptors_2);
void matching_fb_matcher(const Mat& descriptors_1,
                       const Mat& descriptors_2,
                       vector<DMatch>& matches);

void matching_good_matching_filter( vector<DMatch>& matches,
                                    vector<DMatch>& good_matches,
                                    const vector<KeyPoint> keypoints_1,
                                    const vector<KeyPoint> keypoints_2,
                                    vector<KeyPoint> good_keypoints_1,
                                    vector<KeyPoint> good_keypoints_2);
//void matching_



#endif // MATCHING_H
