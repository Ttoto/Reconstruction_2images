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

void matching_get_feature_descriptors(const Mat& img,
                                      vector<KeyPoint>& keypoint,
                                      Mat& descriptor);

void matching_fb_matcher(const Mat& descriptors_1,
                         const Mat& descriptors_2,
                         vector<DMatch>& matches);

void matching_good_matching_filter( vector<DMatch>& matches,
                                    vector<KeyPoint> imgpts1,
                                    vector<KeyPoint> imgpts2);

//void matching_feature_detector(const Mat& img1,
//                               const Mat& img2,
//                               vector<KeyPoint>& keypoint1,
//                               vector<KeyPoint>& keypoint2
//                               );
//void matching_descriptor_extractor(const Mat& img1,
//                                   const Mat& img2,
//                                   vector<KeyPoint>& keypoint1,
//                                   vector<KeyPoint>& keypoint2,
//                                   Mat& descriptors_1,
//                                   Mat& descriptors_2);


#endif // MATCHING_H
