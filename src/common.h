#ifndef COMMON_H
#define COMMON_H


#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <iostream>
#include <list>
#include <set>


struct CloudPoint {
    cv::Point3d pt;
    std::vector<int> imgpt_for_img;
    double reprojection_error;
};

std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<CloudPoint> cpts);

void KeyPointsToPoints(const std::vector<cv::KeyPoint>& kps, std::vector<cv::Point2f>& ps);

void GetAlignedPointsFromMatch(const std::vector<cv::KeyPoint>& imgpts1,
                               const std::vector<cv::KeyPoint>& imgpts2,
                               const std::vector<cv::DMatch>& matches,
                               std::vector<cv::KeyPoint>& pt_set1,
                               std::vector<cv::KeyPoint>& pt_set2);


void save_descriptors_to_file(const std::string& filename,
                              const std::vector<cv::KeyPoint>& imgpts,
                              const cv::Mat& descriptors);

void restore_descriptors_from_file(const std::string& filename,
                                   std::vector<cv::KeyPoint>& imgpts,
                                   cv::Mat& descriptors);

#endif // COMMON_H
