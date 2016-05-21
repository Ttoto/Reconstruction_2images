#include"common.h"

using namespace std;
using namespace cv;

std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<CloudPoint> cpts) {
    std::vector<cv::Point3d> out;
    for (unsigned int i=0; i<cpts.size(); i++) {
        out.push_back(cpts[i].pt);
    }
    return out;
}


void KeyPointsToPoints(const vector<KeyPoint>& kps, vector<Point2f>& ps) {
    ps.clear();
    for (unsigned int i=0; i<kps.size(); i++) ps.push_back(kps[i].pt);
}


void GetAlignedPointsFromMatch(const std::vector<cv::KeyPoint>& imgpts1,
                               const std::vector<cv::KeyPoint>& imgpts2,
                               const std::vector<cv::DMatch>& matches,
                               std::vector<cv::KeyPoint>& pt_set1,
                               std::vector<cv::KeyPoint>& pt_set2)
{
    for (unsigned int i=0; i<matches.size(); i++) {
        assert(matches[i].queryIdx < imgpts1.size());
        pt_set1.push_back(imgpts1[matches[i].queryIdx]);
        assert(matches[i].trainIdx < imgpts2.size());
        pt_set2.push_back(imgpts2[matches[i].trainIdx]);
    }
}


void save_descriptors_to_file(const std::string& filename,
                              const std::vector<cv::KeyPoint>& imgpts,
                              const cv::Mat& descriptors)
{
    cv::FileStorage fs(filename, FileStorage::WRITE);
    write( fs , "keypoints", imgpts);
    write( fs , "Descriptor", descriptors);
    fs.release();
}

void restore_descriptors_from_file(const std::string& filename,
                                   std::vector<cv::KeyPoint>& imgpts,
                                   cv::Mat& descriptors)
{
    cv::FileStorage fs(filename, FileStorage::READ);
    FileNode kptFileNode = fs["keypoints"];
    read( kptFileNode, imgpts );
    fs["Descriptor"] >> descriptors;
    fs.release();
}
