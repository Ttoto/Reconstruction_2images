#ifndef FINDCAMERAMATRICES_H
#define FINDCAMERAMATRICES_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "common.h"

//#undef __SFM__DEBUG__
using namespace cv;


bool CheckCoherentRotation(cv::Mat_<double>& R);
//bool TestTriangulation(const std::vector<CloudPoint>& pcloud, const cv::Matx34d& P, std::vector<uchar>& status);

cv::Mat GetFundamentalMat(const std::vector<KeyPoint>& imgpts1,
                      const std::vector<KeyPoint>& imgpts2,
                      std::vector<KeyPoint>& imgpts1_good,
                      std::vector<KeyPoint>& imgpts2_good,
                      std::vector<DMatch>& matches
);

bool FindCameraMatrices(const Mat& K,
                        const Mat& Kinv,
                        const Mat& distcoeff,
                        const vector<KeyPoint>& imgpts1,
                        const vector<KeyPoint>& imgpts2,
                        vector<KeyPoint>& imgpts1_good,
                        vector<KeyPoint>& imgpts2_good,
                        Matx34d& P,
                        Matx34d& P1,
                        vector<DMatch>& matches,
                        vector<CloudPoint>& outCloud
                        );




#endif // FINDCAMERAMATRICES_H
