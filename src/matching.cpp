#include "matching.h"

#define __SFM__DEBUG__


/* ------------------------------------------------------------------------- */
/** \fn void matching_feature_detector(const Mat& img1,
                               const Mat& img2,
                               vector<KeyPoint>& keypoint1,
                               vector<KeyPoint>& keypoint2
                               )
*
* \brief detect the feature of two image
*
*
*
* \param[in]
img1 -> first image mat
img2 -> decond image mat
* \param[out]
keypoint1 -> the keypoint of the img1
keypoint2 -> the keypoint of the img2
*
* \return
Diese Funktion gibt 0 zurück wenn alles ok sonst 1
*/
/* ------------------------------------------------------------------------- */
void matching_feature_detector(const Mat& img1,
                               const Mat& img2,
                               vector<KeyPoint>& keypoint1,
                               vector<KeyPoint>& keypoint2
                               )
{
    //-- Step 1: Detect the keypoints using SURF Detector


    /*The minHessian is a threshold to decide from which value you are willing to accept keypoints.
      In practice, the higher the minHessian, the fewer keypoints you will obtain, but you expect
      them to be more repetitive (w.r.t. image transformations), and then, more useful.
      On the other hand, the lower the minHessian, the more keypoints you get, but they may be more noisy.
    */
    int minHessian = 100;
    SurfFeatureDetector detector( minHessian );
    detector.detect( img1, keypoint1 );
    detector.detect( img2, keypoint2 );
}




/* ------------------------------------------------------------------------- */
/** \fn void matching_descriptor_extractor(const Mat& img1,
                                   const Mat& img2,
                                   vector<KeyPoint>& keypoint1,
                                   vector<KeyPoint>& keypoint2,
                                   Mat& descriptors_1,
                                   Mat& descriptors_2)
*
* \brief Calculate descriptors
*
* \param[in]
img1 -> first image mat
img2 -> decond image mat
* \param[out]
keypoint1 -> the keypoint of the img1
keypoint2 -> the keypoint of the img2
descriptors_1 -> the descriptors of the img1
descriptors_2 -> the descriptors of the img2
*
* \return
Diese Funktion gibt 0 zurück wenn alles ok sonst 1
*/
/* ------------------------------------------------------------------------- */
void matching_descriptor_extractor(const Mat& img1,
                                   const Mat& img2,
                                   vector<KeyPoint>& keypoint1,
                                   vector<KeyPoint>& keypoint2,
                                   Mat& descriptors_1,
                                   Mat& descriptors_2)
{

    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("SIFT");
    //    SiftDescriptorExtractor extractor(48,16,true);
    //    SurfDescriptorExtractor extractor(8,4,true);
    //			extractor.compute( img_1, keypoints_1, desc );
    //			desc.copyTo(descriptors_1);

    extractor->compute(img1,keypoint1,descriptors_1);
    extractor->compute(img2,keypoint2,descriptors_2);
}


//-- Step 3: Matching descriptor vectors using FLANN matcher
//FlannBasedMatcher matcher;
void matching_fb_matcher(const Mat& descriptors_1,
                         const Mat& descriptors_2,
                         vector<DMatch>& matches)
{
    cout << "Step 3: Matching descriptor vectors using FLANN matcher-->";
    BFMatcher matcher(NORM_L2,true);
    matcher.match( descriptors_1, descriptors_2, matches );
    cout << "Finished" << endl;
    cout << "matches->size() " << matches.size() << endl;
}

void matching_good_matching_filter( vector<DMatch>& matches,
                                    vector<DMatch>& good_matches,
                                    const vector<KeyPoint> keypoints_1,
                                    const vector<KeyPoint> keypoints_2,
                                    vector<KeyPoint> good_keypoints_1,
                                    vector<KeyPoint> good_keypoints_2)
{
    cout<<"Running good matching filter"<<endl;
    double max_dist = 0; double min_dist = 1000.0;
    //-- Quick calculation of max and min distances between keypoints
    for(unsigned int i = 0; i < matches.size(); i++ )
    {
        double dist = (matches)[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    if (min_dist <= 0) {
        min_dist = 10.0;
    }
    cout<<"-- Max dist : "<<max_dist<<endl;
    cout<<"-- Min dist : "<<min_dist<<endl;

    //matcher.match( descriptors_1, descriptors_2, matches );
    //               queryIdx       trainIdx

    //    vector<KeyPoint> imgpts1_good,imgpts2_good;

    //double cutoff = min_dist+(max_dist-min_dist)*0.2;
    double cutoff = min_dist*4;
    std::set<int> existing_trainIdx;
    for(unsigned int i = 0; i < matches.size(); i++ )
    {
        if(matches[i].distance > 0.0 && matches[i].distance < cutoff )
        {
            good_matches.push_back( matches[i]);
            good_keypoints_1.push_back(keypoints_1[matches[i].queryIdx]);
            good_keypoints_2.push_back(keypoints_2[matches[i].trainIdx]);
            existing_trainIdx.insert(matches[i].trainIdx);
        }
    }
    cout << "good_keypoints_1.size"<<good_keypoints_1.size()<<endl;
    cout << "matching point:" << matches.size() <<"---->" << good_matches.size() << endl;
}

