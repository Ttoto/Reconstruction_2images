#include "matching.h"


void matching_get_feature_descriptors(const Mat& img,
                                      vector<KeyPoint>& keypoint,
                                      Mat& descriptor)
{

    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> extractor;

    detector = FeatureDetector::create("SIFT");
    extractor = DescriptorExtractor::create("SIFT");

    detector->detect( img, keypoint );
    extractor->compute(img,keypoint,descriptor);
}



//FlannBasedMatcher matcher;
void matching_fb_matcher(const Mat& descriptors_1,
                         const Mat& descriptors_2,
                         vector<DMatch>& matches)
{
    BFMatcher matcher(NORM_L2,true);
    matcher.match( descriptors_1, descriptors_2, matches );
}

void matching_good_matching_filter(vector<DMatch>& matches)
{
    vector<DMatch> good_matches;
    int sizebf = matches.size();//size of matches before filter

    cout<<"Filtering good matches"<<endl;

    double max_dist = 0; double min_dist = 1000.0;
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

    //double cutoff = min_dist+(max_dist-min_dist)*0.2;
    double cutoff = min_dist*4;

    for(unsigned int i = 0; i < matches.size(); i++ )
    {
        if(matches[i].distance > 0.0 && matches[i].distance < cutoff )
        {
            good_matches.push_back( matches[i]);
        }
    }

    matches.clear();
    matches = good_matches;

    int sizeexist = matches.size();

    cout << "matching point:" << sizebf <<"---->" << sizeexist << "  "<< (int)(((float)sizeexist/sizebf)*100) << "% exist after filting"<< endl;
}

