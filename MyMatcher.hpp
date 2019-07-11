#ifndef MyMatcher_hpp
#define MyMatcher_hpp

#include "vo_features.h"
class MyMatcher
{

    private:

    public:
        double GOOD_MATCH_RATIO;
        vector<DMatch> matches;

        MyMatcher()
        {
            GOOD_MATCH_RATIO = 0.3;

        }


        template <class T>
            int matchKeypoints(vector<Point_<T> > &pts1, vector<Point_<T> > &pts2, Mat &descriptors1, Mat &descriptors2, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, Mat &i_prev, Mat &i_curr)
            {
                //matching keypoints
                Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

                matcher -> match(descriptors1, descriptors2, matches, Mat());

                //sort matches by score
                std::sort(matches.begin(),matches.end());

                //remove bad matches
                int numGoodMatch = matches.size()*GOOD_MATCH_RATIO;
                matches.erase(matches.begin()+numGoodMatch,matches.end());

                pts1.clear();
                pts2.clear();
                for(int i = 0 ; i < matches.size() ; i++)
                {
                    pts1.push_back(keypoints1[matches[i].queryIdx].pt);
                    pts2.push_back(keypoints2[matches[i].trainIdx].pt);

                    //cout<<"Pts1 added point: "<<keypoints1[matches[i].queryIdx].pt<<endl;
                    //cout<<"Pts2 added point: "<<keypoints2[matches[i].trainIdx].pt<<endl;
                    //cout<<"-------"<<endl;
                }


                //draw matches
                Mat i_matches;
                drawMatches(i_prev,keypoints1,i_curr,keypoints2,matches,i_matches);
                imshow("matches_feature", i_matches);

                cout<<matches.size()<<" of common keypoints found between prev and curr."<<endl;


                return matches.size();
            }

        template <class T>
            int matchKeypointsWithKLTFusion(vector<Point_<T> > &pts1, vector<Point_<T> > &pts2, Mat &descriptors1, Mat &descriptors2, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, Mat &i_prev, Mat &i_curr)
            {
                vector<uchar> status;


                //matching keypoints
                matchKeypoints(pts1, pts2, descriptors1, descriptors2, keypoints1, keypoints2, i_prev, i_curr);

                //use optical flow
                vector<Point2f> pts2_optiflow;
                featureTracking(i_prev, i_curr, pts1, pts2_optiflow, status);

                for(int i = 0 ; i < status.size() ; i++)
                {
                    cout<<int(status[i])<<",";
                }
                cout<<endl;

                //draw tracked matches
                Mat i_matches_track;
                cv::hconcat(i_prev, i_curr, i_matches_track);
                cvtColor(i_matches_track,i_matches_track,CV_GRAY2BGR);


                for(int i = 0 ; i < pts1.size() ; i++)
                {
                    arrowedLine(i_matches_track, pts1[i], Point2f(pts2_optiflow[i]+i_prev.cols), Scalar( 255, 0, 0), 1, 8, 0, 0.1);
                }
                imshow("matches_track", i_matches_track);

                cout<<matches.size()<<" of points are tracked between prev and curr."<<endl;



                //further refine matches based on tracked results
                float thres = 1;
                vector<Point2f> pts1_final, pts2_final;
                for(int i = 0 ; i < pts1.size(); i++)
                {
                    if(diff(pts2[i] , pts2_optiflow[i])<thres)
                    {
                        pts1_final.push_back(pts1[i]);
                        pts2_final.push_back(pts2[i]);
                    }
                }

                //draw tracked matches
                Mat i_matches_fuse;
                cv::hconcat(i_prev, i_curr, i_matches_fuse);
                cvtColor(i_matches_fuse,i_matches_fuse,CV_GRAY2BGR);

                for(int i = 0 ; i < pts1_final.size() ; i++)
                {
                    arrowedLine(i_matches_fuse, pts1_final[i], Point2f(pts2_final[i]+i_prev.cols), Scalar( 255, 0, 0), 1, 8, 0, 0.1);
                }
                imshow("matches_fuse", i_matches_fuse);

                cout<<"After fusing feature matching and KLT results, "<<pts1_final.size()<<" points are mathced with "<<pts2_final.size()<<" points."<<endl;


                pts1 = pts1_final;
                pts2 = pts2_final;
                return pts1.size();
            }

        template <class T>
            int KLTthenFeature(vector<Point_<T> > &pts1, vector<Point_<T> > &pts2, Mat &descriptors1, Mat &descriptors2, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, Mat &i_prev, Mat &i_curr)
            {
                vector<uchar> status;

                //matching keypoints
                //matchKeypoints(pts1, pts2, descriptors1, descriptors2, keypoints1, keypoints2, i_prev, i_curr);
g

                //check if pts1 is empty
                    cout<<"pts1 has size of "<<pts1.size()<<endl; 
                if(pts1.size()==0)
                {
                    //need to init with feature points
                    cout<<"pts1 is empty, init with feature and KLT fusion method"<<endl;
                    matchKeypointsWithKLTFusion(pts1, pts2, descriptors1, descriptors2, keypoints1, keypoints2, i_prev, i_curr);
                    pts2.clear();
                    cout<<"pts1 is inited with size of "<<pts1.size()<<endl; 
                }
                else
                {
                    // no need to do anything...
                }

                //use optical flow
                vector<Point2f> pts2_optiflow;
                featureTracking(i_prev, i_curr, pts1, pts2_optiflow, status);

                //use tracked results as matches
                vector<Point2f> pts1_final, pts2_final;
                for(int i = 0 ; i < status.size() ; i++)
                {
                    cout<<int(status[i])<<",";
                    if(int(status[i]==1))
                    {
                        pts1_final.push_back(pts1[i]);
                        pts2_final.push_back(pts2_optiflow[i]);
                    }
                }
                cout<<endl;
                if(pts1_final.size() < 20)
                {
                    //need to add new feature points
                    cout<<"Insufficient tracked points, need adding more feature points."<<endl;
                   // 
                }
                else
                {

                    //draw tracked matches
                    Mat i_matches_fuse;
                    cv::hconcat(i_prev, i_curr, i_matches_fuse);
                    cvtColor(i_matches_fuse,i_matches_fuse,CV_GRAY2BGR);

                    for(int i = 0 ; i < pts1_final.size() ; i++)
                    {
                        arrowedLine(i_matches_fuse, pts1_final[i], Point2f(pts2_final[i]+i_prev.cols), Scalar( 255, 0, 0), 1, 8, 0, 0.1);
                    }
                    imshow("matches_fuse", i_matches_fuse);

                    cout<<"After using KLT results, "<<pts1_final.size()<<" points are mathced with "<<pts2_final.size()<<" points."<<endl;


                    pts1 = pts1_final;
                    pts2 = pts2_final;
                    return pts1.size();

                }
            }
};


#endif
