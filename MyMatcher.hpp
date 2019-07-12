#ifndef MyMatcher_hpp
#define MyMatcher_hpp

#include "vo_features.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

class MyMatcher
{

    private:

    public:
        double GOOD_MATCH_RATIO;
        vector<DMatch> matches;

        MyMatcher()
        {
            GOOD_MATCH_RATIO = 0.3;
            /* initialize random seed: */
            srand (time(NULL));
        }

        Scalar randomColor()
        {
            return Scalar( rand() % 256, rand() % 256,rand() % 256);
        }

template <class T>
        void myDrawMatches(string windowName, Mat &i_prev, Mat &i_curr, vector<Point_<T> > &pts1, vector<Point_<T> > &pts2)
        {
            Mat i_matches;
            cv::hconcat(i_prev, i_curr, i_matches);
            cvtColor(i_matches,i_matches,CV_GRAY2BGR);

            for(int i = 0 ; i < pts1.size() ; i++)
            {
                arrowedLine(i_matches, pts1[i], Point2f(pts2[i]+i_prev.cols), randomColor(), 1, 8, 0, 0.1);
            }

            imshow(windowName, i_matches);
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
myDrawMatches("matches_track", i_prev, i_curr, pts1, pts2);

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
myDrawMatches("matches_fuse", i_prev, i_curr, pts1_final, pts2_final);



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
myDrawMatches("matches_KLT", i_prev, i_curr, pts1_final, pts2_final);

                    cout<<"After using KLT results, "<<pts1_final.size()<<" points are mathced with "<<pts2_final.size()<<" points."<<endl;


                    pts1 = pts1_final;
                    pts2 = pts2_final;
                    return pts1.size();

                }
            }
};


#endif
