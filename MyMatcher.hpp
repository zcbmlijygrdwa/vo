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

                vector<Point3f> points_3d_final;
                vector<KeyPoint> keypoints1_final, keypoints2_final;

                vector<DMatch> matches_final = matches;
                cv::Mat descriptors1_final(matches_final.size(),descriptors1.cols, CV_8U, cv::Scalar(0));
                cv::Mat descriptors2_final(matches_final.size(),descriptors2.cols, CV_8U, cv::Scalar(0));

                for(int i = 0 ; i < matches_final.size() ; i++)
                {
                    
                    keypoints1_final.push_back(keypoints1[matches_final[i].queryIdx]);
                    keypoints2_final.push_back(keypoints2[matches_final[i].trainIdx]);

                    pts1.push_back(keypoints1[matches_final[i].queryIdx].pt);
                    pts2.push_back(keypoints2[matches_final[i].trainIdx].pt);

                    descriptors1.row(matches_final[i].queryIdx).copyTo(descriptors1_final.row(i));
                    descriptors2.row(matches_final[i].trainIdx).copyTo(descriptors2_final.row(i));
                }

                keypoints1 = keypoints1_final;
                keypoints2 = keypoints2_final;
                descriptors1 = descriptors1_final;
                descriptors2 = descriptors2_final;


                //get new matches
                matcher -> match(descriptors1, descriptors2, matches_final, Mat());

                //draw matches
                Mat i_matches;
                drawMatches(i_prev,keypoints1,i_curr,keypoints2,matches_final,i_matches);
                imshow("matches_feature", i_matches);

                cout<<matches_final.size()<<" of common keypoints found between prev and curr."<<endl;


                return matches_final.size();
            }

        template <class T>
            int matchKeypointsSmallArea(vector<Point3_<T> > &points_3d, vector<Point_<T> > &pts1, vector<Point_<T> > &pts2, Mat &descriptors1, Mat &descriptors2, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, Mat &i_prev, Mat &i_curr)
            {

                //matching keypoints
                Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

                cout<<"before small area, descriptors1.rows = "<<descriptors1.rows<<endl;
                cout<<"before small area, descriptors2.rows = "<<descriptors2.rows<<endl;
                matcher -> match(descriptors1, descriptors2, matches, Mat());

                cout<<"matches.size() = "<<matches.size()<<endl;

                //sort matches by score
                std::sort(matches.begin(),matches.end());

                ////remove bad matches
                //int numGoodMatch = matches.size()*GOOD_MATCH_RATIO;
                //matches.erase(matches.begin()+numGoodMatch,matches.end());

                pts1.clear();
                pts2.clear();


                float smallAreaThres = 10;

                vector<Point3f> points_3d_final;
                vector<KeyPoint> keypoints1_final, keypoints2_final;

                vector<DMatch> matches_final;
                for(int i = 0 ; i < matches.size() ; i++)
                {

                    if(diff(keypoints1[matches[i].queryIdx].pt, keypoints2[matches[i].trainIdx].pt)<smallAreaThres)
                    {
                        matches_final.push_back(matches[i]);
                    }
                }

                cv::Mat descriptors1_final(matches_final.size(),descriptors1.cols, CV_8U, cv::Scalar(0));
                cv::Mat descriptors2_final(matches_final.size(),descriptors2.cols, CV_8U, cv::Scalar(0));

                for(int i = 0 ; i < matches_final.size() ; i++)
                {

                    points_3d_final.push_back(points_3d[matches_final[i].queryIdx]);
                    
                    keypoints1_final.push_back(keypoints1[matches_final[i].queryIdx]);
                    keypoints2_final.push_back(keypoints2[matches_final[i].trainIdx]);

                    pts1.push_back(keypoints1[matches_final[i].queryIdx].pt);
                    pts2.push_back(keypoints2[matches_final[i].trainIdx].pt);

                    descriptors1.row(matches_final[i].queryIdx).copyTo(descriptors1_final.row(i));
                    descriptors2.row(matches_final[i].trainIdx).copyTo(descriptors2_final.row(i));

                    //cout<<"Pts1 added point: "<<keypoints1[matches[i].queryIdx].pt<<endl;
                    //cout<<"Pts2 added point: "<<keypoints2[matches[i].trainIdx].pt<<endl;
                    //cout<<"-------"<<endl;
                }

                keypoints1 = keypoints1_final;
                keypoints2 = keypoints2_final;
                descriptors1 = descriptors1_final;
                descriptors2 = descriptors2_final;
                points_3d = points_3d_final;


                //get new matches
                matcher -> match(descriptors1, descriptors2, matches_final, Mat());

                cout<<"with small area, match size reduced from "<<matches.size()<<" to "<<matches_final.size()<<endl;

                cout<<"after small area, keypoints1.size() = "<<keypoints1.size()<<endl;
                cout<<"after small area, keypoints2.size() = "<<keypoints2.size()<<endl;
                cout<<"after small area, descriptors1.rows = "<<descriptors1.rows<<endl;
                cout<<"after small area, descriptors2.rows = "<<descriptors2.rows<<endl;
                cout<<"after small area, matches_final.size() = "<<matches_final.size()<<endl;


                //draw matches
                Mat i_matches;
                drawMatches(i_prev,keypoints1,i_curr,keypoints2,matches_final,i_matches);
                imshow("matches_feature", i_matches);

                cout<<matches.size()<<" of common keypoints found between prev and curr."<<endl;


                return matches.size();
            }

        template <class T>
            int matchKeypointsWithKLT(vector<Point3_<T> > &points_3d, vector<Point_<T> > &pts1, vector<Point_<T> > &pts2, Mat &descriptors1, Mat &descriptors2, vector<KeyPoint> &keypoints1, vector<KeyPoint> &keypoints2, Mat &i_prev, Mat &i_curr)
            {
                vector<uchar> status;

                //use optical flow
                vector<Point2f> pts2_optiflow;
                featureTracking(i_prev, i_curr, pts1, pts2_optiflow, status);

                cout<<"status.size() = "<<status.size()<<endl;

                vector<Point2f> pts1_final, pts2_final;
                vector<Point3f> points_3d_final;

                int trackedNumber = 0;

                for(int i = 0 ; i < status.size() ; i++)
                {
                    cout<<int(status[i])<<",";
                    if(int(status[i]==1))
                    {
                        trackedNumber++;
                        pts1_final.push_back(pts1[i]);
                        pts2_final.push_back(pts2[i]);
                        points_3d_final.push_back(points_3d[i]);

                    }
                }
                cout<<endl;

                cout<<trackedNumber<<" of points are tracked between prev and curr."<<endl;



                //further refine matches based on tracked results
                //???????
                //float thres = 1;
                //for(int i = 0 ; i < pts1.size(); i++)
                //{
                //    if(diff(pts2[i] , pts2_optiflow[i])<thres)
                //    {
                //        pts1_final.push_back(pts1[i]);
                //        pts2_final.push_back(pts2[i]);
                //        points_3d_final.push_back(points_3d[i]);
                //    }
                //}

                //cout<<"After fusing feature matching and KLT results, "<<pts1_final.size()<<" points are mathced with "<<pts2_final.size()<<" points."<<endl;




                pts1 = pts1_final;
                pts2 = pts2_final;
                points_3d = points_3d_final;

                //draw tracked matches
                myDrawMatches("matches_KLT", i_prev, i_curr, pts1_final, pts2_final);



                return pts1.size();
            }

};


#endif
