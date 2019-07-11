

#include <iostream>
#include <opencv2/opencv.hpp>

#include <opencv2/features2d.hpp>
#include "vo_features.h"

using namespace std;

using namespace cv;


Point2f operator+(Point2f p1, int i)
{
    p1.x += i;
    return p1;
}

    template <class T>
T diff(Point_<T> p1, Point_<T> p2)
{
    return abs(p1.x - p2.x) + abs(p1.y - p2.y);
}




int main(int argc, char** argv)
{
    cout<<"VO main program!"<<endl;

    Mat R_f, t_f; //the final rotation and tranlation vectors containing the 

    vector<uchar> status;

    float scale = 1;

    VideoCapture cap("../dash.mp4");

    Mat i_prev, i_curr;

    int MAX_FEATURES = 500;
    double GOOD_MATCH_RATIO = 0.3;

    Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);

    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;


    bool isSlamInit = false;
    double k1 = -0.0000004;
    double k2 = 0;
    double p1 = 0;
    double p2 = 0;
    Mat distCoeffs = (Mat_<double>(1,4)<<k1, k2, p1, p2);
    cout<<"distCoeffs = "<<distCoeffs<<endl;


    if(!cap.isOpened())
    {
        cout<<"failed to open video."<<endl;
    }


    cap >> i_prev;
    cvtColor(i_prev,i_prev,CV_BGR2GRAY);

    Point2d pp = Point2d(i_prev.cols/2., i_prev.rows/2);

    double focal_x = 1;
    double focal_y = 1;

    Mat camera_matrix = (Mat_<double>(3,3)<<focal_x, 0, pp.x, 0, focal_y, pp.y, 0, 0, 1);
    cout<<"camera_matrix = "<<camera_matrix<<endl;

    int count = 0;

    Mat newCameraMatrix;
    Mat i_undistort;
    //undistort the prev frame
    undistort(i_prev, i_undistort, camera_matrix, distCoeffs, newCameraMatrix);
    i_prev = i_undistort.clone();
    while(!i_prev.empty())
    {
        // capture new frame
        cap >> i_curr;
        //undistort the new frame
        undistort(i_curr, i_undistort, camera_matrix, distCoeffs, newCameraMatrix);
        i_curr = i_undistort.clone();

        count++;
        if(count%5!=1)
        {
            //continue;
        }
        cvtColor(i_curr,i_curr,CV_BGR2GRAY);

        //extract orb features


        orb->detectAndCompute(i_prev,Mat(),keypoints1, descriptors1);
        orb->detectAndCompute(i_curr,Mat(),keypoints2, descriptors2);


        Mat i_prev_kp, i_curr_kp;

        drawKeypoints(i_prev, keypoints1, i_prev_kp, Scalar::all(-1),DrawMatchesFlags::DEFAULT);
        drawKeypoints(i_curr, keypoints2, i_curr_kp, Scalar::all(-1),DrawMatchesFlags::DEFAULT);

        //imshow("prev keypoints",i_prev_kp);
        //imshow("curr keypoints",i_curr_kp);


        //matching keypoints
        vector<DMatch> matches;

        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

        matcher -> match(descriptors1, descriptors2, matches, Mat());

        //sort matches by score
        std::sort(matches.begin(),matches.end());

        //remove bad matches
        int numGoodMatch = matches.size()*GOOD_MATCH_RATIO;        
        matches.erase(matches.begin()+numGoodMatch,matches.end());

        //draw matches
        Mat i_matches;
        drawMatches(i_prev,keypoints1,i_curr,keypoints2,matches,i_matches);
        imshow("matches_feature", i_matches);

        cout<<matches.size()<<" of common keypoints found between prev and curr."<<endl;




        ////refine matching by Ransac + homogeneous matrix
        //int ransacTime = 100;
        //for(int i = 0 ; i < ransacTime ; i++)
        //{
        //    select 3 points
        //}



        //check if slam is init
        if(!isSlamInit)
        {
            //use epipolar f=geometry to solve for fundamental matrix
            Mat fundamental_matrix;
            vector<Point2f> pts1, pts2;
            for(int i = 0 ; i < matches.size() ; i++)
            {
                pts1.push_back(keypoints1[matches[i].queryIdx].pt);
                pts2.push_back(keypoints2[matches[i].trainIdx].pt);

                //cout<<"Pts1 added point: "<<keypoints1[matches[i].queryIdx].pt<<endl;
                //cout<<"Pts2 added point: "<<keypoints2[matches[i].trainIdx].pt<<endl;
                //cout<<"-------"<<endl;
            }

            cout<<"pts1 size = "<<pts1.size()<<endl;
            cout<<"pts2 size = "<<pts2.size()<<endl;



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



            //find essental matrix for pose recovery
            Mat essential_matrix;
            Mat mask;
            essential_matrix = findEssentialMat(pts1,pts2,focal_x, pp, RANSAC, 0.99999999999, 1.0, mask);

            cout << "Essential matrix found: " << essential_matrix << endl;
            if(essential_matrix.empty())
            {
                cout<<"No Essential matrix found!"<<endl;
            }
            else
            {

                //recover pose from Essential matrix
                int numInliers = 0;
                Mat R, t;
                numInliers = recoverPose(essential_matrix, pts1, pts2, R, t, focal_x, pp, mask);

                cout<<"numInliers = "<<numInliers<<endl;

                if(count==1)
                {
                    R_f = R.clone();
                    t_f = t.clone();
                    cout<<"R_f t_f init!"<<endl;
                }

                //convert rotation matrix to rodrigues vector
                Mat jacobian;
                Mat rodrigues;
                Rodrigues(R, rodrigues, jacobian);

                cout<<"Pose recovered, "<<endl<<"rodrigues = "<<rodrigues<<endl<<"t = "<<t<<endl;


                t_f = t_f + scale*(R_f*t);
                R_f = R*R_f;

                cout<<"Coordinates: x = "<<t_f.at<double>(0)<<" y = "<<t_f.at<double>(1)<<" z = "<<t_f.at<double>(2)<<endl;
            }
        }
        else
        {

        }


        i_prev = i_curr.clone();

        waitKey(0);
    }


    cap.release();

    return 0;
}
