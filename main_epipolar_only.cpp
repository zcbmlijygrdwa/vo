#include <iostream>
#include <opencv2/opencv.hpp>

#include <opencv2/features2d.hpp>
#include "vo_features.h"
#include "MyMatcher.hpp"

#include "CameraConfig.hpp"

using namespace std;
using namespace cv;


int main(int argc, char** argv)
{
    cout<<"VO main program!"<<endl;

    Mat R_f, t_f; //the final rotation and tranlation vectors 

    float scale = 1;

    VideoCapture cap("../dash.mp4");

    Mat i_prev, i_curr;

    int MAX_FEATURES = 500;

    Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);

    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;

    bool isSlamInit = false;
    bool isGlobalRTInit = false;

    if(!cap.isOpened())
    {
        cout<<"failed to open video."<<endl;
    }


    cap >> i_prev;
    cvtColor(i_prev,i_prev,CV_BGR2GRAY);

    CameraConfig camConfig = CameraConfig(i_prev);

    Mat newCameraMatrix;
    Mat i_undistort;
    //undistort the prev frame
    undistort(i_prev, i_undistort, camConfig.camera_matrix, camConfig.distCoeffs, newCameraMatrix);
    i_prev = i_undistort.clone();
        vector<Point2f> pts1, pts2;
    while(!i_prev.empty())
    {
        // capture new frame
        cap >> i_curr;
        //undistort the new frame
        undistort(i_curr, i_undistort, camConfig.camera_matrix, camConfig.distCoeffs, newCameraMatrix);
        i_curr = i_undistort.clone();

        cvtColor(i_curr,i_curr,CV_BGR2GRAY);

        //extract orb features
        orb->detectAndCompute(i_prev,Mat(),keypoints1, descriptors1);
        orb->detectAndCompute(i_curr,Mat(),keypoints2, descriptors2);

        Mat i_prev_kp, i_curr_kp;

        //drawKeypoints(i_prev, keypoints1, i_prev_kp, Scalar::all(-1),DrawMatchesFlags::DEFAULT);
        //drawKeypoints(i_curr, keypoints2, i_curr_kp, Scalar::all(-1),DrawMatchesFlags::DEFAULT);

        //imshow("prev keypoints",i_prev_kp);
        //imshow("curr keypoints",i_curr_kp);

        MyMatcher *mmc = new MyMatcher();

        ////use pure feature matching
        //mmc -> matchKeypoints(pts1, pts2, descriptors1, descriptors2, keypoints1, keypoints2, i_prev, i_curr);

        ////use optical flow
        //mmc -> matchKeypointsWithKLTFusion(pts1, pts2, descriptors1, descriptors2, keypoints1, keypoints2, i_prev, i_curr);

        ////use optical flow as primary
        mmc -> KLTthenFeature(pts1, pts2, descriptors1, descriptors2, keypoints1, keypoints2, i_prev, i_curr);

        cout<<"pts1 size = "<<pts1.size()<<endl;
        cout<<"pts2 size = "<<pts2.size()<<endl;



        //check if slam is init
        if(!isSlamInit)
        {
            //use epipolar f=geometry to solve for fundamental matrix
            Mat fundamental_matrix;

            //find essental matrix for pose recovery
            Mat essential_matrix;
            Mat mask;
            essential_matrix = findEssentialMat(pts1,pts2,camConfig.focal_x, camConfig.pp, RANSAC, 0.99999999999, 1.0, mask);

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
                numInliers = recoverPose(essential_matrix, pts1, pts2, R, t, camConfig.focal_x, camConfig.pp, mask);

                cout<<"numInliers = "<<numInliers<<endl;

                if(!isGlobalRTInit)
                {
                    isGlobalRTInit = true;
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
        pts1 = pts2;
        waitKey(0);
    }


    cap.release();

    return 0;
}
