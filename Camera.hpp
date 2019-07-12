#ifndef Camera_hpp
#define Camera_hpp


#include <iostream>

class Camera
{

    private:


    public:

        //for focal length
        double fx;
        double fy;

        //Principal point
        Point2d pp;
        double cx;
        double cy;

        //for distortion
        double k1;
        double k2;
        double p1;
        double p2;
        Mat distCoeffs;
        cv::Mat K;
        Camera(Mat &sample)
        {
            fx = 1;
            fy = 1;

            cx = sample.cols/2.;
            cy = sample.rows/2.;

            pp = Point2d(cx, cy);

            k1 = -0.0000004;
            k2 = 0;
            p1 = 0;
            p2 = 0;

            distCoeffs = (Mat_<float>(1,4)<<k1, k2, p1, p2);

            K = cv::Mat::eye(3,3,CV_32F);
            K.at<float>(0,0) = fx;
            K.at<float>(1,1) = fy;
            K.at<float>(0,2) = cx;
            K.at<float>(1,2) = cy;
        }


        void showK()
        {
            std::cout<<"Camera K: "<<K<<std::endl;            
        }

        void showDisCoeffs()
        {
            std::cout<<"Camera distCoeffs: "<<distCoeffs<<std::endl;
        }

};


#endif
