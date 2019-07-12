#ifndef CameraConfig_hpp
#define CameraConfig_hpp

class CameraConfig
{

    private:


    public:

        //for lens
        double focal_x;
        double focal_y;

        //for imaging
        Point2d pp;

        //for distortion
        double k1;
        double k2;
        double p1;
        double p2;
        Mat distCoeffs;
        Mat camera_matrix;

        CameraConfig(Mat &sample)
        {
            focal_x = 1;
            focal_y = 1;

            pp = Point2d(sample.cols/2., sample.rows/2.);

            k1 = -0.0000004;
            k2 = 0;
            p1 = 0;
            p2 = 0;
            distCoeffs = (Mat_<double>(1,4)<<k1, k2, p1, p2);

            camera_matrix = (Mat_<double>(3,3)<<focal_x, 0, pp.x, 0, focal_y, pp.y, 0, 0, 1);
        }

};


#endif
