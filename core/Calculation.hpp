#ifndef Calculation_hpp
#define Calculation_hpp

class Calculation
{

    private:

    public:

template <class T>
        static void Triangulate(const cv::Point_<T> &p1, const cv::Point_<T> &p2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
        {
            cv::Mat A(4,4,CV_32F);

            A.row(0) = p1.x*P1.row(2)-P1.row(0);
            A.row(1) = p1.y*P1.row(2)-P1.row(1);
            A.row(2) = p2.x*P2.row(2)-P2.row(0);
            A.row(3) = p2.y*P2.row(2)-P2.row(1);

            cv::Mat u,w,vt;
            cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
            x3D = vt.row(3).t();
            x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
        }



};

#endif
