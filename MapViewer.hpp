#ifndef MapViewer_hpp
#define MapViewer_hpp

#include <iostream>
#include <pangolin/pangolin.h>
#include "MapPoint.hpp"
#include <mutex>

class MapViewer
{


    private:

    public:
        std::vector<MapPoint> *points;

        MapViewer()
        {
            std::cout<<"A MapViewer instance is constructed."<<std::endl;
            //points.clear();

        }

        ~MapViewer()
        {

            std::cout<<"A MapViewer instance is distructed."<<std::endl;

        }

        void setDataSource(std::vector<MapPoint> *points_in)
        {
            points = points_in;
        }

        //void addPoint(float x, float y, float z)
        //{
        //    points.push_back(MapPoint(x,y,z));    
        //}

        void spin()
        {
            pangolin::CreateWindowAndBind("Main",640,480);
            glEnable(GL_DEPTH_TEST);

            // Define Projection and initial ModelView matrix
            pangolin::OpenGlRenderState s_cam = pangolin::OpenGlRenderState(
                    pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,1000),
                    pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
                    );

            // Create Interactive View in window
            pangolin::Handler3D handler(s_cam);
            pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
                .SetHandler(&handler);

            int mPointSize = 6;
            while( !pangolin::ShouldQuit() )
            {
                // Clear screen and activate view to render into
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                d_cam.Activate(s_cam);

                // Render OpenGL Cube
                //pangolin::glDrawColouredCube();

                glPointSize(mPointSize);
                glBegin(GL_POINTS);
                glColor3f(1.0,1.0,0.0);

                for(size_t i=0 ; i<points->size(); i++)
                {
                    glVertex3f(points->at(i).x, points->at(i).y, points->at(i).z);
                }   
                glEnd();
                // Swap frames and Process Events
                pangolin::FinishFrame();
            }
        }

};


#endif
