#ifndef MapPoint_hpp
#define MapPoint_hpp

class MapPoint
{

    private:


    public:

        float x;
        float y;
        float z;

        MapPoint(float xi, float yi, float zi)
        {
            x = xi;
            y = yi;
            z = zi;
        }


        //3D position
        //Point3f position;

        //ORB decriptor
        //Descriptor descriptor;

        //maximum and minimumu observed distance
        float observed_distance_max;
        float observed_distance_min;


};

#endif
