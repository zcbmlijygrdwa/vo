#include "MapViewer.hpp"
#include "MapPoint.hpp"

#include <iostream>

using namespace std;

int main(int argc, char** argv)
{

cout<<"MapViewer test."<<endl;


MapViewer mv = MapViewer();


for(int i = 0 ; i < 10 ; i ++)
{
    mv.addPoint(i/10.0,2*i/10.0,1 - i/10.0);
}

mv.run();

return 0;
}
