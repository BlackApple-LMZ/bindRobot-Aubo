/******************************************************
******************************************************/

#include <iostream>  
#include "camera.h" 

using namespace std;

using cv::Point;

int main( int argc, char **argv )
{
	Camera kinect;
	
	while(1)
   	{  
		if(!kinect.WaitForNew())
        	continue;
        	
        if(!kinect.Run())
			break;
    }

    return 0;  
}



