// Cのmin,maxマクロを無効にする
#define NOMINMAX

#include <iostream>

#include <Windows.h>

#include <pcl/visualization/cloud_viewer.h>

void main()
{
    try {
        // Create Cloud Viewer
        pcl::visualization::PCLVisualizer viewer( "Point Cloud Viewer" );
        while ( !viewer.wasStopped() ){
            viewer.spinOnce();

            // Input Key ( Exit ESC key )
            if ( GetKeyState( VK_ESCAPE ) < 0 ){
                break;
            }
        }
    }
    catch ( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }
}
