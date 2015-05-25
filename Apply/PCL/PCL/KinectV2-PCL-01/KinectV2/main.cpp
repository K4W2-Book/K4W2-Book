// PCL 01 モデルを表示

// Cのmin,maxマクロを無効にする
#define NOMINMAX

#include <iostream>

#include <Windows.h>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

// 点群の型を定義しておく
typedef pcl::PointXYZRGB PointType;

void main()
{
    try {
        // ビューアー
        pcl::visualization::PCLVisualizer viewer( "Point Cloud Viewer" );

        // 点群
        pcl::PointCloud<PointType>::Ptr cloud( new pcl::PointCloud<PointType> );

        // PLYファイルを読み込む
        pcl::PLYReader reader;
        reader.read( "model1.ply", *cloud );

        // ビューアーに追加して表示
        viewer.addPointCloud( cloud );
        viewer.spin();
    }
    catch ( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }
}
