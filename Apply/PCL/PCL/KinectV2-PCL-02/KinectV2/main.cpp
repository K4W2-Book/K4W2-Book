// PCL 02 ダウンサンプリング

// Cのmin,maxマクロを無効にする
#define NOMINMAX

#include <iostream>

#include <Windows.h>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/print.h>
#include <pcl/filters/voxel_grid.h>

// 点群の型を定義しておく
typedef pcl::PointXYZRGB PointType;

void voxelGridFilter( pcl::PointCloud<PointType>::Ptr cloud )
{
    // フィルター前の点群の数を表示する
    pcl::console::print_info( "before point clouds : %d\n",
        cloud->size() );

    // フィルターする範囲
    // Kinect FusionはKinectのカメラ座標系で記録されるのでメートル単位
    // 0.01の場合は1cm単位でフィルターする
    float leaf = 0.01f;

    pcl::VoxelGrid<PointType> grid;

    // フィルターする範囲を設定
    grid.setLeafSize( leaf, leaf, leaf );

    // フィルターする点群を設定
    grid.setInputCloud( cloud );

    // フィルターする(新しい点群に保存する)
    pcl::PointCloud<PointType>::Ptr
        cloud_filtered( new pcl::PointCloud<PointType> );
    grid.filter( *cloud_filtered );

    // 点群を戻す
    pcl::copyPointCloud( *cloud_filtered, *cloud );

    // フィルター後の点群の数を表示する
    pcl::console::print_info( "after point clouds : %d\n",
        cloud->size() );
}

void main()
{
    try {
        // ビューアーを作成する
        pcl::visualization::PCLVisualizer viewer( "Point Cloud Viewer" );

        // 点群
        pcl::PointCloud<PointType>::Ptr cloud( new pcl::PointCloud<PointType> );

        // PLYファイルを読み込む
        pcl::PLYReader reader;
        reader.read( "model1.ply", *cloud );

        // ビューアーに追加して更新
        viewer.addPointCloud( cloud );
        viewer.spinOnce();

        // VoxelGrid Filter
        voxelGridFilter( cloud );

        // ビューアーを更新する
        viewer.updatePointCloud( cloud );
        viewer.spin();
    }
    catch ( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }
}
