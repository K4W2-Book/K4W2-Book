// PCL 02 ダウンサンプリング

// Cのmin,maxマクロを無効にする
#define NOMINMAX

#include <iostream>

#include <Windows.h>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/print.h>
#include <pcl/filters/passthrough.h>

// 点群の型を定義しておく
typedef pcl::PointXYZRGB PointType;

void passThroughFilter( pcl::PointCloud<PointType>::Ptr cloud,
    const std::string &field_name, float limit_min, float limit_max )
{
    // フィルター前の点群の数を表示する
    pcl::console::print_info( "before point clouds : %d\n",
        cloud->size() );

    pcl::PassThrough<PointType> pass;

    // フィルターする点群を設定する
    pass.setInputCloud( cloud );

    // フィルターする範囲を設定する
    pass.setFilterFieldName( field_name );
    pass.setFilterLimits( limit_min, limit_max );

    // フィルターする(新しい点群に保存する)
    pcl::PointCloud<PointType>::Ptr
        cloud_filtered( new pcl::PointCloud<PointType> );
    pass.filter( *cloud_filtered );

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

        // PassThroughフィルター
        passThroughFilter( cloud, "x", -0.3, 0.3 );
        passThroughFilter( cloud, "y", -0.3, 0.3 );
        passThroughFilter( cloud, "z", -1, 0 );

        // ビューアーを更新する
        viewer.updatePointCloud( cloud );
        viewer.spin();
    }
    catch ( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }
}
