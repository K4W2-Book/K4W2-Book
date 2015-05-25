// PCL 04 位置合わせ

// Cのmin,maxマクロを無効にする
#define NOMINMAX

// 安全でないメソッドの呼び出しでの警告を無効にする
#define _SCL_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS

#include <iostream>

#include <Windows.h>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/print.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

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

void iterativeClosestPoint( pcl::PointCloud<PointType>::Ptr target,
                            pcl::PointCloud<PointType>::Ptr source)
{
    // 位置合わせ
    pcl::IterativeClosestPoint<PointType, PointType> icp;

    // Targetに合わせてSourceを移動する
    // 移動された点群はalignの引数に格納される
    icp.setInputTarget( target );
    icp.setInputSource( source );
    icp.align( *source );

    // 結果の表示
    std::cout << "has converged: " << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
}

void main()
{
    try {
        // ビューアーを作成する
        pcl::visualization::PCLVisualizer viewer( "Point Cloud Viewer" );

        // 点群
        pcl::PointCloud<PointType>::Ptr cloud1( new pcl::PointCloud<PointType> );
        pcl::PointCloud<PointType>::Ptr cloud2( new pcl::PointCloud<PointType> );

        // PLYファイルを読み込む
        pcl::PLYReader reader;
        reader.read( "model1.ply", *cloud1 );
        reader.read( "model2.ply", *cloud2 );

        // ビューアーに追加して更新(Qキーで次の処理へ)
        viewer.addPointCloud( cloud1, "cloud1" );
        viewer.addPointCloud( cloud2, "cloud2" );
        viewer.spin();

        // VoxelGrid Filter
        voxelGridFilter( cloud1 );
        voxelGridFilter( cloud2 );

        // 位置合わせ
        iterativeClosestPoint( cloud1, cloud2 );

        // ビューアーを更新する(Qキーで次の処理へ)
        viewer.updatePointCloud( cloud1, "cloud1" );
        viewer.updatePointCloud( cloud2, "cloud2" );
        viewer.spin();

        // 点群をマージして重複を削除する
        *cloud1 += *cloud2;
        voxelGridFilter( cloud1 );

        // PLYファイルとして書き出す
        pcl::PLYWriter writer;
        writer.write( "output.ply", *cloud1 );

        // ビューアーを更新する(Qキーで次の処理へ)
        viewer.updatePointCloud( cloud1, "cloud1" );
        viewer.removePointCloud( "cloud2" );
        viewer.spin();
    }
    catch ( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }
}
