// Cのmin,maxマクロを無効にする
#define NOMINMAX

// 安全でないメソッドの呼び出しでの警告を無効にする
#define _SCL_SECURE_NO_WARNINGS

#include <iostream>

#include <Windows.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include "kinect2_grabber.h"

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

pcl::PointIndices::Ptr segmentation( pcl::PointCloud<PointType>::Ptr& cloud )
{
    pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices );

    // Create the segmentation object
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients( true );
    seg.setModelType( pcl::SACMODEL_PLANE );
    seg.setMethodType( pcl::SAC_RANSAC );
    seg.setDistanceThreshold( 0.01 );

    seg.setInputCloud( cloud );
    seg.segment( *inliers, *coefficients );

    return inliers;
}

void extractIndices( pcl::PointCloud<PointType>::Ptr cloud,
                     pcl::PointIndices::Ptr inliers )
{
    pcl::PointCloud<PointType>::Ptr tmp( new pcl::PointCloud<PointType> );
    pcl::copyPointCloud( *cloud, *tmp );

    //フィルタリング
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud( tmp );
    extract.setIndices( inliers );

    // true にすると平面を除去、false にすると平面以外を除去
    extract.setNegative( false );
    extract.filter( *cloud );
}

void main()
{
    try {
        // ビューアー
        pcl::visualization::PCLVisualizer viewer( "Point Cloud Viewer" );

        // 点群
        pcl::PointCloud<PointType>::Ptr cloud( new pcl::PointCloud<PointType> );

        // 点群の排他処理
        boost::mutex mutex;

        // データの更新ごとに呼ばれる関数(オブジェクト)
        boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
            [&cloud, &mutex]( const pcl::PointCloud<PointType>::ConstPtr &new_cloud ){
                boost::mutex::scoped_lock lock( mutex );
                pcl::copyPointCloud( *new_cloud, *cloud );
        };

        // Kinect2Grabberを開始する
        pcl::Kinect2Grabber grabber;
        grabber.registerCallback( function );
        grabber.start();

        // ビューアーが終了されるまで動作する
        while ( !viewer.wasStopped() ){
            // 表示を更新する
            viewer.spinOnce();

            // 点群がある場合
            boost::mutex::scoped_try_lock lock( mutex );
            if ( (cloud->size() != 0) && lock.owns_lock() ){
                // 点群を間引く
                voxelGridFilter( cloud );

                // 平面を検出
                auto inliers = segmentation( cloud );

                // 平面を抽出
                extractIndices( cloud, inliers );

                // 点群を更新する
                auto ret = viewer.updatePointCloud( cloud, "cloud" );
                if ( !ret ){
                    // 更新がエラーになった場合は、
                    // 未作成なので新しい点群として追加する
                    viewer.addPointCloud( cloud, "cloud" );
                }
            }

            // エスケープキーが押されたら終了する
            if ( GetKeyState( VK_ESCAPE ) < 0 ){
                break;
            }
        }
    }
    catch ( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }
}
