// Cのmin,maxマクロを無効にする
#define NOMINMAX

// 安全でないメソッドの呼び出しでの警告を無効にする
#define _SCL_SECURE_NO_WARNINGS

#include <iostream>

#include <Windows.h>

#include <pcl/visualization/cloud_viewer.h>

#include "kinect2_grabber.h"

// 点群の型を定義しておく
typedef pcl::PointXYZRGB PointType;

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
            if ( cloud && lock.owns_lock() ){
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
