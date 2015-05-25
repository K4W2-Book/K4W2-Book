#include <iostream>
#include <sstream>

#include <Kinect.h>

// 次のように使います
// ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );
// 解説でコードを見やすくするためにマクロにしています。
// 実際には展開した形で使うことを検討してください。
#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
    }

void main()
{
    try {
        // Kinectセンサーと関連付ける
        IKinectSensor* kinect = nullptr;
        ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );

        // Kinectセンサーを動作させる
        ERROR_CHECK( kinect->Open() );

        // Kinectセンサーが動いたかどうか状態を取得する
        BOOLEAN isOpen = false;
        ERROR_CHECK( kinect->get_IsOpen( &isOpen ) );
        std::cout << "Kinect is " << (isOpen ? "Open" : "Not Open") << std::endl;

        // ちょっとまつ
        ::Sleep( 3000 );

        // Kinectセンサーの動作を止める
        kinect->Close();
        kinect->Release();
    }
    catch ( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }
}
