#include <iostream>
#include <sstream>

#include <Kinect.h>
#include <opencv2\opencv.hpp>

#include <atlbase.h>

// 次のように使います
// ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );
// 書籍での解説のためにマクロにしています。実際には展開した形で使うことを検討してください。
#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
    }

class KinectApp
{
private:

    // Kinect SDK
    CComPtr<IKinectSensor> kinect = nullptr;

    CComPtr<IInfraredFrameReader> infraredFrameReader = nullptr;

    int infraredWidth;
    int infraredHeight;

    // 表示用
    std::vector<UINT16> infraredBuffer;

public:

    ~KinectApp()
    {
        // Kinectの動作を終了する
        if ( kinect != nullptr ){
            kinect->Close();
        }
    }

    // 初期化
    void initialize()
    {
        // デフォルトのKinectを取得する
        ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );
        ERROR_CHECK( kinect->Open() );

        // 赤外線画像リーダーを取得する
        CComPtr<IInfraredFrameSource> infraredFrameSource;
        ERROR_CHECK( kinect->get_InfraredFrameSource( &infraredFrameSource ) );
        ERROR_CHECK( infraredFrameSource->OpenReader( &infraredFrameReader ) );

        // 赤外線画像のサイズを取得する
        CComPtr<IFrameDescription> infraredFrameDescription;
        ERROR_CHECK( infraredFrameSource->get_FrameDescription(
                                                &infraredFrameDescription ) );
        ERROR_CHECK( infraredFrameDescription->get_Width( &infraredWidth ) );
        ERROR_CHECK( infraredFrameDescription->get_Height( &infraredHeight ) );

        // バッファーを作成する
        infraredBuffer.resize( infraredWidth * infraredHeight );
    }

    void run()
    {
        while ( 1 ) {
            update();
            draw();

            auto key = cv::waitKey( 10 );
            if ( key == 'q' ){
                break;
            }
        }
    }

private:

    // データの更新処理
    void update()
    {
        updateInfrared();
    }

    void updateInfrared()
    {
        // フレームを取得する
        CComPtr<IInfraredFrame> infraredFrame;
        auto ret = infraredFrameReader->AcquireLatestFrame( &infraredFrame );
        if ( FAILED( ret ) ){
            return;
        }

        // データを取得する
        ERROR_CHECK( infraredFrame->CopyFrameDataToArray(
            infraredBuffer.size(), &infraredBuffer[0] ) );
    }

    void draw()
    {
        // カラーデータを表示する
        cv::Mat colorImage( infraredHeight, infraredWidth,
            CV_16UC1, &infraredBuffer[0] );
        cv::imshow( "Infrared Image", colorImage );
    }
};

void main()
{
    try {
        KinectApp app;
        app.initialize();
        app.run();
    }
    catch ( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }
}
