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

    CComPtr<IBodyIndexFrameReader> bodyIndexFrameReader = nullptr;
    int bodyIndexWidth;
    int bodyIndexHeight;

    // 取得データ
    std::vector<BYTE> bodyIndexBuffer;

    // 表示用
    cv::Scalar colors[6];

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

        // ボディインデックスリーダーを取得する
        CComPtr<IBodyIndexFrameSource> bodyIndexFrameSource;
        ERROR_CHECK( kinect->get_BodyIndexFrameSource( &bodyIndexFrameSource ) );
        ERROR_CHECK( bodyIndexFrameSource->OpenReader( &bodyIndexFrameReader ) );

        // ボディインデックスの解像度を取得する
        CComPtr<IFrameDescription> bodyIndexFrameDescription;
        ERROR_CHECK( bodyIndexFrameSource->get_FrameDescription(
                                    &bodyIndexFrameDescription ) );
        bodyIndexFrameDescription->get_Width( &bodyIndexWidth );
        bodyIndexFrameDescription->get_Height( &bodyIndexHeight );

        // バッファーを作成する
        bodyIndexBuffer.resize( bodyIndexWidth * bodyIndexHeight );

        // プレイヤーの色を設定する
        colors[0] = cv::Scalar( 255,   0,   0 );
        colors[1] = cv::Scalar(   0, 255,   0 );
        colors[2] = cv::Scalar(   0,   0, 255 );
        colors[3] = cv::Scalar( 255, 255,   0 );
        colors[4] = cv::Scalar( 255,   0, 255 );
        colors[5] = cv::Scalar(   0, 255, 255 );
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
        updateBodyIndexFrame();
    }

    // ボディインデックスフレームの更新
    void updateBodyIndexFrame()
    {
        // フレームを取得する
        CComPtr<IBodyIndexFrame> bodyIndexFrame;
        auto ret = bodyIndexFrameReader->AcquireLatestFrame( &bodyIndexFrame );
        if ( FAILED( ret ) ){
            return;
        }

        // データを取得する
        ERROR_CHECK( bodyIndexFrame->CopyFrameDataToArray(
            bodyIndexBuffer.size(), &bodyIndexBuffer[0] ) );
    }

    void draw()
    {
        drawBodyIndexFrame();
    }

    void drawBodyIndexFrame()
    {
        // ボディインデックスをカラーデータに変換して表示する
        cv::Mat bodyIndexImage( bodyIndexHeight, bodyIndexWidth, CV_8UC4 );

        for ( int i = 0; i < bodyIndexWidth * bodyIndexHeight; ++i ){
            int index = i * 4;
            // 人がいれば255以外
            if ( bodyIndexBuffer[i] != 255 ){
                auto color = colors[bodyIndexBuffer[i]];
                bodyIndexImage.data[index + 0] = color[0];
                bodyIndexImage.data[index + 1] = color[1];
                bodyIndexImage.data[index + 2] = color[2];
            }
            else{
                bodyIndexImage.data[index + 0] = 0;
                bodyIndexImage.data[index + 1] = 0;
                bodyIndexImage.data[index + 2] = 0;
            }
        }

        cv::imshow( "BodyIndex Image", bodyIndexImage );
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
