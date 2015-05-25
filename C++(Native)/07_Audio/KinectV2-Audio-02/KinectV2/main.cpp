#include <iostream>
#include <sstream>

#include <Kinect.h>
#include <opencv2\opencv.hpp>

#include <math.h>

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

    CComPtr<IKinectSensor> kinect = nullptr;
    CComPtr<IAudioBeamFrameReader> audioBeamFrameReader = nullptr;

    float beamAngle;
    float beamAngleConfidence;

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

        // オーディオを開く
        CComPtr<IAudioSource> audioSource;
        ERROR_CHECK( kinect->get_AudioSource( &audioSource ) );

        ERROR_CHECK( audioSource->OpenReader( &audioBeamFrameReader ) );
    }

    void run()
    {
        std::cout << "キーを押すと終了します" << std::endl;

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
        updateAudioFrame();
    }

    // オーディオフレームの更新
    void updateAudioFrame()
    {
        // ビームフレームリストを取得する
        CComPtr<IAudioBeamFrameList> audioBeamFrameList;
        auto ret = audioBeamFrameReader->AcquireLatestBeamFrames(
                                                &audioBeamFrameList );
        if ( ret != S_OK ){
            return;
        }

        // ビームフレームを取得する
        UINT beamCount = 0;
        ERROR_CHECK( audioBeamFrameList->get_BeamCount( &beamCount ) );
        for ( int i = 0; i < beamCount; ++i ){
            CComPtr<IAudioBeamFrame> audioBeamFrame;
            ERROR_CHECK( audioBeamFrameList->OpenAudioBeamFrame(
                i, &audioBeamFrame ) );

            // サブフレームを取得する
            UINT subFrameCount = 0;
            ERROR_CHECK( audioBeamFrame->get_SubFrameCount( &subFrameCount ) );

            for ( int j = 0; j < subFrameCount; ++j ){
                CComPtr<IAudioBeamSubFrame> audioBeamSubFrame;
                ERROR_CHECK( audioBeamFrame->GetSubFrame( 
                                            j, &audioBeamSubFrame ) );

                // 角度および角度の信頼性を取得する
                ERROR_CHECK( audioBeamSubFrame->get_BeamAngle( &beamAngle ) );
                ERROR_CHECK( audioBeamSubFrame->get_BeamAngleConfidence(
                                                        &beamAngleConfidence ) );
            }
        }
    }

    void draw()
    {
        cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC4);

        // ラジアンから度に変換する
        auto angle = beamAngle * 180 / 3.1416;

        // 線を回転させる(逆回転)
        auto alpha = 3.1416 / -angle;
        int offsetX = 320;
        int offsetY = 240;

        auto X2 = 0 * cos( alpha ) - offsetY * sin( alpha );
        auto Y2 = 0 * sin( alpha ) + offsetY * cos( alpha );

        // 回転させた線を描画する
        cv::line( image, cv::Point( offsetX, 0 ), cv::Point( offsetX + X2, Y2 ),
                    cv::Scalar( 255, 255, 255 ), 10 );

        cv::imshow("AudioBeamAngle", image);
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
