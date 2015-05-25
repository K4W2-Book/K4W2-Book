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

    // Kinect SDK
    CComPtr<IKinectSensor> kinect = nullptr;

    // BodyIndex
    CComPtr<IBodyIndexFrameReader> bodyIndexFrameReader = nullptr;
    int BodyIndexWidth;
    int BodyIndexHeight;
    std::vector<BYTE> bodyIndexBuffer;

    // Body
    CComPtr<IBodyFrameReader> bodyFrameReader = nullptr;
    IBody* bodies[6];

    // Audio
    CComPtr<IAudioBeamFrameReader> audioBeamFrameReader;
    float beamAngle;

    // ビーム方向のTrackingIdとそのインデックス
    UINT64 audioTrackingId = (UINT64)-1;
    int audioTrackingIndex = -1;


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

        // ボディリーダーを取得する
        CComPtr<IBodyFrameSource> bodyFrameSource;
        ERROR_CHECK( kinect->get_BodyFrameSource( &bodyFrameSource ) );
        ERROR_CHECK( bodyFrameSource->OpenReader( &bodyFrameReader ) );

        for ( auto& body : bodies ){
            body = nullptr;
        }

        // ボディインデックスリーダーを取得する
        CComPtr<IBodyIndexFrameSource> bodyIndexFrameSource;
        ERROR_CHECK( kinect->get_BodyIndexFrameSource( &bodyIndexFrameSource ) );
        ERROR_CHECK( bodyIndexFrameSource->OpenReader( &bodyIndexFrameReader ) );

        // ボディインデックスの解像度を取得する
        CComPtr<IFrameDescription> bodyIndexFrameDescription;
        ERROR_CHECK( bodyIndexFrameSource->get_FrameDescription(
            &bodyIndexFrameDescription ) );
        bodyIndexFrameDescription->get_Width( &BodyIndexWidth );
        bodyIndexFrameDescription->get_Height( &BodyIndexHeight );

        // バッファーを作成する
        bodyIndexBuffer.resize( BodyIndexWidth * BodyIndexHeight );

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
        updateBodyFrame();
        updateBodyIndexFrame();
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

                // 音声方向を取得する
                ERROR_CHECK( audioBeamSubFrame->get_BeamAngle( &beamAngle ) );

                // ビーム方向にいる人の数を取得する
                UINT32 count = 0;
                ERROR_CHECK( audioBeamSubFrame->get_AudioBodyCorrelationCount(
                                                                    &count ) );
                if ( count == 0 ){
                    audioTrackingId = (UINT64)-1;
                    return;
                }

                // ビーム方向の人のTrackingIdを取得する
                CComPtr<IAudioBodyCorrelation> audioBodyCorrelation;
                ERROR_CHECK( audioBeamSubFrame->GetAudioBodyCorrelation(
                                                        0, &audioBodyCorrelation ) );

                ERROR_CHECK( audioBodyCorrelation->get_BodyTrackingId(
                                                        &audioTrackingId ) );
            }
        }
    }

    // ボディフレームの更新
    void updateBodyFrame()
    {
        // フレームを取得する
        CComPtr<IBodyFrame> bodyFrame;
        auto ret = bodyFrameReader->AcquireLatestFrame( &bodyFrame );
        if ( ret != S_OK ){
            return;
        }

        // 前回のBodyを解放する
        for ( auto& body : bodies ){
            if ( body != nullptr ){
                body->Release();
                body = nullptr;
            }
        }

        // データを取得する
        ERROR_CHECK( bodyFrame->GetAndRefreshBodyData( 6, &bodies[0] ) );

        // ビーム方向の人のインデックスを探す
        audioTrackingIndex = -1;

        if ( audioTrackingId != (UINT64)-1 ){
            for ( int i = 0; i < 6; ++i ){
                UINT64 trackingId = 0;
                bodies[i]->get_TrackingId( &trackingId );
                if ( trackingId == audioTrackingId ){
                    audioTrackingIndex = i;
                    break;
                }
            }
        }
    }

    // ボディインデックスフレームの更新
    void updateBodyIndexFrame()
    {
        // フレームを取得する
        CComPtr<IBodyIndexFrame> bodyIndexFrame;
        auto ret = bodyIndexFrameReader->AcquireLatestFrame( &bodyIndexFrame );
        if ( ret != S_OK ){
            return;
        }

        // データを取得する
        ERROR_CHECK( bodyIndexFrame->CopyFrameDataToArray(
                            bodyIndexBuffer.size(), &bodyIndexBuffer[0] ) );
    }

    void draw()
    {
        cv::Mat image = cv::Mat::zeros( BodyIndexHeight, BodyIndexWidth, CV_8UC4 );

        // ビーム方向の人に色付けする
        for ( int i = 0; i < BodyIndexWidth * BodyIndexHeight; ++i ){
            int index = i * 4;
            // 人がいれば255以外
            if ( bodyIndexBuffer[i] != 255 ){
                if ( bodyIndexBuffer[i] == audioTrackingIndex ){
                    image.data[index + 0] = 255;
                    image.data[index + 1] = 0;
                    image.data[index + 2] = 0;
                }
                else {
                    image.data[index + 0] = 0;
                    image.data[index + 1] = 0;
                    image.data[index + 2] = 255;
                }
            }
            else{
                image.data[index + 0] = 255;
                image.data[index + 1] = 255;
                image.data[index + 2] = 255;
            }
        }

        // ラジアンから度に変換する
        auto angle = beamAngle * 180 / 3.1416;

        // 線を回転させる(逆回転)
        auto alpha = 3.1416 / -angle;
        int offsetX = BodyIndexWidth / 2;
        int offsetY = BodyIndexHeight / 2;

        auto X2 = 0 * cos( alpha ) - offsetY * sin( alpha );
        auto Y2 = 0 * sin( alpha ) + offsetY * cos( alpha );

        // 回転させた線を描画する
        cv::line( image, cv::Point( offsetX, 0 ), 
                  cv::Point( offsetX + X2, Y2 ), 0, 10 );

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
