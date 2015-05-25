#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include <Kinect.h>
#include <conio.h>

#include <atlbase.h>

#include "WaveFile.h"

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
    CComPtr<IAudioBeamFrameReader> audioBeamFrameReader = nullptr;

    // 音声データ
    std::vector<BYTE> audioBuffer;
    WaveFile audioFile;

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

        // オーディオビームリーダーを開く
        ERROR_CHECK( audioSource->OpenReader( &audioBeamFrameReader ) );

        // データバッファを作成する
        UINT subFrameLength = 0;
        ERROR_CHECK( audioSource->get_SubFrameLengthInBytes( &subFrameLength ) );

        audioBuffer.resize( subFrameLength );

        // Waveファイルを設定する
        audioFile.Open( "KinectAudio.wav" );
    }

    void run()
    {
        std::cout << "キーを押すと終了します" << std::endl;

        while ( 1 ) {
            update();
            draw();

            if ( _kbhit() != 0 ){
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
            ERROR_CHECK( audioBeamFrame->get_SubFrameCount(
                &subFrameCount ) );

            for ( int j = 0; j < subFrameCount; ++j ){
                CComPtr<IAudioBeamSubFrame> audioBeamSubFrame;
                ERROR_CHECK( audioBeamFrame->GetSubFrame(
                    j, &audioBeamSubFrame ) );

                audioBeamSubFrame->CopyFrameDataToArray(
                    audioBuffer.size(), &audioBuffer[0] );

                // 音声データを書き込む
                audioFile.Write( &audioBuffer[0], audioBuffer.size() );
            }
        }
    }

    void draw()
    {
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
