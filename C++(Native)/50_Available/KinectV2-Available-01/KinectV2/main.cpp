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

    IKinectSensor* kinect = nullptr;

    IColorFrameReader* colorFrameReader = nullptr;
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;

    WAITABLE_HANDLE waitableHandle = 0;

    bool isAvailable = false;

public:

    // 初期化
    void initialize()
    {
        // デフォルトのKinectを取得する
        ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );

        // Kinectを開く
        ERROR_CHECK( kinect->Open() );

        BOOLEAN isOpen = false;
        ERROR_CHECK( kinect->get_IsOpen( &isOpen ) );
        if ( !isOpen ){
            throw std::runtime_error("Kinectが開けません");
        }

        // Kinectの接続確認に登録する
        kinect->SubscribeIsAvailableChanged( &waitableHandle );
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
        // Kinectの状態更新
        updateKinectAvailable();

        // カラーフレームの更新
        updateColorFrame();
    }

    // Kinectの状態更新
    void updateKinectAvailable()
    {
        ComPtr<IIsAvailableChangedEventArgs> args;
        auto ret = kinect->GetIsAvailableChangedEventData( waitableHandle, &args );
        if ( ret != S_OK ) {
            return;
        }

        BOOLEAN available = false;
        args->get_IsAvailable( &available );

        // 未接続→接続
        if ( !isAvailable && available ){
            if ( colorFrameReader == nullptr ){
                // カラーリーダーを取得する
                ComPtr<IColorFrameSource> colorFrameSource;
                ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );
                ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

                // カラー画像のサイズを取得する
                ComPtr<IFrameDescription> colorFrameDescription;
                ERROR_CHECK( colorFrameSource->CreateFrameDescription(
                    ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription ) );
                ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) );
                ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) );
                ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) );

                // バッファーを作成する
                colorBuffer.resize( colorWidth * colorHeight * colorBytesPerPixel );
            }
        }
        // 接続→未接続
        else if ( isAvailable && !available ){
            cv::destroyAllWindows();
        }

        // 状態の更新
        isAvailable = available;
    }

    // カラーフレームの更新
    void updateColorFrame()
    {
        if ( colorFrameReader == nullptr ){
            return;
        }

        // フレームを取得する
        ComPtr<IColorFrame> colorFrame;
        auto ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
        if ( ret == S_OK ){
            // BGRAの形式でデータを取得する
            ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray(
                colorBuffer.size(), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );

            // カラーデータを表示する
            cv::Mat colorImage( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
            cv::imshow( "Color Image", colorImage );

            // スマートポインタを使ってない場合は、自分でフレームを解放する
            // colorFrame->Release();
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
