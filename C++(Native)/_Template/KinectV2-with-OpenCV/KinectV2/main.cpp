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
    CComPtr<IColorFrameReader> colorFrameReader = nullptr;

    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;

    ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;

    // 表示部分
    std::vector<BYTE> colorBuffer;

public:

    ~KinectApp()
    {
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

        // カラーリーダーを取得する
        CComPtr<IColorFrameSource> colorFrameSource;
        ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );
        ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

        // カラー画像のサイズを取得する
        CComPtr<IFrameDescription> colorFrameDescription;
        ERROR_CHECK( colorFrameSource->CreateFrameDescription(
            colorFormat, &colorFrameDescription ) );
        ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) );
        ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) );
        ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) );
        std::cout << "create  : " << colorWidth << ", " <<
                                     colorHeight << ", " <<
                                     colorBytesPerPixel << std::endl;

        // バッファーを作成する
        colorBuffer.resize( colorWidth * colorHeight * colorBytesPerPixel );
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
        updateColorFrame();
    }

    // カラーフレームの更新
    void updateColorFrame()
    {
        // フレームを取得する
        CComPtr<IColorFrame> colorFrame;
        auto ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
        if ( FAILED( ret ) ){
            return;
        }

        // BGRAの形式でデータを取得する
        ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray(
            colorBuffer.size(), &colorBuffer[0], ColorImageFormat_Bgra ) );
    }

    // データの表示処理
    void draw()
    {
        drawColorFrame();
    }


    // カラーデータの表示処理
    void drawColorFrame()
    {
        // カラーデータを表示する
        cv::Mat colorImage( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
        cv::imshow( "Color Image", colorImage );
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
