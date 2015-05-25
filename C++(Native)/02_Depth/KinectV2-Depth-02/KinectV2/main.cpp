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

    CComPtr<IKinectSensor> kinect = nullptr;
    CComPtr<IColorFrameReader> colorFrameReader = nullptr;
    std::vector<BYTE> colorBuffer;

    CComPtr<IDepthFrameReader> depthFrameReader = nullptr;
    std::vector<UINT16> depthBuffer;

    const char* ColorWindowName = "Color Image";
    const char* DepthWindowName = "Depth Image";

    int colorWidth;
    int colorHeight;
    const int ColorBytesPerPixel = 4;

    int minDepth;
    int maxDepth;

    UINT16 minDepthReliableDistance;
    UINT16 maxDepthReliableDistance;

    int depthWidth;
    int depthHeight;

    int depthPointX;
    int depthPointY;

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

        // Kinectを開く
        ERROR_CHECK( kinect->Open() );

        BOOLEAN isOpen = false;
        ERROR_CHECK( kinect->get_IsOpen( &isOpen ) );
        if ( !isOpen ){
            throw std::runtime_error( "Kinectが開けません" );
        }

        // カラーリーダーを取得する
        CComPtr<IColorFrameSource> colorFrameSource;
        ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );
        ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

        // カラー画像のサイズを取得する
        CComPtr<IFrameDescription> colorFrameDescription;
        ERROR_CHECK( colorFrameSource->get_FrameDescription( &colorFrameDescription ) );
        ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) );
        ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) );

        // Depthリーダーを取得する
        CComPtr<IDepthFrameSource> depthFrameSource;
        ERROR_CHECK( kinect->get_DepthFrameSource( &depthFrameSource ) );
        ERROR_CHECK( depthFrameSource->OpenReader( &depthFrameReader ) );

        // Depth画像のサイズを取得する
        CComPtr<IFrameDescription> depthFrameDescription;
        ERROR_CHECK( depthFrameSource->get_FrameDescription( &depthFrameDescription ) );
        ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) );
        ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) );

        depthPointX = depthWidth / 2;
        depthPointY = depthHeight / 2;

        // Depthの最大値、最小値を取得する
        ERROR_CHECK( depthFrameSource->get_DepthMinReliableDistance( &minDepthReliableDistance ) );
        ERROR_CHECK( depthFrameSource->get_DepthMaxReliableDistance( &maxDepthReliableDistance ) );

        std::cout << "Depthデータの幅   : " << depthWidth << std::endl;
        std::cout << "Depthデータの高さ : " << depthHeight << std::endl;

        std::cout << "Depth最小値       : " << minDepthReliableDistance << std::endl;
        std::cout << "Depth最大値       : " << maxDepthReliableDistance << std::endl;

        // バッファーを作成する
        colorBuffer.resize( colorWidth * colorHeight * ColorBytesPerPixel );
        depthBuffer.resize( depthWidth * depthHeight );

        // 画面を作成
        cv::namedWindow( ColorWindowName );

        // 表示範囲距離のトラックバーを作成
        minDepth = minDepthReliableDistance;
        maxDepth = maxDepthReliableDistance;
        cv::createTrackbar( "Min Depth", ColorWindowName, &minDepth, maxDepthReliableDistance );
        cv::createTrackbar( "Max Depth", ColorWindowName, &maxDepth, maxDepthReliableDistance );
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
        updateColor();
        updateDepth();
    }

    void updateColor()
    {
        // フレームを取得する
        CComPtr<IColorFrame> colorFrame;
        auto ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
        if ( ret == S_OK ){
            // BGRAの形式でデータを取得する
            ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray(
                colorBuffer.size(), &colorBuffer[0], ColorImageFormat_Bgra ) );

            // 自動解放を使わない場合には、フレームを解放する
            // colorFrame->Release();
        }
    }

    void updateDepth()
    {
        // フレームを取得する
        CComPtr<IDepthFrame> depthFrame;
        auto ret = depthFrameReader->AcquireLatestFrame( &depthFrame );
        if ( ret == S_OK ){
            // データを取得する
            ERROR_CHECK( depthFrame->CopyFrameDataToArray( depthBuffer.size(), &depthBuffer[0] ) );

            // 自動解放を使わない場合には、フレームを解放する
            // depthFrame->Release();
        }
    }

    void draw()
    {
        //drawColorMap();
        drawDepthMap();
    }

    void drawColorMap()
    {
        CComPtr<ICoordinateMapper> mapper;
        ERROR_CHECK( kinect->get_CoordinateMapper( &mapper ) );

        std::vector<ColorSpacePoint> colorSpacePoints( depthBuffer.size() );
        ERROR_CHECK( mapper->MapDepthFrameToColorSpace( depthBuffer.size(), &depthBuffer[0],
            colorSpacePoints.size(), &colorSpacePoints[0] ) );

        // カラーデータを表示する
        cv::Mat colorImage( depthHeight, depthWidth, CV_8UC4 );

        for ( int i = 0; i < colorImage.total(); ++i ){
            int x = (int)colorSpacePoints[i].X;
            int y = (int)colorSpacePoints[i].Y;

            int srcIndex = ((y * colorWidth) + x) * ColorBytesPerPixel;
            int destIndex = i * ColorBytesPerPixel;

            if ( isValidColorRange( x, y ) && isValidDepthRange( i ) ){
                colorImage.data[destIndex + 0] = colorBuffer[srcIndex + 0];
                colorImage.data[destIndex + 1] = colorBuffer[srcIndex + 1];
                colorImage.data[destIndex + 2] = colorBuffer[srcIndex + 2];
            }
            else {
                colorImage.data[destIndex + 0] = 255;
                colorImage.data[destIndex + 1] = 255;
                colorImage.data[destIndex + 2] = 255;
            }
        }

        cv::imshow( ColorWindowName, colorImage );
    }

    void drawDepthMap()
    {
        CComPtr<ICoordinateMapper> mapper;
        ERROR_CHECK( kinect->get_CoordinateMapper( &mapper ) );

        std::vector<DepthSpacePoint> depthSpacePoints( colorWidth * colorHeight );
        ERROR_CHECK( mapper->MapColorFrameToDepthSpace( depthBuffer.size(), &depthBuffer[0],
            depthSpacePoints.size(), &depthSpacePoints[0] ) );

        // カラーデータを表示する
        cv::Mat colorImage( colorHeight, colorWidth, CV_8UC4 );

        for ( int i = 0; i < colorImage.total(); ++i ){
            int x = (int)depthSpacePoints[i].X;
            int y = (int)depthSpacePoints[i].Y;

            int depthIndex = (y * depthWidth) + x;
            int colorIndex = i * ColorBytesPerPixel;

            if ( isValidColorRange( x, y ) && isValidDepthRange( depthIndex ) ){
                colorImage.data[colorIndex + 0] = colorBuffer[colorIndex + 0];
                colorImage.data[colorIndex + 1] = colorBuffer[colorIndex + 1];
                colorImage.data[colorIndex + 2] = colorBuffer[colorIndex + 2];
            }
            else {
                colorImage.data[colorIndex + 0] = 255;
                colorImage.data[colorIndex + 1] = 255;
                colorImage.data[colorIndex + 2] = 255;
            }
        }

        cv::imshow( ColorWindowName, colorImage );
    }

    bool isValidColorRange( int x, int y )
    {
        return ((0 <= x) && (x < colorWidth)) && ((0 <= y) && (y < colorHeight));
    }

    bool isValidDepthRange( int index )
    {
        return (minDepth <= depthBuffer[index]) && (depthBuffer[index] <= maxDepth);
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
