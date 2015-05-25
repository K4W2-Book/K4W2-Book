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

    // アプリ状態
    int showState = 0;

    // Kinect
    CComPtr<IKinectSensor> kinect = nullptr;
    CComPtr<ICoordinateMapper> coordinateMapper = nullptr;

    // Color
    CComPtr<IColorFrameReader> colorFrameReader = nullptr;
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;

    // Depth
    CComPtr<IDepthFrameReader> depthFrameReader = nullptr;
    std::vector<UINT16> depthBuffer;

    int depthWidth;
    int depthHeight;

    // BodyIndex
    CComPtr<IBodyIndexFrameReader> bodyIndexFrameReader = nullptr;
    std::vector<BYTE> bodyIndexBuffer;

public:

    // 初期化
    void initialize()
    {
        // デフォルトのKinectを取得する
        ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );
        ERROR_CHECK( kinect->Open() );

        // 座標変換インタフェースを取得
        kinect->get_CoordinateMapper( &coordinateMapper );

        // フレームの初期化
        initializeColorFrame();
        initializeDepthFrame();
        initializeBodyIndexFrame();
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
            else if ( (key >> 16) == VK_RIGHT ){
                showState = (showState == 1) ? 0 : 1;
            }
        }
    }

private:

    void initializeColorFrame()
    {
        // カラーリーダーを取得する
        CComPtr<IColorFrameSource> colorFrameSource;
        ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );
        ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

        // カラー画像のサイズを取得する
        CComPtr<IFrameDescription> colorFrameDescription;
        ERROR_CHECK( colorFrameSource->CreateFrameDescription(
            ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription ) );
        ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) );
        ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) );
        ERROR_CHECK( colorFrameDescription->get_BytesPerPixel(
                                                    &colorBytesPerPixel ) );

        // バッファーを作成する
        colorBuffer.resize( colorWidth * colorHeight * colorBytesPerPixel );
    }

    void initializeDepthFrame()
    {
        // Depthリーダーを取得する
        CComPtr<IDepthFrameSource> depthFrameSource;
        ERROR_CHECK( kinect->get_DepthFrameSource( &depthFrameSource ) );
        ERROR_CHECK( depthFrameSource->OpenReader( &depthFrameReader ) );

        // Depth画像のサイズを取得する
        CComPtr<IFrameDescription> depthFrameDescription;
        ERROR_CHECK( depthFrameSource->get_FrameDescription(
                                                &depthFrameDescription ) );
        ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) );
        ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) );

        // バッファーを作成する
        depthBuffer.resize( depthWidth * depthHeight );
    }

    void initializeBodyIndexFrame()
    {
        // ボディインデックスリーダーを取得する
        CComPtr<IBodyIndexFrameSource> bodyIndexFrameSource;
        ERROR_CHECK( kinect->get_BodyIndexFrameSource( &bodyIndexFrameSource ) );
        ERROR_CHECK( bodyIndexFrameSource->OpenReader( &bodyIndexFrameReader ) );

        // バッファーを作成する
        bodyIndexBuffer.resize( depthWidth * depthHeight );
    }

    // データの更新処理
    void update()
    {
        updateColorFrame();
        updateDepthFrame();
        updateBodyIndexFrame();
    }

    void draw()
    {
        // カラー画像の解像度に合わせる
        if ( showState == 0 ){
            drawColorCoodinate();
        }
        // Depth画像の解像度に合わせる
        else {
            drawDepthCoodinate();
        }
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
            colorBuffer.size(), &colorBuffer[0],
            ColorImageFormat::ColorImageFormat_Bgra ) );
    }

    // Depthフレームの更新
    void updateDepthFrame()
    {
        // Depthフレームを取得する
        CComPtr<IDepthFrame> depthFrame;
        auto ret = depthFrameReader->AcquireLatestFrame( &depthFrame );
        if ( FAILED( ret ) ){
            return;
        }

        // データを取得する
        ERROR_CHECK( depthFrame->CopyFrameDataToArray(
            depthBuffer.size(), &depthBuffer[0] ) );
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

    void drawColorCoodinate()
    {
        // カラー画像の解像度でデータを作る
        cv::Mat colorImage = cv::Mat::zeros( colorHeight, colorWidth, CV_8UC4 );

        // カラー座標系に対応するDepth座標系の一覧を取得する
        std::vector<DepthSpacePoint> depthSpace( colorWidth * colorHeight );
        coordinateMapper->MapColorFrameToDepthSpace(
            depthBuffer.size(), &depthBuffer[0], depthSpace.size(), &depthSpace[0] );

        // 人を検出した個所のみカラー画像を表示する
        for ( int i = 0; i < colorWidth * colorHeight; ++i ){
            int depthX = (int)depthSpace[i].X;
            int depthY = (int)depthSpace[i].Y;
            if ( (depthX < 0) || (depthWidth <= depthX) ||
                (depthY < 0) || (depthHeight <= depthY) ){
                continue;
            }

            // Depth座標系のインデックス
            int depthIndex = (depthY * depthWidth) + depthX;
            int bodyIndex = bodyIndexBuffer[depthIndex];

            // 人を検出していない座標
            if ( bodyIndex == 255 ){
                continue;
            }

            // カラー画像を設定する
            int colorImageIndex = i * colorBytesPerPixel;
            colorImage.data[colorImageIndex + 0] = colorBuffer[colorImageIndex + 0];
            colorImage.data[colorImageIndex + 1] = colorBuffer[colorImageIndex + 1];
            colorImage.data[colorImageIndex + 2] = colorBuffer[colorImageIndex + 2];
        }

        cv::imshow( "Color Image", colorImage );
    }

    void drawDepthCoodinate()
    {
        // Depth画像の解像度でデータを作る
        cv::Mat colorImage = cv::Mat::zeros( depthHeight, depthWidth, CV_8UC4 );

        // Depth座標系に対応するカラー座標系の一覧を取得する
        std::vector<ColorSpacePoint> colorSpace( depthWidth * depthHeight );
        coordinateMapper->MapDepthFrameToColorSpace(
            depthBuffer.size(), &depthBuffer[0], colorSpace.size(), &colorSpace[0] );

        // 人を検出した個所のみカラー画像を表示する
        for ( int i = 0; i < depthWidth * depthHeight; ++i ){
            int colorX = (int)colorSpace[i].X;
            int colorY = (int)colorSpace[i].Y;
            if ( (colorX < 0) || (colorWidth <= colorX) ||
                (colorY < 0) || (colorHeight <= colorY) ){
                continue;
            }

            // カラー座標系のインデックス
            int colorIndex = (colorY * colorWidth) + colorX;
            int bodyIndex = bodyIndexBuffer[i];

            // 人を検出した位置だけ色を付ける
            if ( bodyIndex == 255 ){
                continue;
            }

            // カラー画像を設定する
            int colorImageIndex = i * colorBytesPerPixel;
            int colorBufferIndex = colorIndex * colorBytesPerPixel;
            colorImage.data[colorImageIndex + 0] = colorBuffer[colorBufferIndex + 0];
            colorImage.data[colorImageIndex + 1] = colorBuffer[colorBufferIndex + 1];
            colorImage.data[colorImageIndex + 2] = colorBuffer[colorBufferIndex + 2];
        }

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
