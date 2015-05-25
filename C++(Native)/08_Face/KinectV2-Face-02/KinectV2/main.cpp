#include <iostream>
#include <sstream>
#include <vector>
#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#include <atlbase.h>

#include <Windows.h>
#include <Kinect.h>
#include <Kinect.Face.h>

#include <opencv2/opencv.hpp>

#define ERROR_CHECK( ret )											\
	if( FAILED( ret ) ){											\
		std::stringstream ss;										\
		ss << "failed " #ret " " << std::hex << ret << std::endl;	\
		throw std::runtime_error( ss.str().c_str() );				\
	}

class KinectApp
{
private:

	CComPtr<IKinectSensor> kinect;

	CComPtr<IColorFrameReader> colorFrameReader;
	CComPtr<IBodyFrameReader> bodyFrameReader;
	CComPtr<ICoordinateMapper> coordinateMapper;
	std::vector<BYTE> colorBuffer;
	int colorWidth;
	int colorHeight;
	unsigned int colorBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;
	cv::Mat colorImage;

	CComPtr<IHighDefinitionFaceFrameReader> hdFaceFrameReader;
	CComPtr<IFaceModelBuilder> faceModelBuilder;
	CComPtr<IFaceAlignment> faceAlignment;
	CComPtr<IFaceModel> faceModel;
	std::array<float, FaceShapeDeformations::FaceShapeDeformations_Count> shapeUnits;
	UINT32 vertexCount;
	UINT64 trackingId;
	int trackingCount;
	bool produced;

	std::array<cv::Scalar, BODY_COUNT> colors;
	int font = cv::FONT_HERSHEY_SIMPLEX;

public:

	void initialize()
	{
		// Sensorを取得
		ERROR_CHECK( GetDefaultKinectSensor( &kinect ) );

		ERROR_CHECK( kinect->Open() );

		BOOLEAN isOpen;
		ERROR_CHECK( kinect->get_IsOpen( &isOpen ) );
		if( !isOpen ){
			throw std::runtime_error( "failed IKinectSensor::get_IsOpen( &isOpen )" );
		}

		ERROR_CHECK( kinect->get_CoordinateMapper( &coordinateMapper ) );

		// Color Frame Sourceを取得、Color Frame Readerを開く
		CComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );
		ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

		CComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK( colorFrameSource->CreateFrameDescription( colorFormat, &colorFrameDescription ) );
		ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) );
		ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) );
		ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) );

		colorBuffer.resize( colorWidth * colorHeight * colorBytesPerPixel );

		// Body Frame Sourceを取得、Body Frame Readerを開く
		CComPtr<IBodyFrameSource> bodyFrameSource;
		ERROR_CHECK( kinect->get_BodyFrameSource( &bodyFrameSource ) );
		ERROR_CHECK( bodyFrameSource->OpenReader( &bodyFrameReader ) );

		// HDFaceを初期化
		initializeHDFace();

		// Lookup Table
		colors[0] = cv::Scalar( 255, 0, 0 );
		colors[1] = cv::Scalar( 0, 255, 0 );
		colors[2] = cv::Scalar( 0, 0, 255 );
		colors[3] = cv::Scalar( 255, 255, 0 );
		colors[4] = cv::Scalar( 255, 0, 255 );
		colors[5] = cv::Scalar( 0, 255, 255 );
	}

	void run()
	{
		while( 1 ){
			update();
			draw();

			int key = cv::waitKey( 10 );
			if( key == VK_ESCAPE ){
				cv::destroyAllWindows();
				break;
			}
		}
	}

private:

	void initializeHDFace()
	{
		// HDFace Frame Sourceを取得
		CComPtr<IHighDefinitionFaceFrameSource> hdFaceFrameSource;
		ERROR_CHECK( CreateHighDefinitionFaceFrameSource( kinect, &hdFaceFrameSource ) );
		
		// HDFace Frame Readerを開く
		ERROR_CHECK( hdFaceFrameSource->OpenReader( &hdFaceFrameReader ) );

		// Face Alignmentを作成
		ERROR_CHECK( CreateFaceAlignment( &faceAlignment ) );

		// Face Modelを作成
		ERROR_CHECK( CreateFaceModel( 1.0f, FaceShapeDeformations::FaceShapeDeformations_Count, &shapeUnits[0], &faceModel ) );
		ERROR_CHECK( GetFaceModelVertexCount( &vertexCount ) );

		// Face Model Builderを作成、開始
		FaceModelBuilderAttributes attribures = FaceModelBuilderAttributes::FaceModelBuilderAttributes_None;
		ERROR_CHECK( hdFaceFrameSource->OpenModelBuilder( attribures, &faceModelBuilder ) );
		ERROR_CHECK( faceModelBuilder->BeginFaceDataCollection() );
	}

	void update()
	{
		updateColorFrame();
		updateBodyFrame();
		updateHDFaceFrame();
	}

	void updateColorFrame()
	{
		CComPtr<IColorFrame> colorFrame;
		HRESULT ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
		if( FAILED( ret ) ){
			return;
		}

		ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray( static_cast<UINT>( colorBuffer.size() ), &colorBuffer[0], colorFormat ) );

		colorImage = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
	}

	void updateBodyFrame()
	{
		CComPtr<IBodyFrame> bodyFrame;
		HRESULT ret = bodyFrameReader->AcquireLatestFrame( &bodyFrame );
		if( FAILED( ret ) ){
			return;
		}

		std::array<CComPtr<IBody>, BODY_COUNT> bodies;
		ERROR_CHECK( bodyFrame->GetAndRefreshBodyData( BODY_COUNT, &bodies[0] ) );

		// Sensorに最も近いBodyを選択
		findClosestBody( bodies );
	}

	inline void findClosestBody( const std::array<CComPtr<IBody>, BODY_COUNT>& bodies )
	{
		float closest = FLT_MAX;
		for( int count = 0; count < BODY_COUNT; count++ ){
			CComPtr<IBody> body = bodies[count];
			BOOLEAN tracked;
			ERROR_CHECK( body->get_IsTracked( &tracked ) );
			if( !tracked ){
				continue;
			}

			// Joint(Head)を取得
			std::array<Joint, JointType::JointType_Count> joints;
			ERROR_CHECK( body->GetJoints( JointType::JointType_Count, &joints[0] ) );
			Joint joint = joints[JointType::JointType_Head];
			if( joint.TrackingState == TrackingState::TrackingState_NotTracked ){
				continue;
			}

			// Sensorからの距離を算出( √( x^2 + y^2 + z^2 ) )
			CameraSpacePoint point = joint.Position;
			float distance = std::sqrt( std::pow( point.X, 2 ) + std::pow( point.Y, 2 ) + std::pow( point.Z, 2 ) );
			if( closest <= distance ){
				continue;
			}
			closest = distance;

			// Tracking IDを取得
			UINT64 id;
			ERROR_CHECK( body->get_TrackingId( &id ) );
			if( trackingId != id ){
				trackingId = id;
				trackingCount = count;
				produced = false;

				// Tracking IDを登録
				CComPtr<IHighDefinitionFaceFrameSource> hdFaceFrameSource;
				ERROR_CHECK( hdFaceFrameReader->get_HighDefinitionFaceFrameSource( &hdFaceFrameSource ) );
				ERROR_CHECK( hdFaceFrameSource->put_TrackingId( trackingId ) );
			}
		}
	}

	void updateHDFaceFrame()
	{
		// 最新のHDFace Frameを取得
		CComPtr<IHighDefinitionFaceFrame> hdFaceFrame;
		HRESULT ret = hdFaceFrameReader->AcquireLatestFrame( &hdFaceFrame );
		if( FAILED( ret ) ){
			return;
		}

		// Tracking IDの登録確認
		BOOLEAN tracked;
		ERROR_CHECK( hdFaceFrame->get_IsFaceTracked( &tracked ) );
		if( !tracked ){
			return;
		}

		// HDFace Frame Resultを取得
		ERROR_CHECK( hdFaceFrame->GetAndRefreshFaceAlignmentResult( faceAlignment ) );
		if( faceAlignment != nullptr ){
			// Face Modelのフィッティング
			buildFaceModel();

			// 結果を取得、描画
			result();
		}
	}

	inline void buildFaceModel()
	{
		if( produced ){
			cv::putText( colorImage, "Status : Complete", cv::Point( 50, 50 ), font, 1.0f, colors[trackingCount], 2, CV_AA );
			return;
		}

		// Face Model Builderの状態を取得
		FaceModelBuilderCollectionStatus collection;
		ERROR_CHECK( faceModelBuilder->get_CollectionStatus( &collection ) );
		if( collection ){
			// コレクション状態
			cv::putText( colorImage, "Status : " + std::to_string( collection ), cv::Point( 50, 50 ), font, 1.0f, colors[trackingCount], 2, CV_AA );
			std::string status = status2string( collection );
			cv::putText( colorImage, status, cv::Point( 50, 80 ), font, 1.0f, colors[trackingCount], 2, CV_AA );

			// キャプチャー状態
			FaceModelBuilderCaptureStatus capture;
			ERROR_CHECK( faceModelBuilder->get_CaptureStatus( &capture ) );
			status = status2string( capture );
			cv::putText( colorImage, status, cv::Point( 50, 110 ), font, 1.0f, colors[trackingCount], 2, CV_AA );

			return;
		}

		// Face Modelのフィッティング
		CComPtr<IFaceModelData> faceModelData;
		ERROR_CHECK( faceModelBuilder->GetFaceData( &faceModelData ) );
		if( faceModelData != nullptr ){
			ERROR_CHECK( faceModelData->ProduceFaceModel( &faceModel ) );
			produced = true;
		}
	}

	inline std::string status2string( const FaceModelBuilderCollectionStatus collection )
	{
		std::string status;
		if( collection & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded ){
			status = "Need : Tilted Up Views";
		}
		else if( collection & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_RightViewsNeeded ){
			status = "Need : Right Views";
		}
		else if( collection & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_LeftViewsNeeded ){
			status = "Need : Left Views";
		}
		else if( collection & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_FrontViewFramesNeeded ){
			status = "Need : Front ViewFrames";
		}

		return status;
	}

	inline std::string status2string( const FaceModelBuilderCaptureStatus capture )
	{
		std::string status;
		switch( capture ){
			case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooFar:
				status = "Error : Face Too Far from Camera";
				break;
			case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooNear:
				status = "Error : Face Too Near to Camera";
				break;
			case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_MovingTooFast:
				status = "Error : Moving Too Fast";
				break;
			default:
				status = "";
				break;
		}

		return status;
	}

	inline void result()
	{
		// Vertexs(1347点)の取得、描画
		std::vector<CameraSpacePoint> vertexs( vertexCount );
		ERROR_CHECK( faceModel->CalculateVerticesForAlignment( faceAlignment, vertexCount, &vertexs[0] ) );
		for( const CameraSpacePoint v : vertexs ){
			ColorSpacePoint point;
			ERROR_CHECK( coordinateMapper->MapCameraPointToColorSpace( v, &point ) );
			int x = static_cast<int>( std::ceil( point.X ) );
			int y = static_cast<int>( std::ceil( point.Y ) );
			if( ( x >= 0 ) && ( x < colorWidth ) && ( y >= 0 ) && ( y < colorHeight ) ){
				cv::circle( colorImage, cv::Point( x, y ), 2, colors[trackingCount], -1, CV_AA );
			}
		}

		/*// Head Pivot Point
		CameraSpacePoint point;
		ERROR_CHECK( faceAlignment->get_HeadPivotPoint( &point ) );
		std::cout << point.X << ", " << point.Y << ", " << point.Z << std::endl;*/

		/*// Animation Units ... Motion of Face Parts that Represent Expression (17 AUs)
		std::array<float, FaceShapeAnimations::FaceShapeAnimations_Count> animationUnits;
		ERROR_CHECK( faceAlignment->GetAnimationUnits( FaceShapeAnimations::FaceShapeAnimations_Count, &animationUnits[0] ) );
		for( const float au : animationUnits ){
			std::cout << std::to_string( au ) << std::endl;
		}*/

		/*// Shape Units ... Deformations from Default Face Model (94 points)
		ERROR_CHECK( faceModel->GetFaceShapeDeformations( FaceShapeDeformations::FaceShapeDeformations_Count, &shapeUnits[0] ) );
		for( const float su : shapeUnits ){
			std::cout << std::to_string( su ) << std::endl;
		}*/

		/*// Face Model Scale
		float scale;
		ERROR_CHECK( faceModel->get_Scale( &scale ) );
		std::cout << std::to_string( scale ) << std::endl;*/

		/*// Hair Color (XBGR)
		// Set FaceModelBuilderAttributes::FaceModelBuilderAttributes_HairColor to IHighDefinitionFaceFrameSource::OpenModelBuilder()
		UINT32 hairColor;
		ERROR_CHECK( faceModel->get_HairColor( &hairColor ) );
		std::cout << ( ( hairColor & 0x00ff0000 ) >> 16 ) << std::endl; // B
		std::cout << ( ( hairColor & 0x0000ff00 ) >> 8  ) << std::endl; // G
		std::cout << ( ( hairColor & 0x000000ff ) >> 0  ) << std::endl; // R*/

		/*// Skin Color (XBGR)
		// Set FaceModelBuilderAttributes::FaceModelBuilderAttributes_SkinColor to IHighDefinitionFaceFrameSource::OpenModelBuilder()
		UINT32 skinColor;
		ERROR_CHECK( faceModel->get_SkinColor( &skinColor ) );
		std::cout << ( ( skinColor & 0x00ff0000 ) >> 16 ) << std::endl; // B
		std::cout << ( ( skinColor & 0x0000ff00 ) >> 8  ) << std::endl; // G
		std::cout << ( ( skinColor & 0x000000ff ) >> 0  ) << std::endl; // R*/
	}

	void draw()
	{
		drawHDFaceFrame();
	}

	void drawHDFaceFrame()
	{
		// HDFaceの表示
		if( !colorImage.empty() ){
			cv::imshow( "HDFace", colorImage );
		}
	}
};

void main()
{
	try{
		KinectApp app;
		app.initialize();
		app.run();
	}
	catch( std::exception& ex ){
		std::cout << ex.what() << std::endl;
	}
}