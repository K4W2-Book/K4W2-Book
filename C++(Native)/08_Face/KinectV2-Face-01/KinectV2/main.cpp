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
	std::vector<BYTE> colorBuffer;
	int colorWidth;
	int colorHeight;
	unsigned int colorBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;
	cv::Mat colorImage;

	std::array<CComPtr<IFaceFrameReader>, BODY_COUNT> faceFrameReader;

	std::array<cv::Scalar, BODY_COUNT> colors;
	std::array<std::string, FaceProperty::FaceProperty_Count> labels;
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

		// Faceを初期化
		initializeFace();

		// Lookup Table
		colors[0] = cv::Scalar( 255, 0, 0 );
		colors[1] = cv::Scalar( 0, 255, 0 );
		colors[2] = cv::Scalar( 0, 0, 255 );
		colors[3] = cv::Scalar( 255, 255, 0 );
		colors[4] = cv::Scalar( 255, 0, 255 );
		colors[5] = cv::Scalar( 0, 255, 255 );

		labels[0] = "Happy";
		labels[1] = "Engaged";
		labels[2] = "WearingGlasses";
		labels[3] = "LeftEyeClosed";
		labels[4] = "RightEyeClosed";
		labels[5] = "MouthOpen";
		labels[6] = "MouthMoved";
		labels[7] = "LookingAway";
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

	void initializeFace()
	{
		DWORD features =
			FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
			| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
			| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
			| FaceFrameFeatures::FaceFrameFeatures_Happy
			| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
			| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
			| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
			| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
			| FaceFrameFeatures::FaceFrameFeatures_LookingAway
			| FaceFrameFeatures::FaceFrameFeatures_Glasses
			| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

		for( int count = 0; count < BODY_COUNT; count++ ){
			// Face Frame Sourceを取得
			CComPtr<IFaceFrameSource> faceFrameSource;
			ERROR_CHECK( CreateFaceFrameSource( kinect, 0, features, &faceFrameSource ) );

			// Frace Frame Readerを開く
			ERROR_CHECK( faceFrameSource->OpenReader( &faceFrameReader[count] ) );
		}
	}

	void update()
	{
		updateColorFrame();
		updateBodyFrame();
		updateFaceFrame();
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
		for( int count = 0; count < BODY_COUNT; count++ ){
			CComPtr<IBody> body = bodies[count];
			BOOLEAN tracked;
			ERROR_CHECK( body->get_IsTracked( &tracked ) );
			if( !tracked ){
				continue;
			}

			// Tracking IDを取得
			UINT64 trackingId;
			ERROR_CHECK( body->get_TrackingId( &trackingId ) );

			// Tracking IDを登録
			CComPtr<IFaceFrameSource> faceFrameSource;
			ERROR_CHECK( faceFrameReader[count]->get_FaceFrameSource( &faceFrameSource ) );
			ERROR_CHECK( faceFrameSource->put_TrackingId( trackingId ) );
		}
	}

	void updateFaceFrame()
	{
		for( int count = 0; count < BODY_COUNT; count++ ){
			// 最新のFace Frameを取得
			CComPtr<IFaceFrame> faceFrame;
			HRESULT ret = faceFrameReader[count]->AcquireLatestFrame( &faceFrame );
			if( FAILED( ret ) ){
				continue;
			}

			// Tracking IDの登録確認
			BOOLEAN tracked;
			ERROR_CHECK( faceFrame->get_IsTrackingIdValid( &tracked ) );
			if( !tracked ){
				continue;
			}

			// Face Frame Resultを取得
			CComPtr<IFaceFrameResult> faceResult;
			ERROR_CHECK( faceFrame->get_FaceFrameResult( &faceResult ) );
			if( faceResult != nullptr ){
				// 結果を取得、描画
				result( faceResult, count );
			}
		}
	}

	inline void result( const CComPtr<IFaceFrameResult>& result, const int count )
	{
		// Pointの取得、描画
		std::array<PointF, FacePointType::FacePointType_Count> points;
		ERROR_CHECK( result->GetFacePointsInColorSpace( FacePointType::FacePointType_Count, &points[0] ) );
		for( const PointF p : points ){
			int x = static_cast<int>( std::ceil( p.X ) );
			int y = static_cast<int>( std::ceil( p.Y ) );
			cv::circle( colorImage, cv::Point( x, y ), 5, colors[count], -1, CV_AA );
		}

		// Bounding Boxの取得、描画
		RectI box;
		ERROR_CHECK( result->get_FaceBoundingBoxInColorSpace( &box ) );
		int width = box.Right - box.Left;
		int height = box.Bottom - box.Top;
		cv::rectangle( colorImage, cv::Rect( box.Left, box.Top, width, height ), colors[count] );

		// Rotationの取得、描画
		Vector4 quaternion;
		int pitch, yaw, roll;
		ERROR_CHECK( result->get_FaceRotationQuaternion( &quaternion ) );
		quaternion2degree( &quaternion, &pitch, &yaw, &roll );

		int offset = 30;
		if( box.Left && box.Bottom ){
			std::string rotation = "Pitch, Yaw, Roll : " + std::to_string( pitch ) + ", " + std::to_string( yaw ) + ", " + std::to_string( roll );
			cv::putText( colorImage, rotation, cv::Point( box.Left, box.Bottom + offset ), font, 1.0f, colors[count], 2, CV_AA );
		}

		// Propertyの取得、描画
		std::array<DetectionResult, FaceProperty::FaceProperty_Count> results;
		ERROR_CHECK( result->GetFaceProperties( FaceProperty::FaceProperty_Count, &results[0] ) );

		if( box.Left && box.Bottom ){
			std::string* l = &labels[0];
			for( const DetectionResult r : results ){
				std::string property;
				switch( r ){
					case DetectionResult::DetectionResult_Yes:
						property = *l + " : Yes";
						break;
					case DetectionResult::DetectionResult_Maybe:
						property = *l + " : Maybe";
						break;
					case DetectionResult::DetectionResult_No:
						property = *l + " : No";
						break;
					case DetectionResult::DetectionResult_Unknown:
						property = *l + " : Unknown";
						break;
					default:
						break;
				}
				l++;
				offset += 30;
				cv::putText( colorImage, property, cv::Point( box.Left, box.Bottom + offset ), font, 1.0f, colors[count], 2, CV_AA );
			}
		}
	}

	inline void quaternion2degree( const Vector4* quaternion, int* pitch, int* yaw, int* roll )
	{
		double x = quaternion->x;
		double y = quaternion->y;
		double z = quaternion->z;
		double w = quaternion->w;

		*pitch = static_cast<int>( std::atan2( 2 * ( y * z + w * x ), w * w - x * x - y * y + z * z ) / M_PI * 180.0f );
		*yaw = static_cast<int>( std::asin( 2 * ( w * y - x * z ) ) / M_PI * 180.0f );
		*roll = static_cast<int>( std::atan2( 2 * ( x * y + w * z ), w * w + x * x - y * y - z * z ) / M_PI * 180.0f );
	}

	void draw()
	{
		drawFaceFrame();
	}

	void drawFaceFrame()
	{
		// Faceの表示
		if( !colorImage.empty() ){
			cv::imshow( "Face", colorImage );
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