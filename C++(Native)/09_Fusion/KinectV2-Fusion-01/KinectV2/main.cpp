#include <iostream>
#include <sstream>
#include <vector>
#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#include <atlbase.h>
#include <ppl.h> // for Concurrency::parallel_for

#include <Windows.h>
#include <Kinect.h>
#include <NuiKinectFusionApi.h>
// Quote from Kinect for Windows SDK v2.0 - Samples/Native/KinectFusionExplorer-D2D, and Partial Modification
// KinectFusionHelper is: Copyright (c) Microsoft Corporation. All rights reserved.
#include "KinectFusionHelper.h"

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
	CComPtr<IDepthFrameReader> depthFrameReader;
	CComPtr<ICoordinateMapper> coordinateMapper;
	std::vector<BYTE> colorBuffer;
	std::vector<UINT16> depthBuffer;
	int colorWidth;
	int colorHeight;
	int depthWidth;
	int depthHeight;
	unsigned int colorBytesPerPixel;
	unsigned int depthBytesPerPixel;
	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;

	CComPtr<INuiFusionColorReconstruction> reconstruction;
	NUI_FUSION_IMAGE_FRAME* depthImageFrame;
	NUI_FUSION_IMAGE_FRAME* smoothDepthImageFrame;
	NUI_FUSION_IMAGE_FRAME* colorImageFrame;
	NUI_FUSION_IMAGE_FRAME* pointCloudImageFrame;
	NUI_FUSION_IMAGE_FRAME* surfaceImageFrame;
	/*NUI_FUSION_IMAGE_FRAME* normalImageFrame;*/
	NUI_FUSION_RECONSTRUCTION_PARAMETERS reconstructionParameters;
	NUI_FUSION_CAMERA_PARAMETERS cameraParameters;
	Matrix4 worldToCameraTransform;
	cv::Mat surfaceImage;
	/*cv::Mat normalImage;*/

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

		// Depth Frame Sourceを取得、Depth Frame Readerを開く
		CComPtr<IDepthFrameSource> depthFrameSource;
		ERROR_CHECK( kinect->get_DepthFrameSource( &depthFrameSource ) );
		ERROR_CHECK( depthFrameSource->OpenReader( &depthFrameReader ) );

		CComPtr<IFrameDescription> depthFrameDescription;
		ERROR_CHECK( depthFrameSource->get_FrameDescription( &depthFrameDescription ) );
		ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) );
		ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) );
		ERROR_CHECK( depthFrameDescription->get_BytesPerPixel( &depthBytesPerPixel ) );

		depthBuffer.resize( depthWidth * depthHeight );

		// Fusionの初期化
		initializeFusion();
	}

	void run()
	{
		while( 1 ){
			update();
			draw();

			int key = cv::waitKey( 10 );
			if( key == VK_ESCAPE ){
				release();
				cv::destroyAllWindows();
				break;
			}
			else if( key == 'r' ){
				std::cout << "Reset Reconstruction" << std::endl;
				reset();
			}
			else if( key == 's' ){
				std::cout << "Save Mesh File" << std::endl;
				save();
			}
		}
	}

	private:

	void initializeFusion()
	{
		// 再構成パラメーターの設定
		reconstructionParameters.voxelsPerMeter = 256;
		reconstructionParameters.voxelCountX = 512;
		reconstructionParameters.voxelCountY = 384;
		reconstructionParameters.voxelCountZ = 512;

		// Reconstructionの作成
		SetIdentityMatrix( worldToCameraTransform );
		ERROR_CHECK( NuiFusionCreateColorReconstruction( &reconstructionParameters, NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE::NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP, -1, &worldToCameraTransform, &reconstruction ) );

		// カメラパラメーターの設定
		cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
		cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
		cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
		cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;

		// Image Frameの作成
		ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_FLOAT, depthWidth, depthHeight, &cameraParameters, &depthImageFrame ) );
		ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_FLOAT, depthWidth, depthHeight, &cameraParameters, &smoothDepthImageFrame ) );
		ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, &cameraParameters, &colorImageFrame ) );
		ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, depthWidth, depthHeight, &cameraParameters, &pointCloudImageFrame ) );
		ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, &cameraParameters, &surfaceImageFrame ) );
		/*ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, &cameraParameters, &normalImageFrame ) );*/
	}

	void update()
	{
		updateColorFrame();
		updateDepthFrame();
		updateFusionFrame();
	}

	void updateColorFrame()
	{
		CComPtr<IColorFrame> colorFrame;
		HRESULT ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
		if( FAILED( ret ) ){
			return;
		}

		ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray( static_cast<UINT>( colorBuffer.size() ), &colorBuffer[0], colorFormat ) );
	}

	void updateDepthFrame()
	{
		CComPtr<IDepthFrame> depthFrame;
		HRESULT ret = depthFrameReader->AcquireLatestFrame( &depthFrame );
		if( FAILED( ret ) ){
			return;
		}

		ERROR_CHECK( depthFrame->CopyFrameDataToArray( static_cast<UINT>( depthBuffer.size() ), &depthBuffer[0] ) );
	}

	void updateFusionFrame()
	{
		// DepthデータをFloat Imageに入力、平滑化
		ERROR_CHECK( reconstruction->DepthToDepthFloatFrame( &depthBuffer[0], static_cast<UINT>( depthBuffer.size() * depthBytesPerPixel ), depthImageFrame, NUI_FUSION_DEFAULT_MINIMUM_DEPTH/* 0.5[m] */, NUI_FUSION_DEFAULT_MAXIMUM_DEPTH/* 8.0[m] */, true ) );
		ERROR_CHECK( reconstruction->SmoothDepthFloatFrame( depthImageFrame, smoothDepthImageFrame, NUI_FUSION_DEFAULT_SMOOTHING_KERNEL_WIDTH, NUI_FUSION_DEFAULT_SMOOTHING_DISTANCE_THRESHOLD ) );

		// ColorデータをColor Imageに入力
		std::vector<ColorSpacePoint> points( depthWidth * depthHeight );
		ERROR_CHECK( coordinateMapper->MapDepthFrameToColorSpace( depthWidth * depthHeight, &depthBuffer[0], depthWidth * depthHeight, &points[0] ) );
		NUI_FUSION_BUFFER* colorImageFrameBuffer = colorImageFrame->pFrameBuffer;
		RGBQUAD* src = reinterpret_cast<RGBQUAD*>( &colorBuffer[0] );
		RGBQUAD* dst = reinterpret_cast<RGBQUAD*>( colorImageFrameBuffer->pBits );
		Concurrency::parallel_for( 0, depthHeight, [&]( int y ){
			for( int x = 0; x < depthWidth; x++ ){
				unsigned int index = y * depthWidth + x;
				const ColorSpacePoint point = points[index];
				int colorX = static_cast<int>( std::ceil( point.X ) );
				int colorY = static_cast<int>( std::ceil( point.Y ) );
				if( ( colorX >= 0 ) && ( colorX < colorWidth ) && ( colorY >= 0 ) && ( colorY < colorHeight ) ){
					dst[index] = src[colorY * colorWidth + colorX];
				}
				else{
					dst[index] = {};
				}
			}
		} );

		// ワールド座標系からカメラ座標系への変換行列を取得
		ERROR_CHECK( reconstruction->GetCurrentWorldToCameraTransform( &worldToCameraTransform ) );

		// 3次元形状の再構成処理
		HRESULT ret = reconstruction->ProcessFrame( smoothDepthImageFrame, colorImageFrame, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT, NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES, nullptr, &worldToCameraTransform );
		if( FAILED( ret ) ){
			static unsigned int errorCount = 0;
			if( ++errorCount >= 100 ){
				errorCount = 0;
				reset();
			}
		}

		// 結果を取得、描画
		result();
	}

	inline void result()
	{
		// Point Cloudの計算
		ERROR_CHECK( reconstruction->CalculatePointCloud( pointCloudImageFrame, surfaceImageFrame, &worldToCameraTransform ) );

		/*// Shading Color Transform Matrix
		Matrix4 worldToBGRTransform = { 0.0f };
		worldToBGRTransform.M11 = reconstructionParameters.voxelsPerMeter / reconstructionParameters.voxelCountX;
		worldToBGRTransform.M22 = reconstructionParameters.voxelsPerMeter / reconstructionParameters.voxelCountY;
		worldToBGRTransform.M33 = reconstructionParameters.voxelsPerMeter / reconstructionParameters.voxelCountZ;
		worldToBGRTransform.M41 = 0.5f;
		worldToBGRTransform.M42 = 0.5f;
		worldToBGRTransform.M43 = 0.0f;
		worldToBGRTransform.M44 = 1.0f;

		// Shading Point Cloud
		ERROR_CHECK( NuiFusionShadePointCloud( pointCloudImageFrame, &worldToCameraTransform, &worldToBGRTransform, surfaceImageFrame, normalImageFrame ) );*/
	}

	inline void reset()
	{
		// Reconstructionのリセット
		SetIdentityMatrix( worldToCameraTransform );
		ERROR_CHECK( reconstruction->ResetReconstruction( &worldToCameraTransform, nullptr ) );
	}

	void release()
	{
		// Image Frameの解放
		ERROR_CHECK( NuiFusionReleaseImageFrame( depthImageFrame ) );
		ERROR_CHECK( NuiFusionReleaseImageFrame( smoothDepthImageFrame ) );
		ERROR_CHECK( NuiFusionReleaseImageFrame( colorImageFrame ) );
		ERROR_CHECK( NuiFusionReleaseImageFrame( pointCloudImageFrame ) );
		ERROR_CHECK( NuiFusionReleaseImageFrame( surfaceImageFrame ) );
		/*ERROR_CHECK( NuiFusionReleaseImageFrame( normalImageFrame ) );*/
	}

	inline void save()
	{
		// Meshの計算
		CComPtr<INuiFusionColorMesh> mesh;
		ERROR_CHECK( reconstruction->CalculateMesh( 1, &mesh ) );

		// Mesh Fileの保存
		wchar_t* fileName = L"mesh.ply";
		ERROR_CHECK( WriteAsciiPlyMeshFile( mesh, W2OLE( fileName ), true, true ) );

		/*wchar_t* fileName = L"mesh.stl";
		ERROR_CHECK( WriteBinarySTLMeshFile( mesh, W2OLE( fileName ), true ) );*/

		/*wchar_t* fileName = L"mesh.obj";
		ERROR_CHECK( WriteAsciiObjMeshFile( mesh, W2OLE( fileName ), true ) );*/
	}

	void draw()
	{
		drawFusionFrame();
	}

	void drawFusionFrame()
	{
		// Fusionの表示
		NUI_FUSION_BUFFER* surfaceImageFrameBuffer = surfaceImageFrame->pFrameBuffer;
		surfaceImage = cv::Mat( depthHeight, depthWidth, CV_8UC4, surfaceImageFrameBuffer->pBits );
		cv::imshow( "Surface", surfaceImage );
		/*NUI_FUSION_BUFFER* normalImageFrameBuffer = normalImageFrame->pFrameBuffer;
		normalImage = cv::Mat( depthHeight, depthWidth, CV_8UC4, normalImageFrameBuffer->pBits );
		cv::imshow( "Normal", normalImage );*/
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