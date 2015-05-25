using System;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Threading.Tasks;
using System.IO;
using System.Media;
using Microsoft.Kinect;
using Microsoft.Kinect.Fusion;

// Quoted from Kinect for Windows SDK v2.0 - Samples/Managed/KinectFusionExplorer
// KinectFusionHelper.cs : Copyright (c) Microsoft Corporation.  All rights reserved.
using Microsoft.Samples.Kinect.KinectFusionExplorer;

namespace KinectV2
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        //Kinect
        KinectSensor kinect;
        CoordinateMapper coordinateMapper;
        MultiSourceFrameReader multiFrameReader;
        int colorWidth;
        int colorHeight;
        uint colorBytesPerPixel;
        byte[] colorImagePixels;
        int depthWidth;
        int depthHeight;
        ushort[] depthImagePixels;

        //Fusion
        ColorReconstruction reconstruction;
        FusionFloatImageFrame depthImageFrame;
        FusionFloatImageFrame smoothDepthImageFrame;
        FusionColorImageFrame colorImageFrame;
        FusionPointCloudImageFrame pointCloudImageFrame;
        FusionColorImageFrame surfaceImageFrame;
        ReconstructionParameters reconstructionParameters;
        CameraParameters cameraParameters;
        Matrix4 worldToCameraTransform;
        private const int SmoothingKernelWidth = 1; // 0=just copy, 1=3x3, 2=5x5, 3=7x7, here we create a 3x3 kernel
        private const float SmoothingDistanceThreshold = 0.04f; // 4cm, could use up to around 0.1f;
        int errorCount = 0;

        //WPF
        WriteableBitmap drawBitmap;
        byte[] drawBuffer;
        int drawStride;
        Int32Rect drawRect;

        public MainWindow()
        {
            InitializeComponent();
        }
        void InitializeFusion()
        {
            // Reconstruction Parameters
            float voxelPerMeter = 256;
            int voxelsX = 512;
            int voxelsY = 384;
            int voxelsZ = 512;
            reconstructionParameters = new ReconstructionParameters( voxelPerMeter, voxelsX, voxelsY, voxelsZ );

            //カメラ座標の初期値をワールド座標に設定
            worldToCameraTransform = Matrix4.Identity;

            //FusionのReconstructionオブジェクトを作成
            reconstruction = ColorReconstruction.FusionCreateReconstruction( reconstructionParameters, ReconstructionProcessor.Amp, -1, worldToCameraTransform );

            // Fusionのイメージフレームを作成
            cameraParameters = CameraParameters.Defaults;
            depthImageFrame = new FusionFloatImageFrame( depthWidth, depthHeight, cameraParameters );
            smoothDepthImageFrame = new FusionFloatImageFrame( depthWidth, depthHeight, cameraParameters );
            colorImageFrame = new FusionColorImageFrame( depthWidth, depthHeight, cameraParameters );
            pointCloudImageFrame = new FusionPointCloudImageFrame( depthWidth, depthHeight, cameraParameters );
            surfaceImageFrame = new FusionColorImageFrame( depthWidth, depthHeight, cameraParameters );
        }
        private void Window_Loaded( object sender, RoutedEventArgs e )
        {
            try {
                kinect = KinectSensor.GetDefault();
                if ( kinect == null ) {
                    throw new Exception( "Kinectを開けません" );
                }

                kinect.Open();
                coordinateMapper = kinect.CoordinateMapper;
                multiFrameReader = kinect.OpenMultiSourceFrameReader( FrameSourceTypes.Color | FrameSourceTypes.Depth );
                multiFrameReader.MultiSourceFrameArrived += multiFrameReader_MultiSourceFrameArrived;
                FrameDescription depthFrameDescription = kinect.DepthFrameSource.FrameDescription;
                depthWidth = depthFrameDescription.Width;
                depthHeight = depthFrameDescription.Height;
                depthImagePixels = new ushort[depthWidth*depthHeight];

                FrameDescription colorFrameDescription = kinect.ColorFrameSource.CreateFrameDescription( ColorImageFormat.Bgra );
                colorWidth = colorFrameDescription.Width;
                colorHeight = colorFrameDescription.Height;
                colorBytesPerPixel = colorFrameDescription.BytesPerPixel;
                colorImagePixels = new byte[colorWidth * colorHeight * colorBytesPerPixel];

                drawBitmap = new WriteableBitmap(
                                    depthWidth, depthHeight,
                                    96, 96, PixelFormats.Bgra32, null );
                drawStride = depthWidth * (int)colorBytesPerPixel;
                drawRect = new Int32Rect( 0, 0, depthWidth, depthHeight );
                drawBuffer = new byte[drawStride * depthHeight];
                ImageColor.Source = drawBitmap;

                InitializeFusion();
            }
            catch ( Exception ex ) {
                MessageBox.Show( ex.Message );
                Close();
            }
        }
        unsafe void UpdateFusionFrame()
        {
            //KinectのDepthImageデータからFusionのDepthImageFrameを作成
            reconstruction.DepthToDepthFloatFrame( depthImagePixels, depthImageFrame,
                FusionDepthProcessor.DefaultMinimumDepth, FusionDepthProcessor.DefaultMaximumDepth, true );
            
            //depthImageFrameに平滑化をかけてsmoothDepthFrameに格納
            reconstruction.SmoothDepthFloatFrame( depthImageFrame, smoothDepthImageFrame, SmoothingKernelWidth, SmoothingDistanceThreshold );

            //色情報を取得するためにDepthImage各点のColorImageでの位置を計算
            ColorSpacePoint[] points = new ColorSpacePoint[depthWidth * depthHeight];
            coordinateMapper.MapDepthFrameToColorSpace( depthImagePixels, points );

            //colorImageFrameBufferに色情報を格納
            int[] colorImageFrameBuffer = new int[depthWidth * depthHeight];
            fixed ( byte* ptrColorImagePixels = colorImagePixels ) {
                int* rawColorPixels = (int*)ptrColorImagePixels;
                Parallel.For( 0, depthHeight,
                    y =>
                    {
                        for ( int x = 0; x < depthWidth; x++ ) {
                            int index = y * depthWidth + x;
                            ColorSpacePoint point = points[index];
                            int colorX = (int)(Math.Ceiling( point.X ));
                            int colorY = (int)(Math.Ceiling( point.Y ));
                            if ( (colorX >= 0) && (colorX < colorWidth) && (colorY >= 0) && (colorY < colorHeight) ) {
                                colorImageFrameBuffer[index] = rawColorPixels[colorY * colorWidth + colorX];
                            }
                        }

                    } );
            }

            //colorImageFrameBufferからFusionのcolorImageFrameを作成
            colorImageFrame.CopyPixelDataFrom( colorImageFrameBuffer );

            //現在のカメラ位置や角度の情報を計算に反映させるため、worldToCameraTransformを最新に更新
            worldToCameraTransform = reconstruction.GetCurrentWorldToCameraTransform();

            //最新のsmoothDepthImageFrame,colorImageFrameをFusionのReconstructionオブジェクトに反映
            float alignmentEnergy = 0.0f;
            bool ret = reconstruction.ProcessFrame( smoothDepthImageFrame, colorImageFrame, FusionDepthProcessor.DefaultAlignIterationCount,
                FusionDepthProcessor.DefaultIntegrationWeight, FusionDepthProcessor.DefaultColorIntegrationOfAllAngles,
                out alignmentEnergy, worldToCameraTransform );
            //反映結果がエラーならカウントし、カウンタが100を越えたら初めからやり直す
            if ( !ret ) {
                if ( ++errorCount>=100 ) {
                    errorCount = 0;
                    reset();
                }
            }
            //結果を格納
            result();
        }
        void result()
        {
            reconstruction.CalculatePointCloud( pointCloudImageFrame, surfaceImageFrame, worldToCameraTransform );
        }
        void reset()
        {
            worldToCameraTransform = Matrix4.Identity;
            reconstruction.ResetReconstruction( worldToCameraTransform );
        }
        void multiFrameReader_MultiSourceFrameArrived( object sender, MultiSourceFrameArrivedEventArgs e )
        {
            MultiSourceFrameReference frameReference = e.FrameReference;
            MultiSourceFrame multiSourceFrame = null;
            DepthFrame depthFrame = null;
            ColorFrame colorFrame = null;

            try {
                multiSourceFrame = frameReference.AcquireFrame();
                if ( multiSourceFrame == null ) {
                    return;
                }
                depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();
                colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();

                if ( depthFrame == null || colorFrame == null ) {
                    return;
                }
                depthFrame.CopyFrameDataToArray( depthImagePixels );
                colorFrame.CopyConvertedFrameDataToArray( colorImagePixels, ColorImageFormat.Bgra );
            }
            catch ( Exception ex ) {
                MessageBox.Show( ex.ToString() );
            }
            finally {
                if ( depthFrame!=null ) {
                    depthFrame.Dispose();
                    depthFrame = null;
                }
                if ( colorFrame!=null ) {
                    colorFrame.Dispose();
                    colorFrame = null;
                }
                if ( multiSourceFrame!=null ) {
                    multiSourceFrame = null;
                }
            }
            UpdateFusionFrame();
            DrawFusionFrame();
        }
        void DrawFusionFrame()
        {
            int[] surfaceImagePixels = new int[depthWidth*depthHeight];
            surfaceImageFrame.CopyPixelDataTo( surfaceImagePixels );
            drawBitmap.WritePixels( drawRect, surfaceImagePixels, drawStride, 0 );
        }

        void save()
        {
            ColorMesh mesh;
            mesh = reconstruction.CalculateMesh( 3 );
            using ( var writer = new StreamWriter( @"mesh.ply", false, System.Text.Encoding.ASCII ) ) {
                KinectFusionHelper.SaveAsciiPlyMesh( mesh, writer, true, true );
            }
        }
        private void Window_Closing( object sender, System.ComponentModel.CancelEventArgs e )
        {
            if ( kinect != null ) {
                kinect.Close();
                kinect = null;
            }
        }

        private void Window_KeyDown( object sender, System.Windows.Input.KeyEventArgs e )
        {
            if ( e.Key == System.Windows.Input.Key.R ) {
                reset();
            }
            if ( e.Key == System.Windows.Input.Key.S ) {
                save();
            }
        }
    }
}
