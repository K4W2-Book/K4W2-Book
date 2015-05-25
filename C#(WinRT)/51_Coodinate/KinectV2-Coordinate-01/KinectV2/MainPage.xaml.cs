using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Threading.Tasks;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Popups;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Media.Imaging;
using Windows.UI.Xaml.Navigation;
using WindowsPreview.Kinect;

// 空白ページのアイテム テンプレートについては、http://go.microsoft.com/fwlink/?LinkId=234238 を参照してください

namespace KinectV2
{
    /// <summary>
    /// それ自体で使用できる空白ページまたはフレーム内に移動できる空白ページ。
    /// </summary>
    public sealed partial class MainPage : Page
    {
        // Kinect
        KinectSensor kinect;
        CoordinateMapper mapper;
        MultiSourceFrameReader multiReader;

        // Color
        FrameDescription colorFrameDesc;
        ColorImageFormat colorFormat = ColorImageFormat.Bgra;
        byte[] colorBuffer;

        // Depth
        FrameDescription depthFrameDesc;
        ushort[] depthBuffer;

        // BodyIndex
        byte[] bodyIndexBuffer;

        public MainPage()
        {
            this.InitializeComponent();
        }

        protected override void OnNavigatedTo( NavigationEventArgs e )
        {
            base.OnNavigatedTo( e );

            try {
                kinect = KinectSensor.GetDefault();
                kinect.Open();

                mapper = kinect.CoordinateMapper;

                // カラー画像の情報を作成する(BGRAフォーマット)
                colorFrameDesc = kinect.ColorFrameSource.CreateFrameDescription(
                                                        colorFormat );
                colorBuffer = new byte[colorFrameDesc.LengthInPixels *
                                       colorFrameDesc.BytesPerPixel];

                // Depthデータの情報を取得する
                depthFrameDesc = kinect.DepthFrameSource.FrameDescription;
                depthBuffer = new ushort[depthFrameDesc.LengthInPixels];

                // BodyIndexデータの情報を取得する
                var bodyIndexFrameDesc = kinect.BodyIndexFrameSource.FrameDescription;
                bodyIndexBuffer = new byte[bodyIndexFrameDesc.LengthInPixels];

                // フレームリーダーを開く
                multiReader = kinect.OpenMultiSourceFrameReader(
                    FrameSourceTypes.Color |
                    FrameSourceTypes.Depth |
                    FrameSourceTypes.BodyIndex );

                multiReader.MultiSourceFrameArrived += multiReader_MultiSourceFrameArrived;
            }
            catch ( Exception ex ) {
                MessageDialog dlg = new MessageDialog(ex.Message);
                dlg.ShowAsync();
            }
        }

        protected override void OnNavigatingFrom( NavigatingCancelEventArgs e )
        {
            base.OnNavigatingFrom( e );

            if ( multiReader != null ) {
                multiReader.MultiSourceFrameArrived -= multiReader_MultiSourceFrameArrived;
                multiReader.Dispose();
                multiReader = null;
            }


            if ( kinect != null ) {
                kinect.Close();
                kinect = null;
            }
        }

        async void multiReader_MultiSourceFrameArrived( MultiSourceFrameReader sender,
                                                    MultiSourceFrameArrivedEventArgs args )
        {
            var multiFrame = args.FrameReference.AcquireFrame();
            if ( multiFrame == null ) {
                return;
            }

            // 各種データを取得する
            UpdateColorFrame( multiFrame );
            UpdateBodyIndexFrame( multiFrame );
            UpdateDepthFrame( multiFrame );

            // それぞれの座標系で描画する
            if ( IsColorCoodinate.IsChecked == true ) {
                await DrawColorCoodinate();
            }
            else {
                await DrawDepthCoodinate();
            }
        }


        private void UpdateColorFrame( MultiSourceFrame multiFrame )
        {
            using ( var colorFrame = multiFrame.ColorFrameReference.AcquireFrame() ) {
                if ( colorFrame == null ) {
                    return;
                }

                // BGRAデータを取得する
                colorFrame.CopyConvertedFrameDataToArray(
                                            colorBuffer, colorFormat );
            }
        }

        private void UpdateDepthFrame( MultiSourceFrame multiFrame )
        {
            using ( var depthFrame = multiFrame.DepthFrameReference.AcquireFrame() ) {
                if ( depthFrame == null ) {
                    return;
                }

                // Depthデータを取得する
                depthFrame.CopyFrameDataToArray( depthBuffer );
            }
        }

        private void UpdateBodyIndexFrame( MultiSourceFrame multiFrame )
        {
            using ( var bodyIndexFrame = multiFrame.BodyIndexFrameReference.AcquireFrame() ) {
                if ( bodyIndexFrame == null ) {
                    return;
                }

                // ボディインデックスデータを取得する
                bodyIndexFrame.CopyFrameDataToArray( bodyIndexBuffer );
            }
            return;
        }

        private async Task DrawColorCoodinate()
        {
            // カラー画像の解像度でデータを作る
            var colorImageBuffer = new byte[colorFrameDesc.LengthInPixels *
                                            colorFrameDesc.BytesPerPixel];

            // 変換処理が重いため、非同期に行う
            await Task.Factory.StartNew( () =>
            {
                // カラー座標系に対応するDepth座標系の一覧を取得する
                var depthSpace = new DepthSpacePoint[colorFrameDesc.LengthInPixels];
                kinect.CoordinateMapper.MapColorFrameToDepthSpace( depthBuffer, depthSpace );

                // 並列で処理する
                Parallel.For( 0, colorFrameDesc.LengthInPixels, i =>
                {
                    int depthX = (int)depthSpace[i].X;
                    int depthY = (int)depthSpace[i].Y;
                    if ( (depthX < 0) || (depthFrameDesc.Width <= depthX) ||
                             (depthY < 0) || (depthFrameDesc.Height <= depthY) ) {
                        return;
                    }

                    // Depth座標系のインデックス
                    int depthIndex = (depthY * depthFrameDesc.Width) + depthX;
                    int bodyIndex = bodyIndexBuffer[depthIndex];

                    // 人を検出した位置だけ色を付ける
                    if ( bodyIndex == 255 ) {
                        return;
                    }

                    // カラー画像を設定する
                    int colorImageIndex = (int)(i * colorFrameDesc.BytesPerPixel);
                    colorImageBuffer[colorImageIndex + 0] = colorBuffer[colorImageIndex + 0];
                    colorImageBuffer[colorImageIndex + 1] = colorBuffer[colorImageIndex + 1];
                    colorImageBuffer[colorImageIndex + 2] = colorBuffer[colorImageIndex + 2];
                } );
            } );

            // ビットマップにする
            var colorBitmap = new WriteableBitmap( colorFrameDesc.Width,
                                                   colorFrameDesc.Height );
            var stream = colorBitmap.PixelBuffer.AsStream();
            stream.Write( colorImageBuffer, 0, colorImageBuffer.Length );
            colorBitmap.Invalidate();
            ImageColor.Source = colorBitmap;
        }

        Stopwatch sw = new Stopwatch();

        private async Task DrawDepthCoodinate()
        {
            // Depth画像の解像度でデータを作る
            var colorImageBuffer = new byte[depthFrameDesc.LengthInPixels *
                                            colorFrameDesc.BytesPerPixel];

            // 変換処理が重いため、非同期に行う
            await Task.Factory.StartNew( () =>
            {
                // Depth座標系に対応するカラー座標系の一覧を取得する
                var colorSpace = new ColorSpacePoint[depthFrameDesc.LengthInPixels];
                kinect.CoordinateMapper.MapDepthFrameToColorSpace( depthBuffer, colorSpace );

                // 並列で処理する
                Parallel.For( 0, depthFrameDesc.LengthInPixels, i =>
                {
                    int colorX = (int)colorSpace[i].X;
                    int colorY = (int)colorSpace[i].Y;
                    if ( (colorX < 0) || (colorFrameDesc.Width <= colorX) ||
                     (colorY < 0) || (colorFrameDesc.Height <= colorY) ) {
                        return;
                    }

                    // カラー座標系のインデックス
                    int colorIndex = (colorY * colorFrameDesc.Width) + colorX;
                    int bodyIndex = bodyIndexBuffer[i];

                    // 人を検出した位置だけ色を付ける
                    if ( bodyIndex == 255 ) {
                        return;
                    }

                    // カラー画像を設定する
                    int colorImageIndex = (int)(i * colorFrameDesc.BytesPerPixel);
                    int colorBufferIndex = (int)(colorIndex * colorFrameDesc.BytesPerPixel);
                    colorImageBuffer[colorImageIndex + 0] = colorBuffer[colorBufferIndex + 0];
                    colorImageBuffer[colorImageIndex + 1] = colorBuffer[colorBufferIndex + 1];
                    colorImageBuffer[colorImageIndex + 2] = colorBuffer[colorBufferIndex + 2];
                } );
            } );

            // ビットマップにする
            var colorBitmap = new WriteableBitmap( depthFrameDesc.Width,
                                                   depthFrameDesc.Height );
            var stream = colorBitmap.PixelBuffer.AsStream();
            stream.Write( colorImageBuffer, 0, colorImageBuffer.Length );
            colorBitmap.Invalidate();
            ImageColor.Source = colorBitmap;
        }
    }
}
