using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
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
        // Kinect SDK
        KinectSensor kinect;

        ColorFrameReader colorFrameReader;
        FrameDescription colorFrameDesc;

        ColorImageFormat colorFormat = ColorImageFormat.Bgra;

        // WinRT
        WriteableBitmap colorBitmap;
        byte[] colorBuffer;

        public MainPage()
        {
            this.InitializeComponent();
        }

        protected override void OnNavigatedTo( NavigationEventArgs e )
        {
            base.OnNavigatedTo( e );

            try {
                // Kinectを開く
                kinect = KinectSensor.GetDefault();
                kinect.Open();

                // カラー画像の情報を作成する(BGRAフォーマット)
                colorFrameDesc = kinect.ColorFrameSource.CreateFrameDescription(
                                                                        colorFormat );

                // カラーリーダーを開く
                colorFrameReader = kinect.ColorFrameSource.OpenReader();
                colorFrameReader.FrameArrived += colorFrameReader_FrameArrived;

                // 表示用のデータを作成
                colorBitmap = new WriteableBitmap( colorFrameDesc.Width,
                                                    colorFrameDesc.Height );
                colorBuffer = new byte[colorFrameDesc.LengthInPixels *
                                        colorFrameDesc.BytesPerPixel];
                ImageColor.Source = colorBitmap;
            }
            catch ( Exception ex ) {
                MessageDialog dlg = new MessageDialog( ex.Message );
                dlg.ShowAsync();
            }
        }

        protected override void OnNavigatedFrom( NavigationEventArgs e )
        {
            base.OnNavigatedFrom( e );

            if ( colorFrameReader != null ) {
                colorFrameReader.Dispose();
                colorFrameReader = null;
            }

            if ( kinect != null ) {
                kinect.Close();
                colorFrameReader = null;
            }
        }

        void colorFrameReader_FrameArrived( ColorFrameReader sender,
                                    ColorFrameArrivedEventArgs args )
        {
            UpdateColorFrame( args );
            DrawColorFrame();
        }

        private void UpdateColorFrame( ColorFrameArrivedEventArgs args )
        {

            // カラーフレームを取得する
            using ( var colorFrame = args.FrameReference.AcquireFrame() ) {
                if ( colorFrame == null ) {
                    return;
                }

                // BGRAデータを取得する
                colorFrame.CopyConvertedFrameDataToArray( colorBuffer, colorFormat );
            }
        }

        private void DrawColorFrame()
        {
            // ビットマップにする
            var stream = colorBitmap.PixelBuffer.AsStream();
            stream.Write( colorBuffer, 0, colorBuffer.Length );
            colorBitmap.Invalidate();
        }
    }
}
