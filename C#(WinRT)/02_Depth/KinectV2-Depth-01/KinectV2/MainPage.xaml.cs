using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI;
using Windows.UI.Popups;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Media.Imaging;
using Windows.UI.Xaml.Navigation;
using Windows.UI.Xaml.Shapes;
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
        
        DepthFrameReader depthFrameReader;
        FrameDescription depthFrameDesc;

        // 表示
        ushort[] depthBuffer;
        byte[] depthBitmapBuffer;
        WriteableBitmap depthBitmap;

        Point depthPoint;
        const int R = 20;

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

                // 表示のためのデータを作成
                depthFrameDesc = kinect.DepthFrameSource.FrameDescription;

                // Depthリーダーを開く
                depthFrameReader = kinect.DepthFrameSource.OpenReader();
                depthFrameReader.FrameArrived += depthFrameReader_FrameArrived;

                // 表示のためのデータ
                depthBitmap = new WriteableBitmap( depthFrameDesc.Width,
                                                   depthFrameDesc.Height );
                ImageDepth.Source = depthBitmap;

                depthBuffer = new ushort[depthFrameDesc.LengthInPixels];
                depthBitmapBuffer = new byte[depthFrameDesc.LengthInPixels * 4];

                depthPoint = new Point( depthFrameDesc.Width / 2,
                                        depthFrameDesc.Height / 2 );
            }
            catch ( Exception ex ) {
                MessageDialog dlg = new MessageDialog(ex.Message);
                dlg.ShowAsync();
            }
        }

        protected override void OnNavigatingFrom( NavigatingCancelEventArgs e )
        {
            base.OnNavigatingFrom( e );

            if ( kinect != null ) {
                kinect.Close();
                kinect = null;
            }
        }

        void depthFrameReader_FrameArrived( DepthFrameReader sender, DepthFrameArrivedEventArgs args )
        {
            UpdateDepthFrame( args );
            DrawDepthFrame();
        }

        private void UpdateDepthFrame( DepthFrameArrivedEventArgs args )
        {
            // Depthフレームを取得する
            using ( var depthFrame = args.FrameReference.AcquireFrame() ) {
                if ( depthFrame == null ) {
                    return;
                }

                // Depthデータを取得する
                depthFrame.CopyFrameDataToArray( depthBuffer );
            }
        }

        private void DrawDepthFrame()
        {
            // ポイントされた場所の距離を表示する
            UpdateDepthValue();

            // DepthデータをBGRAデータに変換する
            for ( int i = 0; i < depthBuffer.Length; i++ ) {
                // 0-8000のデータを255ごとに折り返すようにする(見やすく)
                byte value = (byte)(depthBuffer[i] % 255);

                int colorindex = i * 4;
                depthBitmapBuffer[colorindex + 0] = value;
                depthBitmapBuffer[colorindex + 1] = value;
                depthBitmapBuffer[colorindex + 2] = value;
                depthBitmapBuffer[colorindex + 3] = 255;
            }

            // ビットマップにする
            var stream = depthBitmap.PixelBuffer.AsStream();
            stream.Write( depthBitmapBuffer, 0, depthBitmapBuffer.Length );
            depthBitmap.Invalidate();
        }

        private void UpdateDepthValue()
        {
            CanvasPoint.Children.Clear();

            // クリックしたポイントを表示する
            var ellipse = new Ellipse()
            {
                Width = R,
                Height = R,
                StrokeThickness = R / 4,
                Stroke = new SolidColorBrush( Colors.Red ),
            };
            Canvas.SetLeft( ellipse, depthPoint.X - (R / 2) );
            Canvas.SetTop( ellipse, depthPoint.Y - (R / 2) );
            CanvasPoint.Children.Add( ellipse );

            // クリックしたポイントのインデックスを計算する
            int depthindex =(int)((depthPoint.Y  * depthFrameDesc.Width) + depthPoint.X);

            // クリックしたポイントの距離を表示する
            var text = new TextBlock()
            {
                Text = string.Format( "{0}mm", depthBuffer[depthindex] ),
                FontSize = 20,
                Foreground = new SolidColorBrush( Colors.Green ),
            };

            Canvas.SetLeft( text, depthPoint.X );
            Canvas.SetTop( text, depthPoint.Y - R );
            CanvasPoint.Children.Add( text );
        }

        private void Page_PointerPressed( object sender, PointerRoutedEventArgs e )
        {
            depthPoint = e.GetCurrentPoint( CanvasPoint ).Position;
        }
    }
}
