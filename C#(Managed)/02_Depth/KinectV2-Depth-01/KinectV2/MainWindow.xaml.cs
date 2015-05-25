using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;

namespace KinectV2
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        // Kinect SDK
        KinectSensor kinect;

        DepthFrameReader depthFrameReader;
        FrameDescription depthFrameDesc;

        // 表示
        WriteableBitmap depthImage;
        ushort[] depthBuffer;
        byte[] depthBitmapBuffer;
        Int32Rect depthRect;
        int depthStride;

        Point depthPoint;
        const int R = 20;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded( object sender, RoutedEventArgs e )
        {
            try {
                // Kinectを開く
                kinect = KinectSensor.GetDefault();
                kinect.Open();

                // 表示のためのデータを作成
                depthFrameDesc = kinect.DepthFrameSource.FrameDescription;

                // 表示のためのビットマップに必要なものを作成
                depthImage = new WriteableBitmap( 
                    depthFrameDesc.Width, depthFrameDesc.Height,
                    96, 96, PixelFormats.Gray8, null );
                depthBuffer = new ushort[depthFrameDesc.LengthInPixels];
                depthBitmapBuffer = new byte[depthFrameDesc.LengthInPixels];
                depthRect = new Int32Rect( 0, 0,
                                        depthFrameDesc.Width, depthFrameDesc.Height );
                depthStride = (int)(depthFrameDesc.Width);

                ImageDepth.Source = depthImage;

                // 初期の位置表示座標(中心点)
                depthPoint = new Point( depthFrameDesc.Width / 2,
                                        depthFrameDesc.Height / 2 );

                // Depthリーダーを開く
                depthFrameReader = kinect.DepthFrameSource.OpenReader();
                depthFrameReader.FrameArrived += depthFrameReader_FrameArrived;
            }
            catch ( Exception ex ) {
                MessageBox.Show( ex.Message );
                Close();
            }
        }

        private void Window_Closing( object sender, System.ComponentModel.CancelEventArgs e )
        {
            if ( depthFrameReader != null ) {
                depthFrameReader.Dispose();
                depthFrameReader = null;
            }

            if ( kinect != null ) {
                kinect.Close();
                kinect = null;
            }
        }

        void depthFrameReader_FrameArrived( object sender, DepthFrameArrivedEventArgs e )
        {
            UpdateDepthFrame(e);
            DrawDepthFrame();
        }

        // Depthフレームの更新
        private void UpdateDepthFrame( DepthFrameArrivedEventArgs e )
        {
            using ( var depthFrame = e.FrameReference.AcquireFrame() ) {
                if ( depthFrame == null ) {
                    return;
                }

                // Depthデータを取得する
                depthFrame.CopyFrameDataToArray( depthBuffer );
            }
        }

        // Depthフレームの表示
        private void DrawDepthFrame()
        {
            // 距離情報の表示を更新する
            UpdateDepthValue();

            // 0-8000のデータを255ごとに折り返すようにする(見やすく)
            for ( int i = 0; i < depthBuffer.Length; i++ ) {
                depthBitmapBuffer[i] = (byte)(depthBuffer[i] % 255);
            }

            depthImage.WritePixels( depthRect, depthBitmapBuffer, depthStride, 0 );
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
                Stroke = Brushes.Red,
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
                Foreground = Brushes.Green,
            };
            Canvas.SetLeft( text, depthPoint.X );
            Canvas.SetTop( text, depthPoint.Y - R );
            CanvasPoint.Children.Add( text );
        }

        private void Window_MouseLeftButtonDown( object sender, MouseButtonEventArgs e )
        {
            depthPoint = e.GetPosition( this );
        }
    }
}
