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
        KinectSensor kinect;
        ColorFrameReader colorFrameReader;

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

                // 挿抜検出イベントを設定
                kinect.IsAvailableChanged += kinect_IsAvailableChanged;
            }
            catch ( Exception ex ) {
                MessageBox.Show( ex.Message );
                Close();
            }
        }

        // Kinectの挿抜イベント
        void kinect_IsAvailableChanged( object sender, IsAvailableChangedEventArgs e )
        {
            // Kinectが接続された
            if ( e.IsAvailable ) {
                // カラーを設定する
                if ( colorFrameReader == null ) {
                    colorFrameReader = kinect.ColorFrameSource.OpenReader();
                    colorFrameReader.FrameArrived += colorFrameReader_FrameArrived;
                }

                TextStatus.Text = "Kinectが接続されました";
            }
            // Kinectが外された
            else {
                // イメージを初期化する
                ImageColor.Source = null;

                TextStatus.Text = "Kinectが外されました";
            }
        }

        private void Window_Closing( object sender, System.ComponentModel.CancelEventArgs e )
        {
            if ( colorFrameReader != null ) {
                colorFrameReader.Dispose();
                colorFrameReader = null;
            }

            if ( kinect != null ) {
                kinect.IsAvailableChanged -= kinect_IsAvailableChanged;
                kinect.Close();
                kinect = null;
            }
        }

        void colorFrameReader_FrameArrived( object sender, ColorFrameArrivedEventArgs e )
        {
            using ( var colorFrame = e.FrameReference.AcquireFrame() ) {
                if ( colorFrame == null ) {
                    return;
                }

                byte[] colorBuffer = new byte[colorFrame.FrameDescription.Width * colorFrame.FrameDescription.Height * 4];
                colorFrame.CopyConvertedFrameDataToArray( colorBuffer, ColorImageFormat.Bgra );

                ImageColor.Source = BitmapSource.Create( colorFrame.FrameDescription.Width, colorFrame.FrameDescription.Height, 96, 96,
                    PixelFormats.Bgra32, null, colorBuffer, colorFrame.FrameDescription.Width * 4 );
            }
        }
    }
}
