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
        KinectSensor kinect;

        ColorFrameReader colorFrameReader;
        FrameDescription colorFrameDesc;
        byte[] colorBuffer;
        WriteableBitmap colorBitmap;

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

                // 挿抜検出イベントを設定
                kinect.IsAvailableChanged += kinect_IsAvailableChanged;
            }
            catch ( Exception ex ) {
                MessageDialog dlg = new MessageDialog(ex.Message);
                dlg.ShowAsync();
            }
        }

        void kinect_IsAvailableChanged( KinectSensor sender, IsAvailableChangedEventArgs args )
        {
            // Kinectが接続された
            if ( args.IsAvailable ) {
                // カラーを設定する
                if ( colorFrameReader == null ) {
                    // カラー画像の情報を作成する(BGRAフォーマット)
                    colorFrameDesc = kinect.ColorFrameSource.CreateFrameDescription( ColorImageFormat.Bgra );

                    colorBitmap = new WriteableBitmap( colorFrameDesc.Width, colorFrameDesc.Height );
                    ImageColor.Source = colorBitmap;

                    colorBuffer = new byte[colorFrameDesc.Width * colorFrameDesc.Height * colorFrameDesc.BytesPerPixel];

                    // カラーリーダーを開く
                    colorFrameReader = kinect.ColorFrameSource.OpenReader();
                    colorFrameReader.FrameArrived += colorFrameReader_FrameArrived;
                }

                ImageColor.Source = colorBitmap;

                TextStatus.Text = "Kinectが接続されました";
            }
            // Kinectが外された
            else {
                // イメージを初期化する
                ImageColor.Source = null;

                TextStatus.Text = "Kinectが外されました";
            }
        }

        void colorFrameReader_FrameArrived( ColorFrameReader sender, ColorFrameArrivedEventArgs args )
        {
            using ( var colorFrame = args.FrameReference.AcquireFrame()){
                // BGRAデータを取得する
                colorFrame.CopyConvertedFrameDataToArray( colorBuffer, ColorImageFormat.Bgra );

                // ビットマップにする
                var stream = colorBitmap.PixelBuffer.AsStream();
                stream.Write( colorBuffer, 0, colorBuffer.Length );
                colorBitmap.Invalidate();
            }
        }
    }
}
