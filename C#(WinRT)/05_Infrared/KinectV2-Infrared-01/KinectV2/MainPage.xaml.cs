using System;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.UI.Popups;
using Windows.UI.Xaml.Controls;
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

        InfraredFrameReader infraredFrameReader;
        FrameDescription infraredFrameDesc;
        ushort[] infraredBuffer;

        // 表示用
        byte[] infraredBitmapBuffer;
        WriteableBitmap infraredBitmap;

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
                if ( kinect == null ) {
                    throw new Exception( "Kinectを開けません" );
                }

                kinect.Open();

                // 赤外線画像の情報を取得する
                infraredFrameDesc = kinect.InfraredFrameSource.FrameDescription;

                // 画像化のためのバッファを作成する
                infraredBitmapBuffer = new byte[infraredFrameDesc.LengthInPixels * 4];
                infraredBitmap = new WriteableBitmap(
                    infraredFrameDesc.Width, infraredFrameDesc.Height );
                ImageInfrared.Source = infraredBitmap;

                infraredBuffer = new ushort[infraredFrameDesc.LengthInPixels];

                // 赤外線画像リーダーを開く
                infraredFrameReader = kinect.InfraredFrameSource.OpenReader();
                infraredFrameReader.FrameArrived += infraredFrameReader_FrameArrived;
            }
            catch ( Exception ex ) {
                MessageDialog dlg = new MessageDialog( ex.Message );
                dlg.ShowAsync();
            }
        }

        void infraredFrameReader_FrameArrived(
            InfraredFrameReader sender, InfraredFrameArrivedEventArgs args )
        {
            UpdateInfraredFrame( args );
            DrawInfraredFrame();
        }

        private void UpdateInfraredFrame( InfraredFrameArrivedEventArgs args )
        {
            // 赤外線画像フレームを取得する
            using ( var infraredFrame = args.FrameReference.AcquireFrame() ) {
                if ( infraredFrame == null ) {
                    return;
                }

                // 赤外線画像データを取得する
                infraredFrame.CopyFrameDataToArray( infraredBuffer );
            }
        }

        private void DrawInfraredFrame()
        {
            // 赤外線画像データをBGRAデータに変換する
            for ( int i = 0; i < infraredBuffer.Length; i++ ) {
                // 0-65535を0-255に変換する
                byte value = (byte)(infraredBuffer[i] * 255 / 65535);

                int colorindex = i * 4;
                infraredBitmapBuffer[colorindex + 0] = value;
                infraredBitmapBuffer[colorindex + 1] = value;
                infraredBitmapBuffer[colorindex + 2] = value;
                infraredBitmapBuffer[colorindex + 3] = 255;
            }

            // ビットマップにする
            var stream = infraredBitmap.PixelBuffer.AsStream();
            stream.Write( infraredBitmapBuffer, 0, infraredBitmapBuffer.Length );
            infraredBitmap.Invalidate();
        }
    }
}
