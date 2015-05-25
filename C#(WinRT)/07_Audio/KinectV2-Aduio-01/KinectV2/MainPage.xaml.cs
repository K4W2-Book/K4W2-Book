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
        AudioBeamFrameReader audioBeamFrameReader;

        // 音声データ
        byte[] audioBuffer;
        WaveFile waveFile = new WaveFile();

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

                // 音声バッファを作成する
                audioBuffer = new byte[kinect.AudioSource.SubFrameLengthInBytes];

                // 音声リーダーを開く
                audioBeamFrameReader = kinect.AudioSource.OpenReader();
                audioBeamFrameReader.FrameArrived += audioBeamFrameReader_FrameArrived;
            }
            catch ( Exception ex ) {
                MessageDialog dlg = new MessageDialog(ex.Message);
                dlg.ShowAsync();
            }
        }

        void audioBeamFrameReader_FrameArrived(
            AudioBeamFrameReader sender, AudioBeamFrameArrivedEventArgs args )
        {
            using ( var audioFrame = 
                args.FrameReference.AcquireBeamFrames() as AudioBeamFrameList ) {

                if ( audioFrame == null ) {
                    return;
                }

                for ( int i = 0; i < audioFrame.Count; i++ ) {
                    using ( var frame = audioFrame[i] ) {
                        for ( int j = 0; j < frame.SubFrames.Count; j++ ) {
                            using ( var subFrame = frame.SubFrames[j] ) {
                                subFrame.CopyFrameDataToArray( audioBuffer );

                                waveFile.Write( audioBuffer );

                                // 参考:実際のデータは32bit IEEE floatデータ
                                //float data1 = BitConverter.ToSingle( audioBuffer, 0 );
                                //float data2 = BitConverter.ToSingle( audioBuffer, 4 );
                                //float data3 = BitConverter.ToSingle( audioBuffer, 8 );
                            }
                        }
                    }
                }
            }
        }

        protected override void OnNavigatingFrom( NavigatingCancelEventArgs e )
        {
            base.OnNavigatingFrom( e );

            if ( waveFile  != null ) {
                waveFile.Dispose();
                waveFile = null;
            }

            if ( audioBeamFrameReader  != null ) {
                audioBeamFrameReader.Dispose();
                audioBeamFrameReader = null;
            }

            if ( kinect != null ) {
                kinect.Close();
                kinect = null;
            }
        }

        private void Button_Click( object sender, RoutedEventArgs e )
        {
            waveFile.Open( "KinectAudio.wav" );
        }

        private void Button_Click_1( object sender, RoutedEventArgs e )
        {
            waveFile.Close();
        }
    }
}
