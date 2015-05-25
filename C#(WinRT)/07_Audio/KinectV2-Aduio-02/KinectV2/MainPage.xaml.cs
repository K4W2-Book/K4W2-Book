using System;
using System.Collections.Generic;
using System.Diagnostics;
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

                // 音声リーダーを開く
                audioBeamFrameReader = kinect.AudioSource.OpenReader();
                audioBeamFrameReader.FrameArrived += audioBeamFrameReader_FrameArrived;
            }
            catch ( Exception ex ) {
                MessageDialog dlg = new MessageDialog(ex.Message);
                dlg.ShowAsync();
            }
        }

        // http://mtaulty.com/CommunityServer/blogs/mike_taultys_blog/archive/2014/10/01/kinect-for-windows-v2-hello-audio-world-for-the-net-windows-app-developer-amp-harmonica-player.aspx
        void audioBeamFrameReader_FrameArrived( AudioBeamFrameReader sender,
            AudioBeamFrameArrivedEventArgs args )
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
                                // 音の方向
                                LineBeamAngle.Angle =
                                            (int)(subFrame.BeamAngle * 180 / Math.PI);

                                // 音の方向の信頼性[0-1]
                                TextBeamAngleConfidence.Text = 
                                            subFrame.BeamAngleConfidence.ToString();
                            }
                        }
                    }
                }
            }
        }

        protected override void OnNavigatingFrom( NavigatingCancelEventArgs e )
        {
            base.OnNavigatingFrom( e );

            if ( audioBeamFrameReader != null ) {
                audioBeamFrameReader.Dispose();
                audioBeamFrameReader = null;
            }

            if ( kinect != null ) {
                kinect.Close();
                kinect = null;
            }
        }
    }
}
