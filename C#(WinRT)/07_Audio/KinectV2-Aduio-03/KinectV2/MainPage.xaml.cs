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

        // BodyIndex
        BodyIndexFrameReader bodyIndexFrameReader;
        FrameDescription bodyIndexFrameDesc;
        byte[] bodyIndexBuffer;

        // Body
        BodyFrameReader bodyFrameReader;
        Body[] bodies;

        // Audio
        AudioBeamFrameReader audioBeamFrameReader;

        // ビーム方向のTrackingIdとそのインデックス
        ulong AudioTrackingId = ulong.MaxValue;
        int AudioTrackingIndex = -1;

        // 表示用
        int bodyIndexColorBytesPerPixels = 4;
        byte[] bodyIndexColorBuffer;
        WriteableBitmap bodyIndexColorBitmap;

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

                // ボディーインデックスリーダーを開く
                bodyIndexFrameReader = kinect.BodyIndexFrameSource.OpenReader();
                bodyIndexFrameReader.FrameArrived += bodyIndexFrameReader_FrameArrived;

                // ボディーリーダーを開く
                bodyFrameReader = kinect.BodyFrameSource.OpenReader();
                bodyFrameReader.FrameArrived += bodyFrameReader_FrameArrived;

                // Audioリーダーを開く
                audioBeamFrameReader = kinect.AudioSource.OpenReader();
                audioBeamFrameReader.FrameArrived += audioBeamFrameReader_FrameArrived;


                // Bodyを入れる配列を作る
                bodies = new Body[kinect.BodyFrameSource.BodyCount];

                // 表示のためのデータを作成
                bodyIndexFrameDesc = kinect.DepthFrameSource.FrameDescription;

                // ボディインデックデータ用のバッファ
                bodyIndexBuffer = new byte[bodyIndexFrameDesc.LengthInPixels];

                // ビットマップ
                bodyIndexColorBitmap = new WriteableBitmap(
                    bodyIndexFrameDesc.Width, bodyIndexFrameDesc.Height );
                ImageBodyIndex.Source = bodyIndexColorBitmap;

                bodyIndexColorBuffer = new byte[bodyIndexFrameDesc.LengthInPixels *
                                                bodyIndexColorBytesPerPixels];
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

                                // ビーム角度、信頼性、ビーム方向のBody数を表示
                                TextBeamAngleConfidence.Text = 
                                    subFrame.BeamAngleConfidence.ToString();
                                TextAudioBodyCorrelations.Text = 
                                    subFrame.AudioBodyCorrelations.Count.ToString();

                                // ビーム方向に人がいれば、そのTrackibngIdを保存する
                                if ( subFrame.AudioBodyCorrelations.Count != 0 ) {
                                    AudioTrackingId =
                                        subFrame.AudioBodyCorrelations[0].BodyTrackingId;
                                }
                                else {
                                    AudioTrackingId = ulong.MaxValue;
                                }
                            }
                        }
                    }
                }
            }
        }

        void bodyFrameReader_FrameArrived( BodyFrameReader sender,
            BodyFrameArrivedEventArgs args )
        {
            // ボディデータを取得する
            using ( var bodyFrame = args.FrameReference.AcquireFrame() ) {
                if ( bodyFrame == null ) {
                    return;
                }

                bodyFrame.GetAndRefreshBodyData( bodies );
            }

            // ビーム方向と一致するTrackingIdがあれば、そのインデックス(BodyIndex)を保存する
            AudioTrackingIndex = -1;
            for ( int i = 0; i < bodies.Length; i++ ) {
                if ( bodies[i].TrackingId == AudioTrackingId ) {
                    AudioTrackingIndex = i;
                    break;
                }
            }
        }

        void bodyIndexFrameReader_FrameArrived( BodyIndexFrameReader sender, 
            BodyIndexFrameArrivedEventArgs args )
        {
            // ボディインデックスデータを取得する
            using ( var bodyIndexFrame = args.FrameReference.AcquireFrame() ) {
                if ( bodyIndexFrame == null ) {
                    return;
                }

                bodyIndexFrame.CopyFrameDataToArray( bodyIndexBuffer );
            }

            // ボディインデックスデータをBGRAデータに変換する
            for ( int i = 0; i < bodyIndexBuffer.Length; i++ ) {
                var index = bodyIndexBuffer[i];
                var colorIndex = i * 4;

                if ( index != 255 ) {
                    // BodyIndexとビーム方向のTrackingIdのインデックスが
                    // 一致している人の色を変える(青)
                    if ( index == AudioTrackingIndex ) {
                        bodyIndexColorBuffer[colorIndex + 0] = 255;
                        bodyIndexColorBuffer[colorIndex + 1] = 0;
                        bodyIndexColorBuffer[colorIndex + 2] = 0;
                        bodyIndexColorBuffer[colorIndex + 3] = 255;
                    }
                    else {
                        bodyIndexColorBuffer[colorIndex + 0] = 0;
                        bodyIndexColorBuffer[colorIndex + 1] = 0;
                        bodyIndexColorBuffer[colorIndex + 2] = 255;
                        bodyIndexColorBuffer[colorIndex + 3] = 255;
                    }
                }
                else {
                    bodyIndexColorBuffer[colorIndex + 0] = 0;
                    bodyIndexColorBuffer[colorIndex + 1] = 0;
                    bodyIndexColorBuffer[colorIndex + 2] = 0;
                    bodyIndexColorBuffer[colorIndex + 3] = 255;
                }
            }

            // ビットマップにする
            var stream = bodyIndexColorBitmap.PixelBuffer.AsStream();
            stream.Write( bodyIndexColorBuffer, 0, bodyIndexColorBuffer.Length );
            bodyIndexColorBitmap.Invalidate();
        }

        protected override void OnNavigatingFrom(
                                    NavigatingCancelEventArgs e )
        {
            base.OnNavigatingFrom( e );

            if ( bodyIndexFrameReader != null ) {
                bodyIndexFrameReader.Dispose();
                bodyIndexFrameReader = null;
            }

            if ( bodyFrameReader != null ) {
                bodyFrameReader.Dispose();
                bodyFrameReader = null;
            }

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
