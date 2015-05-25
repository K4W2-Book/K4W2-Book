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
        KinectSensor kinect;
        
        // Bodyのデータ
        BodyFrameReader bodyFrameReader;
        Body[] bodies;

        public MainPage()
        {
            this.InitializeComponent();
        }

        protected override void OnNavigatedTo( NavigationEventArgs e )
        {
            base.OnNavigatedTo( e );

            try {
                kinect = KinectSensor.GetDefault();
                if ( kinect == null ) {
                    throw new Exception( "Kinectを開けません" );
                }

                kinect.Open();

                // Body用のバッファを作成
                bodies = new Body[kinect.BodyFrameSource.BodyCount];

                // ボディリーダーを開く
                bodyFrameReader = kinect.BodyFrameSource.OpenReader();
                bodyFrameReader.FrameArrived += bodyFrameReader_FrameArrived;
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

        void bodyFrameReader_FrameArrived( BodyFrameReader sender, BodyFrameArrivedEventArgs args )
        {
            UpdateBodyFrame( args );
            DrawBodyFrame();
        }

        // ボディの更新
        private void UpdateBodyFrame( BodyFrameArrivedEventArgs e )
        {
            using ( var bodyFrame = e.FrameReference.AcquireFrame() ) {
                if ( bodyFrame == null ) {
                    return;
                }

                // ボディデータを取得する
                bodyFrame.GetAndRefreshBodyData( bodies );
            }
        }

        // ボディの表示
        private void DrawBodyFrame()
        {
            CanvasBody.Children.Clear();

            foreach ( var body in bodies.Where( b => b.IsTracked ) ) {
                foreach ( var joint in body.Joints ) {
                    // 手の位置が追跡状態
                    if ( joint.Value.TrackingState == TrackingState.Tracked ) {
                        DrawEllipse( joint.Value, 10, Colors.Blue );

                        // 左手を追跡していたら、手の状態を表示する
                        if ( joint.Value.JointType == JointType.HandLeft ) {
                            DrawHandState( body.Joints[JointType.HandLeft],
                                body.HandLeftConfidence, body.HandLeftState );
                        }
                        // 右手を追跡していたら、手の状態を表示する
                        else if ( joint.Value.JointType == JointType.HandRight ) {
                            DrawHandState( body.Joints[JointType.HandRight],
                                body.HandRightConfidence, body.HandRightState );
                        }
                    }
                    // 手の位置が推測状態
                    else if ( joint.Value.TrackingState == TrackingState.Inferred ) {
                        DrawEllipse( joint.Value, 10, Colors.Yellow );
                    }
                }
            }
        }

        private void DrawHandState( Joint joint,
                        TrackingConfidence trackingConfidence, HandState handState )
        {
            // 手の追跡信頼性が高い
            if ( trackingConfidence != TrackingConfidence.High ) {
                return;
            }

            // 手が開いている(パー)
            if ( handState == HandState.Open ) {
                DrawEllipse( joint, 40, new Color()
                {
                    R = 255,
                    G = 255,
                    A = 128
                } );
            }
            // チョキのような感じ
            else if ( handState == HandState.Lasso ) {
                DrawEllipse( joint, 40, new Color()
                {
                    R = 255,
                    B = 255,
                    A = 128
                } );
            }
            // 手が閉じている(グー)
            else if ( handState == HandState.Closed ) {
                DrawEllipse( joint, 40, new Color()
                {
                    G = 255,
                    B = 255,
                    A = 128
                } );
            }
        }

        private void DrawEllipse( Joint joint, int R, Color color )
        {
            var ellipse = new Ellipse()
            {
                Width = R,
                Height =  R,
                Fill = new SolidColorBrush( color ),
            };

            // カメラ座標系をDepth座標系に変換する
            var point = kinect.CoordinateMapper.MapCameraPointToDepthSpace( joint.Position );
            if ( (point.X < 0) || (point.Y < 0) ) {
                return;
            }

            // Depth座標系で円を配置する
            Canvas.SetLeft( ellipse, point.X - (R / 2) );
            Canvas.SetTop( ellipse, point.Y - (R / 2) );

            CanvasBody.Children.Add( ellipse );
        }
    }
}
