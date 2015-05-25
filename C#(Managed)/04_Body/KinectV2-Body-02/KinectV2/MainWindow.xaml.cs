using System;
using System.Collections.Generic;
using System.Diagnostics;
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

        BodyFrameReader bodyFrameReader;
        Body[] bodies;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded( object sender, RoutedEventArgs e )
        {
            try {
                kinect = KinectSensor.GetDefault();
                kinect.Open();

                // Bodyを入れる配列を作る
                bodies = new Body[kinect.BodyFrameSource.BodyCount];

                // ボディーリーダーを開く
                bodyFrameReader = kinect.BodyFrameSource.OpenReader();
                bodyFrameReader.FrameArrived += bodyFrameReader_FrameArrived;
            }
            catch ( Exception ex ) {
                MessageBox.Show( ex.Message );
                Close();
            }
        }

        private void Window_Closing( object sender, System.ComponentModel.CancelEventArgs e )
        {
            if ( bodyFrameReader != null ) {
                bodyFrameReader.Dispose();
                bodyFrameReader = null;
            }

            if ( kinect != null ) {
                kinect.Close();
                kinect = null;
            }
        }


        void bodyFrameReader_FrameArrived( object sender, BodyFrameArrivedEventArgs e )
        {
            UpdateBodyFrame( e );
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
                        DrawEllipse( joint.Value, 10, Brushes.Blue );

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
                        DrawEllipse( joint.Value, 10, Brushes.Yellow );
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
                DrawEllipse( joint, 40, new SolidColorBrush( new Color()
                {
                    R = 255,
                    G = 255,
                    A = 128
                } ) );
            }
            // チョキのような感じ
            else if ( handState == HandState.Lasso ) {
                DrawEllipse( joint, 40, new SolidColorBrush( new Color()
                {
                    R = 255,
                    B = 255,
                    A = 128
                } ) );
            }
            // 手が閉じている(グー)
            else if ( handState == HandState.Closed ) {
                DrawEllipse( joint, 40, new SolidColorBrush( new Color()
                {
                    G = 255,
                    B = 255,
                    A = 128
                } ) );
            }
        }

        private void DrawEllipse( Joint joint, int R, Brush brush )
        {
            var ellipse = new Ellipse()
            {
                Width = R,
                Height =  R,
                Fill = brush,
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
