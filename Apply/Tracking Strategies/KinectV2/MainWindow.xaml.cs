using System;
using System.Collections.Generic;
using System.Linq;
using System.Reactive.Linq;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
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

        ColorFrameReader colorFrameReader;
        FrameDescription colorFrameDesc;

        ColorImageFormat colorFormat = ColorImageFormat.Bgra;

        // WPF
        WriteableBitmap colorBitmap;
        byte[] colorBuffer;
        int colorStride;
        Int32Rect colorRect;

        float widthScale;
        float heightScale;

        // ゲームリソース
        Random rand;
        List<BitmapImage> images = new List<BitmapImage>();
        IDisposable newItemTimer = null;

        /// <summary>
        /// プレイヤー選択の種類
        /// </summary>
        enum ChooseTrackingPlayerType
        {
            FirstTrackedPlayer,
            ClosestPlayer,
            BoundinBox_1P,
            BoundinBox_2P,
        }

        /// <summary>
        /// 左側のプレイヤー
        /// </summary>
        BoundingBox left = new BoundingBox()
        {
            Min = new Point3D( -1.5, 0, 1.5 ),
            Max = new Point3D( -0.5, 0, 2.0 ),
        };

        /// <summary>
        /// 中心のプレイヤー
        /// </summary>
        BoundingBox center = new BoundingBox()
        {
            Min = new Point3D( -0.5, 0, 1.5 ),
            Max = new Point3D( 0.5, 0, 2.0 ),
        };

        /// <summary>
        /// 右側のプレイヤー
        /// </summary>
        BoundingBox right = new BoundingBox()
        {
            Min = new Point3D( 0.5, 0, 1.5 ),
            Max = new Point3D( 1.5, 0, 2.0 ),
        };

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

                // カラー画像の情報を作成する(BGRAフォーマット)
                colorFrameDesc = kinect.ColorFrameSource.CreateFrameDescription(
                                                        colorFormat );

                // カラーリーダーを開く
                colorFrameReader = kinect.ColorFrameSource.OpenReader();
                colorFrameReader.FrameArrived += colorFrameReader_FrameArrived;

                // カラー用のビットマップを作成する
                colorBitmap = new WriteableBitmap(
                                    colorFrameDesc.Width, colorFrameDesc.Height,
                                    96, 96, PixelFormats.Bgra32, null );
                colorStride = colorFrameDesc.Width * (int)colorFrameDesc.BytesPerPixel;
                colorRect = new Int32Rect( 0, 0,
                                    colorFrameDesc.Width, colorFrameDesc.Height );
                colorBuffer = new byte[colorStride * colorFrameDesc.Height];
                ImageColor.Source = colorBitmap;

                // コンボボックスに設定する
                foreach(var type in Enum.GetValues(typeof(ChooseTrackingPlayerType))){
                    ComboChoosePlayerType.Items.Add( type.ToString() );
                }
                ComboChoosePlayerType.SelectedIndex = 0;

                // カラー画像の画面に対する比率を求める(Bodyの補正用)
                widthScale = (float)(colorFrameDesc.Width / ActualWidth);
                heightScale = (float)(colorFrameDesc.Height / ActualHeight);

                rand = new Random( Environment.TickCount );

                // 落とす画像をロードする
                var baseUri = @"pack://application:,,,/Images/";
                images.Add( new BitmapImage( new Uri( string.Format( "{0}{1}",
                            baseUri, "fruit_ao_ringo.png" ), UriKind.Absolute ) ) );
                images.Add( new BitmapImage( new Uri( string.Format( "{0}{1}",
                            baseUri, "fruit_budou_kyohou.png" ), UriKind.Absolute ) ) );
                images.Add( new BitmapImage( new Uri( string.Format( "{0}{1}",
                            baseUri, "fruit_momo.png" ), UriKind.Absolute ) ) );
                images.Add( new BitmapImage( new Uri( string.Format( "{0}{1}",
                            baseUri, "fruit_ringo.png" ), UriKind.Absolute ) ) );
                images.Add( new BitmapImage( new Uri( string.Format( "{0}{1}",
                            baseUri, "fruit_strawberry.png" ), UriKind.Absolute ) ) );

                // 一定時間ごとに落とすアイテムを追加するタイマー
                StartFallTimer();
            }
            catch ( Exception ex ) {
                MessageBox.Show( ex.Message );
                Close();
            }
        }

        private void Window_Closing( object sender, System.ComponentModel.CancelEventArgs e )
        {
            StopFallTimer();

            if ( bodyFrameReader != null ) {
                bodyFrameReader.Dispose();
                bodyFrameReader = null;
            }

            if ( colorFrameReader != null ) {
                colorFrameReader.Dispose();
                colorFrameReader = null;
            }

            if ( kinect != null ) {
                kinect.Close();
                kinect = null;
            }
        }

        void colorFrameReader_FrameArrived( object sender, ColorFrameArrivedEventArgs e )
        {
            // カラーフレームを取得する
            using ( var colorFrame = e.FrameReference.AcquireFrame() ) {
                if ( colorFrame == null ) {
                    return;
                }

                // BGRAデータを取得する
                colorFrame.CopyConvertedFrameDataToArray(
                                            colorBuffer, colorFormat );

                // ビットマップにする
                colorBitmap.WritePixels( colorRect, colorBuffer, colorStride, 0 );
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

            // 選択を列挙型に変換する
            var type = (ChooseTrackingPlayerType)Enum.Parse(
                typeof( ChooseTrackingPlayerType ), 
                ComboChoosePlayerType.SelectedItem as string ); 

            // 選択ごとの処理
            switch ( type ) {
            case ChooseTrackingPlayerType.FirstTrackedPlayer:
                FirstTrackedPlayer();
                break;
            case ChooseTrackingPlayerType.ClosestPlayer:
                ClosestPlayer();
                break;
            case ChooseTrackingPlayerType.BoundinBox_1P:
                BoundinBox_1P();
                break;
            case ChooseTrackingPlayerType.BoundinBox_2P:
                BoundinBox_2P();
                break;
            }
        }

        private void FirstTrackedPlayer()
        {
            // 最初に追跡している人を有効にする
            var body = bodies.FirstOrDefault( b => b.IsTracked );
            DrawJointPosition( body, Brushes.Green );
            HitTest( body );
        }

        private void ClosestPlayer()
        {
            // Kienctに一番近い人を有効にする
            var body = ChooseClosestBody( bodies, new CameraSpacePoint() );
            DrawJointPosition( body, Brushes.Green );
            HitTest( body );
        }

        private void BoundinBox_1P()
        {
            // 1人プレイ用の検出
            var body = ChooseBoundingBox( bodies, center );
            DrawJointPosition( body, Brushes.Green );
            HitTest( body );
        }

        private void BoundinBox_2P()
        {
            // 2人プレイ用の検出
            var body1 = ChooseBoundingBox( bodies, left );
            DrawJointPosition( body1, Brushes.Red );
            HitTest( body1 );

            var body2 = ChooseBoundingBox( bodies, right );
            DrawJointPosition( body2, Brushes.Blue );
            HitTest( body2 );
        }

        /// <summary>
        /// 1人用のプレイヤーを探す
        /// </summary>
        /// <param name="bodies"></param>
        /// <returns></returns>
        private Body ChooseBoundingBox( Body[] bodies, BoundingBox boundingbox )
        {
            // 中心位置にいる人を返す
            foreach ( var body in bodies ) {
                if ( boundingbox.IsValidPosition( body ) ) {
                    return body;
                }
            }

            return null;
        }

        /// <summary>
        /// ある点から一番近い人を追う
        /// </summary>
        /// <param name="bodies"></param>
        /// <param name="center"></param>
        /// <param name="closestDistance"></param>
        /// <returns></returns>
        private Body ChooseClosestBody( Body[] bodies,
            CameraSpacePoint center = new CameraSpacePoint(), float closestDistance = 2.0f )
        {
            Body closestBody = null;

            // 比較する関節位置
            var baseType = JointType.SpineBase;

            // 追跡しているBodyから選ぶ
            foreach ( var body in bodies.Where( b => b.IsTracked ) ) {
                // 比較する関節位置が追跡状態になければ対象外
                if ( body.Joints[baseType].TrackingState == TrackingState.NotTracked ) {
                    continue;
                }

                // 中心からの距離が近い人を選ぶ
                var distance = Distance( center, body.Joints[baseType].Position );
                if ( distance < closestDistance ) {
                    closestDistance = distance;
                    closestBody = body;
                }
            }

            return closestBody;
        }

        /// <summary>
        /// 2点間の距離を計算する
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        float Distance( CameraSpacePoint p1, CameraSpacePoint p2 )
        {
            return (float)Math.Sqrt( (p2.X-p1.X)*(p2.X-p1.X) + (p2.Y-p1.Y)*(p2.Y-p1.Y) + (p2.Z-p1.Z)*(p2.Z-p1.Z) );
        }

        private void DrawJointPosition( Body body, Brush brush )
        {
            if ( body == null ) {
                return;
            }

            foreach ( var joint in body.Joints ) {
                DrawEllipse( joint.Value, 20, brush );
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
            var point = kinect.CoordinateMapper.MapCameraPointToColorSpace( joint.Position );
            if ( (point.X < 0) || (point.Y < 0) ) {
                return;
            }

            // Depth座標系で円を配置する
            Canvas.SetLeft( ellipse, (point.X / widthScale) - (R / 2) );
            Canvas.SetTop( ellipse, (point.Y / heightScale) - (R / 2) );

            CanvasBody.Children.Add( ellipse );
        }

        private void HitTest(Body body)
        {
            if ( body == null ) {
                return;
            }

            HitTest( body.Joints[JointType.HandRight] );
            HitTest( body.Joints[JointType.HandLeft] );
        }

        void HitTest(Joint joint)
        {
            if ( joint.TrackingState == TrackingState.NotTracked ) {
                return;
            }

            // カメラ座標系をDepth座標系に変換する
            var point = kinect.CoordinateMapper.MapCameraPointToColorSpace( joint.Position );
            if ( (point.X < 0) || (point.Y < 0) ) {
                return;
            }

            // Jointの位置を取得
            var jointX = point.X / widthScale;
            var jointY = point.Y / heightScale;

            foreach ( Image ui in CanvasImage.Children ) {
                var x = Canvas.GetLeft( ui );
                var y = Canvas.GetTop( ui );

                // Jointの位置がImage内にある場合は
                // 当たっているのでImageを削除する
                if( (x < jointX) && (jointX < (x + ui.Width)) &&
                    (y < jointY) && (jointY < (y + ui.Height)) ){
                        CanvasImage.Children.Remove( ui );
                        break;
                }
            }
        }

        private void StartFallTimer()
        {
            StopFallTimer();

            // 位置の更新
            CompositionTarget.Rendering += CompositionTarget_Rendering;

            // 新しいアイテムを降らせる
            newItemTimer =  Observable.Interval( TimeSpan.FromMilliseconds( 500 ) )
                            .ObserveOn( SynchronizationContext.Current )
                            .Subscribe( _ =>
                            {
                                var item = new Image()
                                {
                                    Source = images[rand.Next( images.Count )],
                                    Width = 200,
                                    Height = 200,
                                };

                                const int offset = 100;
                                Canvas.SetLeft( item, rand.Next( offset,
                                    (int)(ActualWidth - item.Width - offset) ) );
                                Canvas.SetTop( item, -item.Height );

                                CanvasImage.Children.Add( item );
                            } );
        }

        void StopFallTimer()
        {
            CompositionTarget.Rendering -= CompositionTarget_Rendering;

            if ( newItemTimer != null ) {
                newItemTimer.Dispose();
                newItemTimer = null;
            }
        }

        void CompositionTarget_Rendering( object sender, EventArgs e )
        {
            List<UIElement> removals = new List<UIElement>();

            // 下に動かす
            foreach ( UIElement ui in CanvasImage.Children ) {
                var y = Canvas.GetTop( ui ) + 5;
                if ( y >= ActualHeight ) {
                    removals.Add( ui );
                }

                Canvas.SetTop( ui, y );
            }

            // 下に移動しきったアイテムは削除する
            foreach( var ui in removals){
                CanvasImage.Children.Remove( ui );
            }
        }
    }
}
