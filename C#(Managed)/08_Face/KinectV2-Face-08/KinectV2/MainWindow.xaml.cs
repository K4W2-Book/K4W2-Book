using System;
using System.Collections.Generic;
using System.Globalization;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;

namespace KinectV2
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor kinect;
        BodyFrameReader bodyFrameReader;
        Body[] bodies = null;
        int bodyCount;

        FaceFrameSource[] faceFrameSources = null;
        FaceFrameReader[] faceFrameReaders = null;
        FaceFrameResult[] faceFrameResults = null;
        List<Brush> faceBrush;

        // WPF
        DrawingGroup drawingGroup;
        DrawingImage imageSource;
        int displayWidth;
        int displayHeight;
        Rect displayRect;
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }
        public MainWindow()
        {
            drawingGroup = new DrawingGroup();
            imageSource = new DrawingImage( drawingGroup );
            this.DataContext = this;
            InitializeComponent();
        }

        void InitializeFace()
        {
            FaceFrameFeatures faceFrameFeatures =
                    FaceFrameFeatures.BoundingBoxInColorSpace
                    | FaceFrameFeatures.PointsInColorSpace
                    | FaceFrameFeatures.RotationOrientation
                    | FaceFrameFeatures.FaceEngagement
                    | FaceFrameFeatures.Glasses
                    | FaceFrameFeatures.Happy
                    | FaceFrameFeatures.LeftEyeClosed
                    | FaceFrameFeatures.RightEyeClosed
                    | FaceFrameFeatures.LookingAway
                    | FaceFrameFeatures.MouthMoved
                    | FaceFrameFeatures.MouthOpen;
            faceFrameSources = new FaceFrameSource[bodyCount];
            faceFrameReaders = new FaceFrameReader[bodyCount];
            for ( int i = 0; i < bodyCount; i++ ) {
                faceFrameSources[i] = new FaceFrameSource( kinect, 0, faceFrameFeatures );
                faceFrameReaders[i] = faceFrameSources[i].OpenReader();
                faceFrameReaders[i].FrameArrived += faceFrameReader_FrameArrived;
            }
            faceFrameResults = new FaceFrameResult[bodyCount];
            faceBrush = new List<Brush>()
                {
                    Brushes.White, 
                    Brushes.Orange,
                    Brushes.Green,
                    Brushes.Red,
                    Brushes.LightBlue,
                    Brushes.Yellow
                };
        }
        void UpdateBodyFrame( BodyFrameArrivedEventArgs e )
        {
            using ( var bodyFrame = e.FrameReference.AcquireFrame() ) {
                if ( bodyFrame == null ) {
                    return;
                }
                bodyFrame.GetAndRefreshBodyData( bodies );
                for ( int i = 0; i < bodyCount; i++ ) {
                    Body body = bodies[i];
                    if ( !body.IsTracked ) {
                        continue;
                    }
                    ulong trackingId = body.TrackingId;
                    faceFrameReaders[i].FaceFrameSource.TrackingId = trackingId;
                }
            }
        }
        void faceFrameReader_FrameArrived( object sender, FaceFrameArrivedEventArgs e )
        {
            UpdateFaceFrame( e );
        }
        void UpdateFaceFrame( FaceFrameArrivedEventArgs e )
        {
            using ( FaceFrame faceFrame = e.FrameReference.AcquireFrame() ) {
                if ( faceFrame == null ) {
                    return;
                }
                bool tracked;
                tracked = faceFrame.IsTrackingIdValid;
                if ( !tracked ) {
                    return;
                }

                FaceFrameResult faceResult = faceFrame.FaceFrameResult;
                int index = GetFaceSourceIndex( faceFrame.FaceFrameSource );
                faceFrameResults[index] = faceResult;
            }
        }
        int GetFaceSourceIndex( FaceFrameSource source )
        {
            int index = -1;
            for ( int i=0; i<bodyCount; i++ ) {
                if ( faceFrameSources[i]==source ) {
                    index = i;
                    break;
                }
            }
            return index;
        }
        void bodyFrameReader_FrameArrived( object sender, BodyFrameArrivedEventArgs e )
        {
            UpdateBodyFrame( e );
            DrawFaceFrames();
        }
        void DrawFaceFrames()
        {
            using ( DrawingContext dc = drawingGroup.Open() ) {
                dc.DrawRectangle( Brushes.Black, null, displayRect );
                for ( int i = 0; i < bodyCount; i++ ) {
                    if ( faceFrameReaders[i].FaceFrameSource.IsTrackingIdValid ) {
                        if ( faceFrameResults[i] != null ) {
                            DrawFaceFrameResult( i, faceFrameResults[i], dc );
                        }
                    }
                }
                drawingGroup.ClipGeometry = new RectangleGeometry( displayRect );
            }
        }
        void DrawFaceFrameResult( int faceIndex, FaceFrameResult faceResult, DrawingContext drawingContext )
        {
            //Brush/Pen
            Brush drawingBrush = faceBrush[0];
            if ( faceIndex<bodyCount ) {
                drawingBrush = faceBrush[faceIndex];
            }
            Pen drawingPen = new Pen( drawingBrush, 5 );

            //Face Points
            var facePoints = faceResult.FacePointsInColorSpace;
            foreach ( PointF pointF in facePoints.Values ) {
                drawingContext.DrawEllipse( null, drawingPen, new Point( pointF.X, pointF.Y ), 10, 10 );
            }

            //Bounding Box
            RectI box = faceResult.FaceBoundingBoxInColorSpace;
            int width = box.Right - box.Left;
            int height = box.Bottom - box.Top;
            Rect rect = new Rect( box.Left, box.Top, width, height );
            drawingContext.DrawRectangle( null, drawingPen, rect );
            String drawingText = String.Empty;

            //Rotation
            if ( faceResult.FaceRotationQuaternion==null ) {
                return;
            }
            Vector4 quaternion = faceResult.FaceRotationQuaternion;
            int offset = 30;
            int pitch, yaw, roll;
            quaternion2degree( quaternion, out pitch, out yaw, out roll );
            drawingText = "Pitch, Yaw, Roll : " + pitch.ToString() + ", " + yaw.ToString() + ", " + roll.ToString();
            FormattedText formattedText = new FormattedText( drawingText, CultureInfo.GetCultureInfo( "ja-JP" ), FlowDirection.LeftToRight, new Typeface( "Georgia" ), 25, drawingBrush );
            drawingContext.DrawText( formattedText, new Point( box.Left, box.Bottom + offset ) );

            //Properties
            if ( faceResult.FaceProperties!=null ) {
                foreach ( var item in faceResult.FaceProperties ) {
                    drawingText = item.Key.ToString();
                    switch ( item.Value ) {
                    case DetectionResult.Yes:
                        drawingText += " : Yes";
                        break;
                    case DetectionResult.Maybe:
                        drawingText += " : Maybe";
                        break;
                    case DetectionResult.No:
                        drawingText += " : No";
                        break;
                    case DetectionResult.Unknown:
                        drawingText += " : Unknown";
                        break;
                    default:
                        break;
                    }
                    offset += 30;
                    formattedText = new FormattedText( drawingText, CultureInfo.GetCultureInfo( "ja-JP" ), FlowDirection.LeftToRight, new Typeface( "Georgia" ), 25, drawingBrush );
                    drawingContext.DrawText( formattedText, new Point( box.Left, box.Bottom+offset ) );
                }
            }
        }
        void quaternion2degree( Vector4 quaternion, out int pitch, out int yaw, out int roll )
        {
            double w = quaternion.W;
            double x = quaternion.X;
            double y = quaternion.Y;
            double z = quaternion.Z;
            pitch = (int)(Math.Atan2( 2 * (y * z + w * x), w * w - x * x - y * y + z * z ) / Math.PI * 180.0);
            yaw = (int)(Math.Asin( 2 * (w * y - x * z) ) / Math.PI * 180.0);
            roll = (int)(Math.Atan2( 2 * (x * y + w * z), w * w + x * x - y * y - z * z ) / Math.PI * 180.0);
        }
        private void Window_Closing( object sender, System.ComponentModel.CancelEventArgs e )
        {
            for ( int i = 0; i < bodyCount; i++ ) {
                if ( faceFrameReaders[i]!=null ) {
                    faceFrameReaders[i].Dispose();
                    faceFrameReaders[i] = null;
                }
                if ( faceFrameSources[i]!=null ) {
                    faceFrameSources[i].Dispose();
                    faceFrameSources[i] = null;
                }
            }
            if ( bodyFrameReader!=null ) {
                bodyFrameReader.Dispose();
                bodyFrameReader = null;
            }
            if ( kinect!=null ) {
                kinect.Close();
                kinect = null;
            }
        }
        private void Window_Loaded( object sender, RoutedEventArgs e )
        {
            try {
                kinect = KinectSensor.GetDefault();
                if ( kinect==null ) {
                    throw new Exception( "Kinectを開けません" );
                }
                kinect.Open();
                FrameDescription frameDescription = kinect.ColorFrameSource.FrameDescription;
                displayWidth = frameDescription.Width;
                displayHeight = frameDescription.Height;
                displayRect = new Rect( 0, 0, displayWidth, displayHeight );

                bodyFrameReader = kinect.BodyFrameSource.OpenReader();
                bodyFrameReader.FrameArrived += bodyFrameReader_FrameArrived;
                bodyCount = kinect.BodyFrameSource.BodyCount;
                bodies = new Body[bodyCount];

                InitializeFace();
            }
            catch ( Exception ex ) {
                MessageBox.Show( ex.Message );
                Close();
            }
        }
    }
}
