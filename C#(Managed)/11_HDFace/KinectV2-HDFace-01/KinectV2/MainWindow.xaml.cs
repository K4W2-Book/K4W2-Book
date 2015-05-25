using System;
using System.Windows;
using System.Windows.Media;
using System.Globalization;
using System.Collections.Generic;
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
        CoordinateMapper coordinateMapper;
        //BodyFrame
        BodyFrameSource bodyFrameSource = null;
        BodyFrameReader bodyFrameReader = null;
        //HDFace
        HighDefinitionFaceFrameSource hdFaceFrameSource = null;
        HighDefinitionFaceFrameReader hdFaceFrameReader = null;
        FaceAlignment faceAlignment = null;
        FaceModel faceModel = null;
        FaceModelBuilder faceModelBuilder = null;
        bool produced = false;

        DrawingGroup drawingGroup;
        DrawingImage imageSource;
        int displayWidth;
        int displayHeight;
        Rect displayRect;

        public ImageSource ImageSource
        {
            get
            {
                return imageSource;
            }
        }

        public MainWindow()
        {
            drawingGroup = new DrawingGroup();
            imageSource = new DrawingImage( drawingGroup );
            this.DataContext = this;
            InitializeComponent();
        }

        private void Window_Loaded( object sender, RoutedEventArgs e )
        {
            try {
                kinect = KinectSensor.GetDefault();
                if ( kinect == null ) {
                    throw new Exception( "Kinectを開けません" );
                }
                coordinateMapper = kinect.CoordinateMapper;
                FrameDescription frameDescription = kinect.ColorFrameSource.FrameDescription;
                displayWidth = frameDescription.Width;
                displayHeight = frameDescription.Height;
                displayRect = new Rect( 0, 0, displayWidth, displayHeight );

                bodyFrameSource = kinect.BodyFrameSource;
                bodyFrameReader = bodyFrameSource.OpenReader();
                bodyFrameReader.FrameArrived += bodyFrameReader_FrameArrived;
                InitializeHDFace();

                kinect.Open();
            }
            catch ( Exception ex ) {
                MessageBox.Show( ex.Message );
                Close();
            }
        }

        void bodyFrameReader_FrameArrived( object sender, BodyFrameArrivedEventArgs e )
        {
            UpdateBodyFrame( e );
        }
        void UpdateBodyFrame( BodyFrameArrivedEventArgs e )
        {
            using ( BodyFrame bodyFrame = e.FrameReference.AcquireFrame() ) {
                if ( bodyFrame == null ) {
                    return;
                }
                Body[] bodies = new Body[bodyFrame.BodyCount];
                bodyFrame.GetAndRefreshBodyData( bodies );
                FindClosestBody( bodies );
            }
        }
        void FindClosestBody( Body[] bodies )
        {
            float closestDistance2 = float.MaxValue;
            Body closestBody = null;
            foreach ( Body body in bodies ) {
                if ( body==null ) {
                    continue;
                }
                Boolean tracked;
                tracked = body.IsTracked;
                if ( !tracked ) {
                    continue;
                }
                Joint head = body.Joints[JointType.Head];
                if ( head.TrackingState==TrackingState.NotTracked ) {
                    continue;
                }
                CameraSpacePoint headPoint = head.Position;
                float distance2 = headPoint.X * headPoint.X + headPoint.Y * headPoint.Y + headPoint.Z * headPoint.Z;
                if ( closestDistance2<=distance2 ) {
                    continue;
                }
                closestDistance2 = distance2;
                closestBody = body;
            }

            if ( closestBody==null ) {
                return;
            }
            ulong trackingId = closestBody.TrackingId;

            if ( hdFaceFrameSource==null ) {
                return;
            }
            hdFaceFrameSource.TrackingId = trackingId;
        }
        private void InitializeHDFace()
        {
            hdFaceFrameSource = new HighDefinitionFaceFrameSource( kinect );
            if ( hdFaceFrameSource==null ) {
                throw new Exception( "Cannot create HD Face Frame Source" );
            }
            hdFaceFrameReader = hdFaceFrameSource.OpenReader();
            hdFaceFrameReader.FrameArrived += hdFaceFrameReader_FrameArrived;
            faceModel = new FaceModel();
            faceAlignment = new FaceAlignment();

            FaceModelBuilderAttributes attributes = FaceModelBuilderAttributes.None;
            faceModelBuilder = hdFaceFrameSource.OpenModelBuilder( attributes );
            if ( faceModelBuilder==null ) {
                throw new Exception( "Cannot open Face Model Builder" );
            }
            faceModelBuilder.BeginFaceDataCollection();
            faceModelBuilder.CollectionCompleted += faceModelBuilder_CollectionCompleted;
        }
        void UpdateHDFaceFrame( HighDefinitionFaceFrameArrivedEventArgs e )
        {
            using ( var hdFaceFrame = e.FrameReference.AcquireFrame() ) {
                if ( hdFaceFrame==null ) {
                    return;
                }
                bool tracked;
                tracked = hdFaceFrame.IsFaceTracked;
                if ( !tracked ) {
                    return;
                }

                hdFaceFrame.GetAndRefreshFaceAlignmentResult( faceAlignment );

                using ( var dc = drawingGroup.Open() ) {
                    dc.DrawRectangle( Brushes.Black, null, displayRect );
                    BuildFaceModel( dc );
                    Result( dc );
                    drawingGroup.ClipGeometry = new RectangleGeometry( displayRect );
                }
            }
        }
        void BuildFaceModel( DrawingContext dc )
        {
            if ( produced ) {
                FormattedText completed_msg = new FormattedText( "Status : Completed", CultureInfo.GetCultureInfo( "ja-JP" ), FlowDirection.LeftToRight, new Typeface( "Georgia" ), 25, Brushes.Green );
                dc.DrawText( completed_msg, new Point( 50, 50 ) );
                return;
            }
            if ( faceModelBuilder == null ) {
                return;
            }
            FaceModelBuilderCollectionStatus collection;
            collection = faceModelBuilder.CollectionStatus;
            if ( collection == 0 ) {
                return;
            }
            //Collection Status
            FormattedText text = new FormattedText( "Status : " + collection.ToString(), CultureInfo.GetCultureInfo( "ja-JP" ), FlowDirection.LeftToRight, new Typeface( "Georgia" ), 25, Brushes.Green );
            dc.DrawText( text, new Point( 50, 50 ) );
            String status = status2string( collection );
            text = new FormattedText( status, CultureInfo.GetCultureInfo( "ja-JP" ), FlowDirection.LeftToRight, new Typeface( "Georgia" ), 25, Brushes.Green );
            dc.DrawText( text, new Point( 50, 80 ) );

            //Capture Status
            FaceModelBuilderCaptureStatus capture;
            capture = faceModelBuilder.CaptureStatus;
            if ( capture == 0 ) {
                return;
            }
            status = status2string( capture );
            text = new FormattedText( status, CultureInfo.GetCultureInfo( "ja-JP" ), FlowDirection.LeftToRight, new Typeface( "Georgia" ), 25, Brushes.Green );
            dc.DrawText( text, new Point( 50, 110 ) );
            return;
        }
        String status2string( FaceModelBuilderCaptureStatus capture )
        {
            String status = String.Empty;
            switch ( capture ) {
            case FaceModelBuilderCaptureStatus.FaceTooFar:
                status = "Error : Face Too Far from Camera";
                break;
            case FaceModelBuilderCaptureStatus.FaceTooNear:
                status = "Error : Face Too Near to Camera";
                break;
            case FaceModelBuilderCaptureStatus.MovingTooFast:
                status = "Error : Moving Too Fast";
                break;
            default:
                status = "";
                break;
            }
            return status;
        }
        String status2string( FaceModelBuilderCollectionStatus collection )
        {
            String status = String.Empty;
            if ( (collection & FaceModelBuilderCollectionStatus.TiltedUpViewsNeeded) != 0 ) {
                status = "Need : Tilted Up Views";
            }
            else if ( (collection & FaceModelBuilderCollectionStatus.RightViewsNeeded) != 0 ) {
                status = "Need : Right Views";
            }
            else if ( (collection & FaceModelBuilderCollectionStatus.LeftViewsNeeded) != 0 ) {
                status = "Need : Left Views";
            }
            else if ( (collection & FaceModelBuilderCollectionStatus.FrontViewFramesNeeded) != 0 ) {
                status = "Need : Front View Frames";
            }
            else if ( (collection & FaceModelBuilderCollectionStatus.MoreFramesNeeded) != 0 ) {
                status = "Need : More Frames";
            }

            return status;
        }
        void Result( DrawingContext dc )
        {
            var vertices = faceModel.CalculateVerticesForAlignment( faceAlignment );
            foreach ( CameraSpacePoint vertex in vertices ) {
                ColorSpacePoint point;
                point = coordinateMapper.MapCameraPointToColorSpace( vertex );
                int x = (int)point.X;
                int y = (int)point.Y;
                if ( (x>=0)&&(x<displayWidth)&&(y>=0)&&(y<displayHeight) ) {
                    Pen drawPen = new Pen();
                    drawPen.Brush = Brushes.White;
                    dc.DrawEllipse( null, drawPen, new Point( x, y ), 2, 2 );
                }
            }
        }
        void faceModelBuilder_CollectionCompleted( object sender, FaceModelBuilderCollectionCompletedEventArgs e )
        {
            var modelData = e.ModelData;
            faceModel = modelData.ProduceFaceModel();
            produced = true;

            faceModelBuilder.Dispose();
            faceModelBuilder = null;
        }

        void hdFaceFrameReader_FrameArrived( object sender, HighDefinitionFaceFrameArrivedEventArgs e )
        {
            UpdateHDFaceFrame( e );
        }
        private void Window_Closing( object sender, System.ComponentModel.CancelEventArgs e )
        {
            if ( faceModelBuilder != null ) {
                faceModelBuilder.Dispose();
                faceModelBuilder = null;
            }
            if ( hdFaceFrameReader != null ) {
                hdFaceFrameReader.Dispose();
                hdFaceFrameReader = null;
            }
            if ( bodyFrameReader != null ) {
                bodyFrameReader.Dispose();
                bodyFrameReader = null;
            }
            if ( faceModel!=null ) {
                faceModel.Dispose();
                faceModel = null;
            }
            if ( kinect != null ) {
                kinect.Close();
                kinect = null;
            }
        }
    }
}
