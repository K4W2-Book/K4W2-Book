using System.Windows.Media.Media3D;
using Microsoft.Kinect;

namespace KinectV2
{
    public class BoundingBox
    {
        public Point3D Min;
        public Point3D Max;

        public BoundingBox()
        {
            Min = new Point3D();
            Max = new Point3D();
        }

        public bool IsValidPosition( Body body )
        {
            if ( body == null ) {
                return false;
            }

            var baseType = JointType.SpineBase;
            if ( body.Joints[baseType].TrackingState == TrackingState.NotTracked ) {
                return false;
            }

            var position = body.Joints[baseType].Position;
            return (Min.X <= position.X) && (position.X <= Max.X) &&
                   (Min.Z <= position.Z) && (position.Z <= Max.Z);
        }
    }
}
