using UnityEngine;

public static class VectorExtensions
{
    public static Quaternion ToQuaternion( this Windows.Kinect.Vector4 vactor,
                                                                Quaternion comp )
    {
        return Quaternion.Inverse( comp ) * 
                    new Quaternion( -vactor.X, -vactor.Y, vactor.Z, vactor.W );
    }

    public static Windows.Kinect.Vector4 ToMirror( this Windows.Kinect.Vector4 vector )
    {
        return new Windows.Kinect.Vector4()
        {
            X = vector.X,
            Y = -vector.Y,
            Z = -vector.Z,
            W = vector.W
        };
    }
}
