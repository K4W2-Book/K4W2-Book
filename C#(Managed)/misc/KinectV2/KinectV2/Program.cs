using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace KinectV2
{
    class Program
    {
        static void Main( string[] args )
        {
            var kinect = KinectSensor.GetDefault();

            Console.WriteLine( kinect.IsAvailable );
            Console.WriteLine( kinect.IsOpen );

            kinect.Open();

            Console.WriteLine( kinect.IsAvailable );
            Console.WriteLine( kinect.IsOpen );
            Console.WriteLine( kinect.UniqueKinectId );
            Console.WriteLine( kinect.KinectCapabilities.ToString() );
        }
    }
}
