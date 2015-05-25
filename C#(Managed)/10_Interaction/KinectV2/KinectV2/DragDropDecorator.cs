using System;
using System.Collections.Generic;
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
using Microsoft.Kinect.Wpf.Controls;
using Microsoft.Kinect.Toolkit.Input;

namespace KinectV2
{
    /// <summary>
    /// DragDropDecoratorはCanvasの子に配置して、子にドラッグ＆ドロップしたいControlを入れる
    /// </summary>
    public class DragDropDecorator : Decorator, IKinectControl
    {
        public bool IsManipulatable
        {
            get
            {
                return true;
            }
        }
        public bool IsPressable
        {
            get
            {
                return false;
            }
        }
        public IKinectController CreateController(IInputModel inputModel, KinectRegion kinectRegion)
        {
            return new DragDropDecoratorController( inputModel, kinectRegion );
        
        }
    }
}
