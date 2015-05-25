using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Controls;
using System.Windows;
using System.Threading.Tasks;
using Microsoft.Kinect;
using Microsoft.Kinect.Toolkit.Input;
using Microsoft.Kinect.Wpf.Controls;

namespace KinectV2
{
    class DragDropDecoratorController:IKinectManipulatableController
    {
        private ManipulatableModel inputModel;
        private KinectRegion kinectRegion;
        private DragDropDecorator dragDropDecorator;
        bool disposed = false ;

        //コンストラクタ
        public DragDropDecoratorController(IInputModel _inputModel, KinectRegion _kinectRegion)
        {
            inputModel = _inputModel as ManipulatableModel;
            kinectRegion = _kinectRegion;
            dragDropDecorator = _inputModel.Element as DragDropDecorator;

            inputModel.ManipulationUpdated += inputModel_ManipulationUpdated;
        }
        
        //DragDropDecoratorのCanvas.Top,Canvas.Leftプロパティを更新
        void inputModel_ManipulationUpdated( object sender, Microsoft.Kinect.Input.KinectManipulationUpdatedEventArgs e )
        {
            //DragDropDecoratorの親はCanvas
            Canvas canvas = dragDropDecorator.Parent as Canvas;
            if(canvas!=null) {
                double y = Canvas.GetTop( dragDropDecorator );
                double x = Canvas.GetLeft( dragDropDecorator );
                if ( double.IsNaN( x ) || double.IsNaN( y ) )
                    return;
                PointF delta = e.Delta.Translation;
                //deltaは-1.0..1.0の相対値で表されているのでKinectRegionに合わせて拡大
                var Dy = delta.Y*kinectRegion.ActualHeight;
                var Dx = delta.X*kinectRegion.ActualWidth;
                Canvas.SetTop( dragDropDecorator, y+Dy );
                Canvas.SetLeft( dragDropDecorator, x+Dx );
            }
        }
        
        FrameworkElement IKinectController.Element
        {
            get
            {
                return inputModel.Element as FrameworkElement;
            }
        }
        ManipulatableModel IKinectManipulatableController.ManipulatableInputModel
        {
            get
            {
                return inputModel;
            }
        }
        void System.IDisposable.Dispose()
        {
            if ( !disposed ) {
                kinectRegion = null;
                inputModel = null;
                dragDropDecorator = null;
                inputModel.ManipulationUpdated -= inputModel_ManipulationUpdated;
                disposed = true;
            }
        }
    }
}
