using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
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
        // Kinect SDK
        KinectSensor kinect;
        AudioBeamFrameReader audioBeamFrameReader;

        // 音声データ
        byte[] audioBuffer;
        WaveFile waveFile = new WaveFile();

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded( object sender, RoutedEventArgs e )
        {
            try {
                // Kinectを開く
                kinect = KinectSensor.GetDefault();
                kinect.Open();

                // 音声バッファを作成する
                audioBuffer = new byte[kinect.AudioSource.SubFrameLengthInBytes];

                // 音声リーダーを開く
                audioBeamFrameReader = kinect.AudioSource.OpenReader();
                audioBeamFrameReader.FrameArrived += audioBeamFrameReader_FrameArrived;
            }
            catch ( Exception ex ) {
                MessageBox.Show( ex.Message );
                Close();
            }
        }

        void audioBeamFrameReader_FrameArrived( object sender,
            AudioBeamFrameArrivedEventArgs e )
        {
            using ( var audioFrame =
                e.FrameReference.AcquireBeamFrames() as AudioBeamFrameList ) {

                if ( audioFrame == null ) {
                    return;
                }

                for ( int i = 0; i < audioFrame.Count; i++ ) {
                    using ( var frame = audioFrame[i] ) {
                        Trace.WriteLine( frame.SubFrames.Count );
                        for ( int j = 0; j < frame.SubFrames.Count; j++ ) {
                            using ( var subFrame = frame.SubFrames[j] ) {
                                subFrame.CopyFrameDataToArray( audioBuffer );

                                waveFile.Write( audioBuffer );

                                // 参考:実際のデータは32bit IEEE floatデータ
                                //float data1 = BitConverter.ToSingle( audioBuffer, 0 );
                                //float data2 = BitConverter.ToSingle( audioBuffer, 4 );
                                //float data3 = BitConverter.ToSingle( audioBuffer, 8 );
                            }
                        }
                    }
                }
            }
        }

        private void Window_Closing( object sender,
                                    System.ComponentModel.CancelEventArgs e )
        {
            if ( waveFile  != null ) {
                waveFile.Dispose();
                waveFile = null;
            }

            if ( audioBeamFrameReader != null ) {
                audioBeamFrameReader.Dispose();
                audioBeamFrameReader = null;
            }

            if ( kinect != null ) {
                kinect.Close();
                kinect = null;
            }
        }


        private void Button_Click( object sender, RoutedEventArgs e )
        {
            waveFile.Open( "KinectAudio.wav" );
        }

        private void Button_Click_1( object sender, RoutedEventArgs e )
        {
            waveFile.Close();
        }
    }
}
