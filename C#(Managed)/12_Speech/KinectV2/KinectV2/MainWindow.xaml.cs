using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Windows;
using Microsoft.Kinect;
using Microsoft.Speech.AudioFormat;
using Microsoft.Speech.Recognition;

// Quoted from Kinect for Windows SDK v2.0 - Samples/Managed/SpeechBasics-WPF
// KinectAudioStream.cs : Copyright (c) Microsoft Corporation.  All rights reserved.
using Microsoft.Samples.Kinect.SpeechBasics;

namespace KinectV2
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        //Kinect
        KinectSensor kinect;

        //Audio
        KinectAudioStream convertStream;
        SpeechRecognitionEngine recognitionEngine;
        const float confidenceThreshold = 0.8f;

        public MainWindow()
        {
            InitializeComponent();
        }
        private void Window_Loaded( object sender, RoutedEventArgs e )
        {
            try {
                kinect = KinectSensor.GetDefault();
                if ( kinect == null ) {
                    throw new Exception("Kinectを開けません");
                }

                kinect.Open();

                InitializeAudio();
                InitializeRecognitionEngine("ja-JP");
                LoadGrammars(recognitionEngine);
                Start();
            }
            catch ( Exception ex ) {
                MessageBox.Show( ex.Message );
                Close();
            }
        }
        void Start()
        {
            recognitionEngine.RecognizeAsync(RecognizeMode.Multiple);
            MessageBlock.Text += "start speech recognition..." + Environment.NewLine;
        }
        void LoadGrammars(SpeechRecognitionEngine engine)
        {
            engine.LoadGrammar(ReadGrammar("Grammar_jaJP.grxml"));
            
            var colors = new Choices();
            colors.Add(new SemanticResultValue("紫", "VIOLET"));
            colors.Add(new SemanticResultValue("紫色", "VIOLET"));
            colors.Add(new SemanticResultValue("黒", "BLACK"));
            colors.Add(new SemanticResultValue("黒色", "BLACK"));
            var gb = new GrammarBuilder { Culture = engine.RecognizerInfo.Culture };
            gb.Append(colors);
            var g = new Grammar(gb);
            engine.LoadGrammar(g);
        }
        void InitializeAudio()
        {
            AudioSource audioSource = kinect.AudioSource;
            if(audioSource==null)
            {
                throw new Exception("no audio source");
            }
            IReadOnlyList<AudioBeam> audioBeamList = audioSource.AudioBeams;
            Stream inputStream = audioBeamList[0].OpenInputStream();
            convertStream = new KinectAudioStream(inputStream);
            convertStream.SpeechActive = true;
        }
        Grammar ReadGrammar(string grammarFilename)
        {
            string currentDir = Directory.GetCurrentDirectory();
            return new Grammar(currentDir+"/"+grammarFilename);
        }

        void InitializeRecognitionEngine(string cultureName = "en-US")
        {
            RecognizerInfo ri = TryGetKinectRecognizer(cultureName);
            if(ri==null)
            {
                throw new Exception("No Recognizer");
            }
            recognitionEngine = new SpeechRecognitionEngine(ri.Id);
            recognitionEngine.SpeechRecognized += recognitionEngine_SpeechRecognized;
            SpeechAudioFormatInfo speechAudioFormatInfo = new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null);
            recognitionEngine.SetInputToAudioStream(convertStream, speechAudioFormatInfo);
        }
        private static RecognizerInfo TryGetKinectRecognizer(string cultureName)
        {
            IEnumerable<RecognizerInfo> recognizers;
            try
            {
                recognizers = SpeechRecognitionEngine.InstalledRecognizers();
            }
            catch(COMException)
            {
                return null;
            }
            foreach(RecognizerInfo recognizer in recognizers)
            {
                string value;
                recognizer.AdditionalInfo.TryGetValue("Kinect", out value);
                if("True".Equals(value, StringComparison.OrdinalIgnoreCase)
                    && cultureName.Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase))
                {
                    return recognizer;
                }
            }
            return null;
        }
        void result(RecognitionResult r)
        {
            MessageBlock.Text += "Semantics : " + r.Semantics.Value.ToString()
                + ",  TEXT : " + r.Text.ToString()
                + ",  Confidence : " + r.Confidence.ToString() + Environment.NewLine;
            if (r.Semantics.Value.ToString() == "EXIT" && r.Confidence > confidenceThreshold)
            {
                this.Close();
            }
        }
        void recognitionEngine_SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            result(e.Result);
        }
        void stop()
        {
            if (convertStream != null)
            {
                convertStream.SpeechActive = false;
            }
            if (recognitionEngine != null)
            {
                recognitionEngine.SpeechRecognized -= recognitionEngine_SpeechRecognized;
                recognitionEngine.RecognizeAsyncStop();
            }
        }
        private void Window_Closing( object sender, System.ComponentModel.CancelEventArgs e )
        {
            stop();
            if ( kinect != null ) {
                kinect.Close();
                kinect = null;
            }
        }
    }
}
