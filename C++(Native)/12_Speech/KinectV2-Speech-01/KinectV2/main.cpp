#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <atlbase.h>

#include <Windows.h>
#include <Kinect.h>
// Quote from Kinect for Windows SDK v2.0 - Sample/Native/SpeechBasics-D2D
// KinectAudioStream.h and .cpp : Copyright (c) Microsoft Corporation.  All rights reserved.
#include "KinectAudioStream.h"

// Speech
#include <sapi.h>
#pragma warning(disable: 4996) // for error GetVersionExW() of sphelper.h
#include <sphelper.h> // for SpFindBestToken()
#include <locale.h>

#define ERROR_CHECK( ret )											\
	if( FAILED( ret ) ){											\
		std::stringstream ss;										\
		ss << "failed " #ret " " << std::hex << ret << std::endl;	\
		throw std::runtime_error( ss.str().c_str() );				\
	}

class KinectApp
{
private:

	CComPtr<IKinectSensor> kinect;

	CComPtr<IAudioBeam> audioBeam;
	CComPtr<IStream> inputStream;
	CComPtr<KinectAudioStream> audioStream;
	CComPtr<ISpStream> speechStream;
	CComPtr<ISpRecognizer> speechRecognizer;
	CComPtr<ISpRecoContext> speechContext;
	std::vector<CComPtr<ISpRecoGrammar>> speechGrammar;
	HANDLE speechEvent;
	const float confidenceThreshold = 0.3f;
	bool exit = false;

public:

	void initialize()
	{
		// Sensorを取得
		ERROR_CHECK( GetDefaultKinectSensor( &kinect ) );

		ERROR_CHECK( kinect->Open() );

		BOOLEAN isOpen;
		ERROR_CHECK( kinect->get_IsOpen( &isOpen ) );
		if( !isOpen ){
			throw std::runtime_error( "failed IKinectSensor::get_IsOpen( &isOpen )" );
		}

		// Audioの初期化
		initializeAudio();
	}

	void run()
	{
		start();
		while( 1 ){
			update();

			if( GetKeyState( VK_ESCAPE ) < 0 || exit ){
				break;
			}
		}
		stop();
	}

private:

	inline void initializeAudio(){
		// Audio Sourceの取得
		CComPtr<IAudioSource> audioSource;
		ERROR_CHECK( kinect->get_AudioSource( &audioSource ) );

		// Audio Beam Listの取得、Audio Beamを開く
		CComPtr<IAudioBeamList> audioBeamList;
		ERROR_CHECK( audioSource->get_AudioBeams( &audioBeamList ) );
		ERROR_CHECK( audioBeamList->OpenAudioBeam( 0, &audioBeam ) );

		// Audio Input Streamを開く
		ERROR_CHECK( audioBeam->OpenInputStream( &inputStream ) );
		audioStream = new KinectAudioStream( inputStream );

		// Speech Streamの初期化
		initializeSpeechStream();

		// Speech Recognizerの作成
		// "en-US" ... English, "ja-JP" ... Japanese
		createSpeechRecognizer( "ja-JP" );

		// ファイル(*.grxml)からSpeech Recognition Grammarを読み込む
		// Grammar ID, Grammar File Name
		loadSpeechGrammar( 0, L"Grammar_jaJP.grxml" );
		/*loadSpeechGrammar( 1, L"Grammar_Additional.grxml" );*/
	}

	inline void initializeSpeechStream()
	{
		// Speech Streamインスタンスの作成
		ERROR_CHECK( CoCreateInstance( CLSID_SpStream, NULL, CLSCTX_INPROC_SERVER, __uuidof( ISpStream ), reinterpret_cast<void**>( &speechStream ) ) );

		// マイクのWave Formatを設定
		WORD AudioFormat = WAVE_FORMAT_PCM;
		WORD AudioChannels = 1;
		DWORD AudioSamplesPerSecond = 16000;
		DWORD AudioAverageBytesPerSecond = 32000;
		WORD AudioBlockAlign = 2;
		WORD AudioBitsPerSample = 16;

		WAVEFORMATEX waveFormat = { AudioFormat, AudioChannels, AudioSamplesPerSecond, AudioAverageBytesPerSecond, AudioBlockAlign, AudioBitsPerSample, 0 };

		// Speech Streamの初期化
		ERROR_CHECK( speechStream->SetBaseStream( audioStream, SPDFID_WaveFormatEx, &waveFormat ) );
	}

	inline void createSpeechRecognizer( const std::string& language = "en-US" )
	{
		// Speech Recognizerインスタンスの作成
		ERROR_CHECK( CoCreateInstance( CLSID_SpInprocRecognizer, NULL, CLSCTX_INPROC_SERVER, __uuidof( ISpRecognizer ), reinterpret_cast<void**>( &speechRecognizer ) ) );

		// Speech RecognizerのInput Streamを登録
		ERROR_CHECK( speechRecognizer->SetInput( speechStream, TRUE ) );
		
		// Recognizer Engineの言語属性を取得
		// Kinect for Windows SDK 2.0 Language Packs http://www.microsoft.com/en-us/download/details.aspx?id=43662
		// L"Language=409;Kinect=True" ... English | United States (MSKinectLangPack_enUS.msi)
		// L"Language=411;Kinect=True" ... Japanese | Japan (MSKinectLangPack_jaJP.msi)
		// Other Languages Hexadecimal Value, Please see here https://msdn.microsoft.com/en-us/library/hh378476(v=office.14).aspx
		std::wstring attribute;
		if( language == "de-DE" ){
			attribute = L"Language=C07;Kinect=True";
		}
		else if( language == "en-AU" ){
			attribute = L"Language=C09;Kinect=True";
		}
		else if( language == "en-CA" ){
			attribute = L"Language=1009;Kinect=True";
		}
		else if( language == "en-GB" ){
			attribute = L"Language=809;Kinect=True";
		}
		else if( language == "en-IE" ){
			attribute = L"Language=1809;Kinect=True";
		}
		else if( language == "en-NZ" ){
			attribute = L"Language=1409;Kinect=True";
		}
		else if( language == "en-US" ){
			attribute = L"Language=409;Kinect=True";
		}
		else if( language == "es-ES" ){
			attribute = L"Language=2C0A;Kinect=True";
		}
		else if( language == "es-MX" ){
			attribute = L"Language=80A;Kinect=True";
		}
		else if( language == "fr-CA" ){
			attribute = L"Language=C0C;Kinect=True";
		}
		else if( language == "fr-FR" ){
			attribute = L"Language=40c;Kinect=True";
		}
		else if( language == "it-IT" ){
			attribute = L"Language=410;Kinect=True";
		}
		else if( language == "ja-JP" ){
			attribute = L"Language=411;Kinect=True";
		}
		else{
			throw std::runtime_error( "failed " __FUNCTION__ );
		}

		// Localの設定
		setlocale( LC_CTYPE, language.c_str() );

		// Speech Recognizer Engineの取得、登録
		CComPtr<ISpObjectToken> engineToken;
		ERROR_CHECK( SpFindBestToken( SPCAT_RECOGNIZERS, attribute.c_str(), NULL, &engineToken ) );
		ERROR_CHECK( speechRecognizer->SetRecognizer( engineToken ) );

		// Speech Recognizer Contextの作成
		ERROR_CHECK( speechRecognizer->CreateRecoContext( &speechContext ) );

		// 音響モデルの適応をOFF(0)に設定
		// (For Long Time (few hours~) Running Program of Speech Recognition)
		ERROR_CHECK( speechRecognizer->SetPropertyNum( L"AdaptationOn", 0 ) );
	}

	inline void loadSpeechGrammar( const ULONGLONG id, const std::wstring& grammar )
	{
		// ファイル(*.grxml)からSpeech Recognition Grammarを読み込む
		speechGrammar.push_back( nullptr );
		ERROR_CHECK( speechContext->CreateGrammar( id, &speechGrammar.back() ) );
		ERROR_CHECK( speechGrammar.back()->LoadCmdFromFile( grammar.c_str(), SPLOADOPTIONS::SPLO_STATIC ) );
	}

	void start()
	{
		std::cout << "start speech recognition..." << std::endl;

		// Audio Input Streamを開始
		audioStream->SetSpeechState( true );

		// Speech Recognition Grammarを有効化
		for( const CComPtr<ISpRecoGrammar> grammar : speechGrammar ){
			ERROR_CHECK( grammar->SetRuleState( NULL, NULL, SPRULESTATE::SPRS_ACTIVE ) );
		}

		// Recognition Statusをアクティブに設定
		ERROR_CHECK( speechRecognizer->SetRecoState( SPRECOSTATE::SPRST_ACTIVE_ALWAYS ) );

		// Eventの発生タイミングを設定(Speech Recognition完了時)
		ERROR_CHECK( speechContext->SetInterest( SPFEI( SPEVENTENUM::SPEI_RECOGNITION ), SPFEI( SPEVENTENUM::SPEI_RECOGNITION ) ) );

		// Speech Recognitionの開始
		ERROR_CHECK( speechContext->Resume( 0 ) );

		// Speech Recognition Event Handleの取得
		speechEvent = speechContext->GetNotifyEventHandle();
	}

	void stop()
	{
		// Audio Input Stremの停止
		audioStream->SetSpeechState( false );

		// Recognition Statusを無効化
		ERROR_CHECK( speechRecognizer->SetRecoState( SPRECOSTATE::SPRST_INACTIVE_WITH_PURGE ) );

		// Speech Recognitionの停止
		ERROR_CHECK( speechContext->Pause( 0 ) );

		// Speech Recognition Event Handleを閉じる
		CloseHandle( speechEvent );
	}

	void update()
	{
		updateAudioFrame();
	}

	void updateAudioFrame()
	{
		ResetEvent( speechEvent );
		HANDLE events[1] = { speechEvent };
		DWORD objects = MsgWaitForMultipleObjectsEx( ARRAYSIZE( events ), events, 50, QS_ALLINPUT, MWMO_INPUTAVAILABLE );
		switch( objects ){
			case WAIT_OBJECT_0:
				// 結果を取得、表示
				result();
				break;
			default:
				break;
		}
	}

	inline void result()
	{
		// Speech Eventを取得
		SPEVENT eventStatus;
		ULONG eventFetch;
		ERROR_CHECK( speechContext->GetEvents( 1, &eventStatus, &eventFetch ) );
		while( eventFetch > 0 ){
			switch( eventStatus.eEventId ){
				case SPEVENTENUM::SPEI_RECOGNITION: // Recognition
					if( eventStatus.elParamType == SPET_LPARAM_IS_OBJECT ){
						// Speech Recognitionの結果を取得
						CComPtr<ISpRecoResult> speechResult = reinterpret_cast<ISpRecoResult*>( eventStatus.lParam );

						// Phraseの取得、表示
						// <tag>PHRASE</tag>
						SPPHRASE* phrase;
						ERROR_CHECK( speechResult->GetPhrase( &phrase ) );
						const SPPHRASEPROPERTY* semantic = phrase->pProperties->pFirstChild;
						if( semantic->SREngineConfidence > confidenceThreshold ){
							std::wstring tag = semantic->pszValue;
							std::wcout << "Phrase : " << tag;
							if( tag == L"EXIT" ){
								exit = true;
							}
						}
						else{
							std::wcout << "Phrase : ";
						}
						CoTaskMemFree( phrase );

						// Textの取得、表示
						// <item>Text</item>
						wchar_t* text;
						ERROR_CHECK( speechResult->GetText( SP_GETWHOLEPHRASE, SP_GETWHOLEPHRASE, FALSE, &text, NULL ) );
						std::wcout << "\tText : " << text << std::endl;
						CoTaskMemFree( text );
					}
					break;
				default:
					break;
			}
			ERROR_CHECK( speechContext->GetEvents( 1, &eventStatus, &eventFetch ) );
		}
	}
};

void main()
{
	try{
		ERROR_CHECK( CoInitializeEx( NULL, COINIT_MULTITHREADED ) );

		KinectApp app;
		app.initialize();
		app.run();

		CoUninitialize();
	}
	catch( std::exception& ex ){
		std::cout << ex.what() << std::endl;
	}
}