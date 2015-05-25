// AudioCaptureRaw-Console‚æ‚èˆê•”ˆø—p‚µ‰ü•Ï
#pragma once 

#include <fstream>

#include <Windows.h>
#include <mmreg.h>

//
//  A wave file consists of:
//
//  RIFF header:    8 bytes consisting of the signature "RIFF" followed by a 4 byte file length.
//  WAVE header:    4 bytes consisting of the signature "WAVE".
//  fmt header:     4 bytes consisting of the signature "fmt " followed by a WAVEFORMATEX 
//  WAVEFORMAT:     <n> bytes containing a waveformat structure.
//  DATA header:    8 bytes consisting of the signature "data" followed by a 4 byte file length.
//  wave data:      <m> bytes containing wave data.
//

//  Header for a WAV file - we define a structure describing the first few fields in the header for convenience.
struct WAVEHEADER
{
    DWORD   dwRiff;                     // "RIFF"
    DWORD   dwSize;                     // Size
    DWORD   dwWave;                     // "WAVE"
    DWORD   dwFmt;                      // "fmt "
    DWORD   dwFmtSize;                  // Wave Format Size
};

//  Static RIFF header, we'll append the format to it.
static const BYTE WaveHeaderTemplate[] =
{
    'R', 'I', 'F', 'F', 0x00, 0x00, 0x00, 0x00, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 0x00, 0x00, 0x00, 0x00
};

//  Static wave DATA tag.
static const BYTE WaveData[] = { 'd', 'a', 't', 'a' };

class WaveFile
{
public:

    std::ofstream audioFile;
    //HANDLE waveFile;

    ULONG written = 0;
    WAVEFORMATEX format;

    WaveFile()
    {
        format.wFormatTag = WAVE_FORMAT_IEEE_FLOAT;
        format.nChannels = 1;
        format.nSamplesPerSec = 16000;
        format.wBitsPerSample = 32;
        format.nBlockAlign = format.nChannels * format.wBitsPerSample / 8;
        format.nAvgBytesPerSec = format.nSamplesPerSec * format.nBlockAlign;
        format.cbSize = 0;
    }

    ~WaveFile()
    {
        Close();
    }

    void Open( const std::string& fileName )
    {
        written = 0;

        audioFile.open( fileName, std::ios::out | std::ios::binary );
        audioFile << std::noskipws;
        WriteWaveHeader( 0 );
    }

    void Close()
    {
        if ( audioFile.is_open() ){
            audioFile.seekp( 0, std::ios::beg );
            WriteWaveHeader( written );
            audioFile.close();
        }
    }

    void Write( const void* data, int size )
    {
        audioFile.write( (char*)data, size );
        written += size;
    }

    /// <summary>
    /// Write the WAV file header contents. 
    /// </summary>
    /// <param name="waveFile">
    /// [in] Handle to file where header will be written.
    /// </param>
    /// <param name="pWaveFormat">
    /// [in] Format of file to write.
    /// </param>
    /// <param name="dataSize">
    /// Number of bytes of data in file's data section.
    /// </param>
    /// <returns>
    /// S_OK on success, otherwise failure code.
    /// </returns>
    void WriteWaveHeader( DWORD dataSize )
    {
        auto pWaveFormat = &format;
        DWORD waveHeaderSize = sizeof( WAVEHEADER ) + sizeof( WAVEFORMATEX ) + pWaveFormat->cbSize + sizeof( WaveData ) + sizeof( DWORD );
        WAVEHEADER waveHeader;

        // Update the sizes in the header
        memcpy_s( &waveHeader, sizeof( waveHeader ), WaveHeaderTemplate, sizeof( WaveHeaderTemplate ) );
        waveHeader.dwSize = waveHeaderSize + dataSize - (2 * sizeof( DWORD ));
        waveHeader.dwFmtSize = sizeof( WAVEFORMATEX ) + pWaveFormat->cbSize;

        // Write the file header
        audioFile.write( (char*)&waveHeader, sizeof( waveHeader ) );

        // Write the format
        audioFile.write( (char*)pWaveFormat, sizeof( WAVEFORMATEX ) + pWaveFormat->cbSize );

        // Write the data header
        audioFile.write( (char*)WaveData, sizeof( WaveData ) );

        audioFile.write( (char*)&dataSize, sizeof( dataSize ) );
    }
};
