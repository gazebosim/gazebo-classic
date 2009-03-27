#ifndef AUDIODECODER_HH
#define AUDIODECODER_HH

extern "C" {

#ifdef SPECIAL_LIBAVFORMAT
#include <libavformat/avformat.h>
#else
#include <avformat.h>
#endif

#ifdef SPECIAL_LIBAVCODEC
#include <libavcodec/avcodec.h>
#else
#include <avcodec.h>
#endif
}

#include <string>

class AudioDecoder
{
  /// \brief Constructor
  public: AudioDecoder();

  /// \brief Destructor
  public: virtual ~AudioDecoder();

  /// \brief Set the file to decode
  public: int SetFile(const std::string &filename);

  /// \brief Decode and audio file
  public: int Decode(uint8_t **outBuffer, unsigned int *outBufferSize);

  /// \brief Get the sample rate from the latest decoded file
  public: int GetSampleRate();

  private: AVFormatContext *formatCtx;
  private: AVCodecContext *codecCtx;

  // libavcodec audio codec
  private: AVCodec *codec;

  // Index of the audio stream
  private: int audioStream;

  private: static bool initialized;
};

#endif
