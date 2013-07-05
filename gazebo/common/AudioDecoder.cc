/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <gazebo/gazebo_config.h>
#include <gazebo/common/Console.hh>
#include <gazebo/common/AudioDecoder.hh>

#ifdef HAVE_FFMPEG
#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
}
#endif

#define AUDIO_INBUF_SIZE (20480 * 2)
#define AUDIO_REFILL_THRESH 4096

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
AudioDecoder::AudioDecoder()
{
  this->formatCtx = NULL;
  this->codecCtx = NULL;
  this->codec = NULL;
  this->audioStream = 0;
}

/////////////////////////////////////////////////
AudioDecoder::~AudioDecoder()
{
  this->Cleanup();
}

/////////////////////////////////////////////////
void AudioDecoder::Cleanup()
{
#ifdef HAVE_FFMPEG
  // Close the audio file
  avformat_close_input(&this->formatCtx);

  // Close the codec
  avcodec_close(this->codecCtx);
#endif
}

/////////////////////////////////////////////////
#ifdef HAVE_FFMPEG
bool AudioDecoder::Decode(uint8_t **_outBuffer, unsigned int *_outBufferSize)
{
  AVPacket packet;
  uint8_t tmpBuf[AUDIO_INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];
  int bytesDecoded = 0;
  unsigned int maxBufferSize = 0;
  AVFrame *decodedFrame = NULL;

  if (this->codec == NULL)
  {
    gzerr << "Set an audio file before decoding.\n";
    return false;
  }

  if (_outBufferSize == NULL)
  {
    gzerr << "outBufferSize is NULL!!\n";
    return false;
  }

  *_outBufferSize = 0;

  if (*_outBuffer)
  {
    delete [] *_outBuffer;
    *_outBuffer = NULL;
  }

  FILE *f = fopen("/home/nkoenig/Music/track2.wav", "rb");

  if (!f)
  {
    gzerr << "Unable to open file\n";
    return false;
  }

  av_init_packet(&packet);
  packet.data = tmpBuf;
  packet.size = fread(tmpBuf, 1, AUDIO_INBUF_SIZE, f);

  bool result = true;
  while (packet.size > 0)
  {
    int gotFrame = 0;

    if (!decodedFrame)
    {
      if (!(decodedFrame = avcodec_alloc_frame()))
      {
        gzerr << "openAL out of memory\n";
        result = false;
        break;
      }
    }
    else
      avcodec_get_frame_defaults(decodedFrame);

    bytesDecoded = avcodec_decode_audio4(this->codecCtx, decodedFrame,
        &gotFrame, &packet);

    if (bytesDecoded < 0)
      break;

    if (gotFrame > 0)
    {
      // Resize the audio buffer as necessary
      if (*_outBufferSize + bytesDecoded > maxBufferSize)
      {
        maxBufferSize += bytesDecoded * 5;
        *_outBuffer = reinterpret_cast<uint8_t*>(realloc(*_outBuffer,
            maxBufferSize * sizeof(*_outBuffer[0])));
      }

      memcpy(*_outBuffer + *_outBufferSize, tmpBuf, bytesDecoded);
      *_outBufferSize += bytesDecoded;
    }

    // subtract data from whatever decode function returns
    packet.size -= bytesDecoded;
    packet.data += bytesDecoded;

    if (packet.size < AUDIO_REFILL_THRESH)
    {
      // Refill the input buffer, to avoid trying to decode
      // incomplete frames. Instead of this, one could also use
      // a parser, or use a proper container format through
      // libavformat.

      memmove(tmpBuf, packet.data, packet.size);
      packet.data = tmpBuf;
      int len = fread(packet.data + packet.size, 1,
          AUDIO_INBUF_SIZE - packet.size, f);

      if (len > 0)
        packet.size += len;
    }
  }

  fclose(f);
  av_free_packet(&packet);

  return result;
}
#else
bool AudioDecoder::Decode(uint8_t ** /*_outBuffer*/,
    unsigned int * /*_outBufferSize*/)
{
  return true;
}
#endif

/////////////////////////////////////////////////
int AudioDecoder::GetSampleRate()
{
#ifdef HAVE_FFMPEG
  return this->codecCtx->sample_rate;
#else
  return 0;
#endif
}

/////////////////////////////////////////////////
#ifdef HAVE_FFMPEG
bool AudioDecoder::SetFile(const std::string &_filename)
{
  unsigned int i;

  this->formatCtx = avformat_alloc_context();

  // Open file
  if (avformat_open_input(&this->formatCtx, _filename.c_str(), NULL, NULL) < 0)
  {
    gzerr << "Unable to open audio file[" << _filename << "]\n";
    return false;
  }

  // Hide av logging
  av_log_set_level(0);

  // Retrieve some information
  if (avformat_find_stream_info(this->formatCtx, NULL) < 0)
  {
    gzerr << "Unable to find stream info\n";
    return false;
  }

  // Dump information about file onto standard error.
  // dump_format(this->formatCtx, 0, "dump.txt", false);

  // Find audio stream;
  this->audioStream = -1;
  for (i = 0; i < this->formatCtx->nb_streams; ++i)
  {
    if (this->formatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_AUDIO)
    {
      this->audioStream = i;
      break;
    }
  }

  if (this->audioStream == -1)
  {
    gzerr << "Couldn't find audio stream\n";
    return false;
  }

  // Get the audio stream codec
  this->codecCtx = this->formatCtx->streams[audioStream]->codec;

  // Find a decoder
  this->codec = avcodec_find_decoder(codecCtx->codec_id);

  if (this->codec == NULL)
  {
    perror("couldn't find codec");
    return false;
  }

  if (this->codec->capabilities & CODEC_CAP_TRUNCATED)
    this->codecCtx->flags |= CODEC_FLAG_TRUNCATED;

  // Open codec
  if (avcodec_open2(this->codecCtx, this->codec, NULL) < 0)
    perror("couldn't open codec");

  return true;
}
#else
bool AudioDecoder::SetFile(const std::string & /*_filename*/)
{
  return false;
}
#endif
