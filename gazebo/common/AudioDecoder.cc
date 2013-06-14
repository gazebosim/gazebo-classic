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
}
#endif

using namespace gazebo;
using namespace common;

bool AudioDecoder::initialized = false;

/////////////////////////////////////////////////
AudioDecoder::AudioDecoder()
{
  this->formatCtx = NULL;
  this->codecCtx = NULL;
  this->codec = NULL;
  this->audioStream = 0;

  // Initialize the ffmpeg library only once
  if (!this->initialized)
  {
    this->initialized = true;
    avcodec_init();
    avcodec_register_all();

    // Register all formats and codecs
    av_register_all();
  }
}

/////////////////////////////////////////////////
AudioDecoder::~AudioDecoder()
{
}

/////////////////////////////////////////////////
bool AudioDecoder::Decode(uint8_t **_outBuffer, unsigned int *_outBufferSize)
{
  AVPacket packet;
  char tmpBuf[AVCODEC_MAX_AUDIO_FRAME_SIZE];
  int tmpBufsize = 0;
  int bytesDecoded = 0;
  unsigned int maxBufferSize = 0;

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

  // Read the next frame of a stream
  while (av_read_frame(this->formatCtx, &packet) >=0)
  {
    if (packet.stream_index == this->audioStream)
    {
      tmpBufsize = sizeof(tmpBuf);

      // Decode the frame
      bytesDecoded = avcodec_decode_audio2( this->codecCtx, (int16_t*)tmpBuf,
          &tmpBufsize, packet.data, packet.size );

      if (bytesDecoded < 0)
      {
        gzerr << "Error decoding audio\n";
        return false;
      }

      if (tmpBufsize <= 0)
      {
        gzerr << "No data yet\n";
        return false;
      }

      // Resize the audio buffer as necessary
      if (*_outBufferSize + tmpBufsize > maxBufferSize)
      {
        maxBufferSize += tmpBufsize * 10;
        *_outBuffer = (uint8_t*)realloc(*_outBuffer,
            maxBufferSize * sizeof(*_outBuffer[0]) );
      }

      memcpy(*_outBuffer + *_outBufferSize, tmpBuf, tmpBufsize);
      *_outBufferSize += tmpBufsize;
    }
  }

  av_free_packet(&packet);

  return true;
}

/////////////////////////////////////////////////
int AudioDecoder::GetSampleRate()
{
  return this->codecCtx->sample_rate;
}

/////////////////////////////////////////////////
bool AudioDecoder::SetFile(const std::string &_filename)
{
  unsigned int i;

  // Open file
  if (av_open_input_file(&this->formatCtx,
        _filename.c_str(), NULL, 0, NULL) != 0)
  {
    gzerr << "Unable to open audio file[" << _filename << "]\n";
    return false;
  }

  // Retrieve some information
  if (av_find_stream_info(this->formatCtx) < 0)
  {
    gzerr << "Unable to find stream info\n";
    return false;
  }

  // Dump information about file onto standard error
  // dump_format(formatCtx, 0, "dump.txt", false);

  // Find audio stream;
  this->audioStream = -1;
  for (i = 0; i < this->formatCtx->nb_streams; ++i)
  {
    if (this->formatCtx->streams[i]->codec->codec_type == CODEC_TYPE_AUDIO)
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
  if (avcodec_open(this->codecCtx, this->codec) < 0)
    perror("couldn't open codec");

  return true;
}
