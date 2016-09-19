/*
* Copyright (C) 2012-2016 Open Source Robotics Foundation
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
*/

#include <gazebo/gazebo_config.h>
#include <gazebo/common/AudioDecoder.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/ffmpeg_inc.h>

#define AUDIO_INBUF_SIZE (20480 * 2)
#define AUDIO_REFILL_THRESH 4096

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
AudioDecoder::AudioDecoder()
{
  this->formatCtx = nullptr;
  this->codecCtx = nullptr;
  this->codec = nullptr;
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
  // Close the codec
  if (this->codecCtx)
    avcodec_close(this->codecCtx);

  // Close the audio file
  if (this->formatCtx)
    avformat_close_input(&this->formatCtx);
#endif
}

/////////////////////////////////////////////////
#ifdef HAVE_FFMPEG
bool AudioDecoder::Decode(uint8_t **_outBuffer, unsigned int *_outBufferSize)
{
  AVPacket packet, packet1;
  int bytesDecoded = 0;
  unsigned int maxBufferSize = 0;
  AVFrame *decodedFrame = nullptr;

  if (this->codec == nullptr)
  {
    gzerr << "Set an audio file before decoding.\n";
    return false;
  }

  if (_outBufferSize == nullptr)
  {
    gzerr << "outBufferSize is null!!\n";
    return false;
  }

  *_outBufferSize = 0;

  if (*_outBuffer)
  {
    delete [] *_outBuffer;
    *_outBuffer = nullptr;
  }

  bool result = true;

  if (!decodedFrame)
  {
    if (!(decodedFrame = common::AVFrameAlloc()))
    {
      gzerr << "Audio decoder out of memory\n";
      result = false;
    }
  }
  else
    common::AVFrameUnref(decodedFrame);

  av_init_packet(&packet);
  while (av_read_frame(this->formatCtx, &packet) == 0)
  {
    if (packet.stream_index == this->audioStream)
    {
      int gotFrame = 0;

      packet1 = packet;
      while (packet1.size)
      {
        // Some frames rely on multiple packets, so we have to make sure
        // the frame is finished before we can use it
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
        bytesDecoded = avcodec_decode_audio4(this->codecCtx, decodedFrame,
            &gotFrame, &packet1);
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

        if (gotFrame)
        {
          // Total size of the data. Some padding can be added to
          // decodedFrame->data[0], which is why we can't use
          // decodedFrame->linesize[0].
          int size = decodedFrame->nb_samples *
            av_get_bytes_per_sample(this->codecCtx->sample_fmt) *
            this->codecCtx->channels;

          // Resize the audio buffer as necessary
          if (*_outBufferSize + size > maxBufferSize)
          {
            maxBufferSize += size * 5;
            *_outBuffer = reinterpret_cast<uint8_t*>(realloc(*_outBuffer,
                  maxBufferSize * sizeof(*_outBuffer[0])));
          }

          memcpy(*_outBuffer + *_outBufferSize, decodedFrame->data[0],
              size);
          *_outBufferSize += size;
        }

        packet1.data += bytesDecoded;
        packet1.size -= bytesDecoded;
      }
    }
    AVPacketUnref(&packet);
  }

  AVPacketUnref(&packet);

  // Seek to the beginning so that it can be decoded again, if necessary.
  av_seek_frame(this->formatCtx, this->audioStream, 0, 0);

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
  if (avformat_open_input(&this->formatCtx,
        _filename.c_str(), nullptr, nullptr) < 0)
  {
    gzerr << "Unable to open audio file[" << _filename << "]\n";
    this->formatCtx = nullptr;
    return false;
  }

  // Hide av logging
  av_log_set_level(0);

  // Retrieve some information
  if (avformat_find_stream_info(this->formatCtx, nullptr) < 0)
  {
    gzerr << "Unable to find stream info.\n";
    avformat_close_input(&this->formatCtx);
    this->formatCtx = nullptr;

    return false;
  }

  // Dump information about file onto standard error.
  // dump_format(this->formatCtx, 0, "dump.txt", false);

  // Find audio stream;
  this->audioStream = -1;
  for (i = 0; i < this->formatCtx->nb_streams; ++i)
  {
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    if (this->formatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_AUDIO)
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
    {
      this->audioStream = i;
      break;
    }
  }

  if (this->audioStream == -1)
  {
    gzerr << "Couldn't find audio stream.\n";
    avformat_close_input(&this->formatCtx);
    this->formatCtx = nullptr;

    return false;
  }

  // Get the audio stream codec
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  this->codecCtx = this->formatCtx->streams[audioStream]->codec;
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

  // Find a decoder
  this->codec = avcodec_find_decoder(codecCtx->codec_id);

  if (this->codec == nullptr)
  {
    gzerr << "Couldn't find codec for audio stream.\n";
    avformat_close_input(&this->formatCtx);
    this->formatCtx = nullptr;

    return false;
  }

  if (this->codec->capabilities & CODEC_CAP_TRUNCATED)
    this->codecCtx->flags |= CODEC_FLAG_TRUNCATED;

  // Open codec
  if (avcodec_open2(this->codecCtx, this->codec, nullptr) < 0)
  {
    gzerr << "Couldn't open audio codec.\n";
    avformat_close_input(&this->formatCtx);
    this->formatCtx = nullptr;

    return false;
  }

  this->filename = _filename;

  return true;
}
#else
bool AudioDecoder::SetFile(const std::string & /*_filename*/)
{
  return false;
}
#endif

/////////////////////////////////////////////////
std::string AudioDecoder::GetFile() const
{
  return this->filename;
}
