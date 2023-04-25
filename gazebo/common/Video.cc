/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Video.hh"
#include "gazebo/common/ffmpeg_inc.h"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
// #ifdef HAVE_FFMPEG
// static void pgm_save(unsigned char *buf, int wrap, int xsize, int ysize,
//                      char *filename)
// {
//   FILE *f;
//   int i;
//
//   f = fopen(filename, "w");
//   fprintf(f, "P6\n%d %d\n%d\n", xsize, ysize, 255);
//   for(i = 0; i < ysize; ++i)
//     fwrite(buf + i * wrap, 1, xsize * 3, f);
//   fclose(f);
// }
// #endif

/////////////////////////////////////////////////
#ifdef HAVE_FFMPEG
int GazeboAVCodecDecodeHelper(AVCodecContext *_codecCtx,
    AVFrame *_frame, int *_gotFrame, AVPacket *_packet)
{
#if LIBAVFORMAT_VERSION_MAJOR >= 59
  // from https://blogs.gentoo.org/lu_zero/2016/03/29/new-avcodec-api/
  int ret;

  *_gotFrame = 0;

  if (_packet)
  {
    ret = avcodec_send_packet(_codecCtx, _packet);
    if (ret < 0)
    {
      return ret == AVERROR_EOF ? 0 : ret;
    }
  }

  ret = avcodec_receive_frame(_codecCtx, _frame);
  if (ret < 0 && ret != AVERROR(EAGAIN))
  {
    return ret;
  }
  if (ret >= 0)
  {
    *_gotFrame = 1;
  }

  // new API always consumes the whole packet
  return _packet ? _packet->size : 0;
#else
  // this was deprecated in ffmpeg version 3.1
  // github.com/FFmpeg/FFmpeg/commit/7fc329e2dd6226dfecaa4a1d7adf353bf2773726
# ifndef _WIN32
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wdeprecated-declarations"
# endif
  return avcodec_decode_video2(_codecCtx, _frame, _gotFrame, _packet);
# ifndef _WIN32
#  pragma GCC diagnostic pop
# endif
#endif
}
#endif

/////////////////////////////////////////////////
Video::Video()
{
  this->formatCtx = nullptr;
  this->codecCtx = nullptr;
  this->swsCtx = nullptr;
  this->avFrame = nullptr;
  this->videoStream = -1;
  this->avFrameDst = nullptr;
}

/////////////////////////////////////////////////
Video::~Video()
{
  this->Cleanup();
}

/////////////////////////////////////////////////
void Video::Cleanup()
{
#ifdef HAVE_FFMPEG
  // Free the YUV frame
  av_free(this->avFrame);

  // Close the video file
  avformat_close_input(&this->formatCtx);

  // Close the codec
  avcodec_close(this->codecCtx);

  av_free(this->avFrameDst);
#endif
}

/////////////////////////////////////////////////
#ifdef HAVE_FFMPEG
bool Video::Load(const std::string &_filename)
{
  const AVCodec *codec = nullptr;
  this->videoStream = -1;

  if (this->formatCtx || this->avFrame || this->codecCtx)
    this->Cleanup();

  this->avFrame = common::AVFrameAlloc();

  // Open video file
  if (avformat_open_input(&this->formatCtx, _filename.c_str(),
        nullptr, nullptr) < 0)
  {
    gzerr << "Unable to read video file[" << _filename << "]\n";
    return false;
  }

  // Retrieve stream information
  if (avformat_find_stream_info(this->formatCtx, nullptr) < 0)
  {
    gzerr << "Couldn't find stream information\n";
    return false;
  }

  // Find the first video stream
  for (unsigned int i = 0; i < this->formatCtx->nb_streams; ++i)
  {
#if LIBAVFORMAT_VERSION_MAJOR >= 59
    if (this->formatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
#else
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    if (this->formatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO)
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#endif
    {
      this->videoStream = static_cast<int>(i);
      break;
    }
  }

  if (this->videoStream == -1)
  {
    gzerr << "Unable to find a video stream\n";
    return false;
  }

  // Get a pointer to the codec context for the video stream
#if LIBAVFORMAT_VERSION_MAJOR >= 59
  // AVCodecContext is not included in an AVStream as of ffmpeg 3.1
  // allocate a codec context based on updated example
  // github.com/FFmpeg/FFmpeg/commit/bba6a03b2816d805d44bce4f9701a71f7d3f8dad
  this->codecCtx = avcodec_alloc_context3(codec);
  if (!this->codecCtx)
  {
    gzerr << "Failed to allocate the codec context" << std::endl;
    return false;
  }

  // Copy codec parameters from input stream to output codec context
  if (avcodec_parameters_to_context(this->codecCtx,
        this->formatCtx->streams[this->videoStream]->codecpar) < 0)
  {
    gzerr << "Failed to copy codec parameters to decoder context"
           << std::endl;
    return false;
  }
#else
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  this->codecCtx = this->formatCtx->streams[this->videoStream]->codec;
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#endif


  // Find the decoder for the video stream
  codec = avcodec_find_decoder(this->codecCtx->codec_id);
  if (codec == nullptr)
  {
    gzerr << "Codec not found\n";
    return false;
  }

  // Inform the codec that we can handle truncated bitstreams -- i.e.,
  // bitstreams where frame boundaries can fall in the middle of packets
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(56, 60, 100)
#if LIBAVCODEC_VERSION_MAJOR < 60
  if (codec->capabilities & AV_CODEC_CAP_TRUNCATED)
    this->codecCtx->flags |= AV_CODEC_FLAG_TRUNCATED;
#endif
#else
  if (codec->capabilities & CODEC_CAP_TRUNCATED)
    this->codecCtx->flags |= CODEC_FLAG_TRUNCATED;
#endif

  // Open codec
  if (avcodec_open2(this->codecCtx, codec, nullptr) < 0)
  {
    gzerr << "Could not open codec\n";
    return false;
  }

  this->swsCtx = sws_getContext(
      this->codecCtx->width,
      this->codecCtx->height,
      this->codecCtx->pix_fmt,
      this->codecCtx->width,
      this->codecCtx->height,
      AV_PIX_FMT_RGB24,
      SWS_BICUBIC, nullptr, nullptr, nullptr);

  if (this->swsCtx == nullptr)
  {
    gzerr << "Error while calling sws_getContext\n";
    return false;
  }

  this->avFrameDst = common::AVFrameAlloc();
  this->avFrameDst->format = this->codecCtx->pix_fmt;
  this->avFrameDst->width = this->codecCtx->width;
  this->avFrameDst->height = this->codecCtx->height;
  av_image_alloc(this->avFrameDst->data, this->avFrameDst->linesize,
      this->codecCtx->width, this->codecCtx->height, this->codecCtx->pix_fmt,
      1);

  // DEBUG: Will save all the frames
  /*Image img;
  char buf[1024];
  int frame = 0;

  // the decoding loop, running until EOF
  while (this->GetNextFrame(img))
  {
    printf("WH[%d %d]\n",this->codecCtx->width, this->codecCtx->height);
    snprintf(buf, sizeof(buf), "/tmp/test_%3d.png", frame++);
    img.SavePNG(buf);
  }
  printf("Done\n");
  */

  return true;
}
#else
bool Video::Load(const std::string &/*_filename*/)
{
  return false;
}
#endif

/////////////////////////////////////////////////
#ifdef HAVE_FFMPEG
bool Video::GetNextFrame(unsigned char **_buffer)
{
  AVPacket packet, tmpPacket;
  int frameAvailable = 0;

  av_init_packet(&packet);

  // Read a frame.
  if (av_read_frame(this->formatCtx, &packet) < 0)
    return false;

  if (packet.stream_index == this->videoStream)
  {
    tmpPacket.data = packet.data;
    tmpPacket.size = packet.size;

    // Process all the data in the frame
    while (tmpPacket.size > 0)
    {
      // sending data to libavcodec
      int processedLength = GazeboAVCodecDecodeHelper(this->codecCtx, this->avFrame,
          &frameAvailable, &tmpPacket);

      if (processedLength < 0)
      {
        gzerr << "Error while processing the data\n";
        break;
      }

      tmpPacket.data = tmpPacket.data + processedLength;
      tmpPacket.size = tmpPacket.size - processedLength;

      // processing the image if available
      if (frameAvailable)
      {
        sws_scale(swsCtx, this->avFrame->data, this->avFrame->linesize, 0,
            this->codecCtx->height, this->avFrameDst->data,
            this->avFrameDst->linesize);

        memcpy(*_buffer, this->avFrameDst->data[0],
            this->codecCtx->height * (this->codecCtx->width*3));

        // Debug:
        // pgm_save(this->pic.data[0], this->pic.linesize[0],
        //          this->codecCtx->width, this->codecCtx->height, buf);
      }
    }
  }
  AVPacketUnref(&packet);

  return true;
}
#else
bool Video::GetNextFrame(unsigned char ** /*_buffer*/)
{
  return false;
}
#endif

/////////////////////////////////////////////////
int Video::GetWidth() const
{
#ifdef HAVE_FFMPEG
  return this->codecCtx->width;
#else
  return 0;
#endif
}

/////////////////////////////////////////////////
int Video::GetHeight() const
{
#ifdef HAVE_FFMPEG
  return this->codecCtx->height;
#else
  return 0;
#endif
}
