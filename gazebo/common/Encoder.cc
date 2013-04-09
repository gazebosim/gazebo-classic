/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include <boost/filesystem.hpp>

#include <gazebo/gazebo_config.h>

#ifdef HAVE_FFMPEG
#ifndef INT64_C
#define INT64_C(codec) (codec ## LL)
#define UINT64_C(codec) (codec ## ULL)
#endif

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/mathematics.h>
}
#endif

#include "gazebo/math/Helpers.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Encoder.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
Encoder::Encoder()
{
  this->initialized = false;
#ifdef HAVE_FFMPEG
  static bool first = true;
  if (first)
  {
    first = false;
    av_register_all();
  }

  this->Reset();
#endif
}

/////////////////////////////////////////////////
Encoder::~Encoder()
{
  this->Cleanup();
}

/////////////////////////////////////////////////
void Encoder::Cleanup()
{
  if (!this->initialized)
    return;


#ifdef HAVE_FFMPEG
  if (this->formatCtx)
  {
    for (unsigned int i = 0; i < this->formatCtx->nb_streams; ++i)
    {
      avcodec_close(this->formatCtx->streams[i]->codec);
      av_freep(&this->formatCtx->streams[i]->codec);
      av_freep(&this->formatCtx->streams[i]);
    }
    av_free(this->formatCtx);
    this->formatCtx = NULL;
  }

  if (this->avOutFrame)
  {
    av_free(this->avOutFrame);
    this->avOutFrame = NULL;
  }

  if (this->avInPicture)
  {
    av_free(this->avInPicture);
    this->avInPicture = NULL;
  }

  if (this->swsCtx)
    sws_freeContext(this->swsCtx);
#endif

  if (this->outbuf)
  {
    delete[] this->outbuf;
    this->outbuf = NULL;
  }
  if (this->pictureBuf)
  {
    delete[] this->pictureBuf;
    this->pictureBuf = NULL;
  }

  this->initialized = false;
}

/////////////////////////////////////////////////
void Encoder::SetBitRate(unsigned int _bitRate)
{
  this->bitRate = _bitRate;
}

/////////////////////////////////////////////////
void Encoder::SetFrameWidth(unsigned int _width)
{
  this->frameWidth = _width;
}

/////////////////////////////////////////////////
void Encoder::SetFrameHeight(unsigned int _height)
{
  this->frameHeight = _height;
}

/////////////////////////////////////////////////
void Encoder::SetFormat(const std::string &_format)
{
  this->format = _format;
}

/////////////////////////////////////////////////
std::string Encoder::GetFormat() const
{
  return this->format;
}

/////////////////////////////////////////////////
#ifdef HAVE_FFMPEG
void Encoder::Init()
{
  if (this->initialized)
    return;

  std::string tmpFileNameFull = this->tmpFilename + "." + this->format;
  this->outputFormat = av_guess_format(NULL, tmpFileNameFull.c_str(), NULL);
  if (!this->outputFormat)
  {
    gzerr << "Could not deduce output format from file extension: "
        << "using MPEG.\n";
    this->outputFormat = av_guess_format("mpeg", NULL, NULL);
  }

  AVCodec *codec;

  // find the video encoder
  codec = avcodec_find_encoder(this->outputFormat->video_codec);
  if (!codec)
  {
    gzerr << "Codec not found\n";
    return;
  }

  this->formatCtx = avformat_alloc_context();
  this->formatCtx->oformat = this->outputFormat;
  snprintf(this->formatCtx->filename,
      sizeof(this->formatCtx->filename),
      "%s", tmpFileNameFull.c_str());
  this->videoStream = avformat_new_stream(this->formatCtx, codec);
  this->videoStream->id = this->formatCtx->nb_streams-1;

  this->codecCtx = this->videoStream->codec;

  // some formats want stream headers to be separate
  if (this->formatCtx->oformat->flags & AVFMT_GLOBALHEADER)
    this->codecCtx->flags |= CODEC_FLAG_GLOBAL_HEADER;

  // put sample parameters
  this->codecCtx->bit_rate = this->bitRate;
  this->codecCtx->rc_max_rate = this->bitRate;
  this->codecCtx->rc_min_rate = this->bitRate;
  // resolution must be a multiple of two
  this->codecCtx->width = this->frameWidth;
  this->codecCtx->height = this->frameHeight;
  // frames per second
  this->codecCtx->time_base.den= this->fps;
  this->codecCtx->time_base.num= 1;
  // emit one intra frame every ten frames
  this->codecCtx->gop_size = 10;
  this->codecCtx->pix_fmt = PIX_FMT_YUV420P;

  // this removes VBV buffer size not set warning msg
  this->codecCtx->rc_initial_buffer_occupancy = this->bitRate;
  this->codecCtx->rc_max_rate = this->bitRate;
  this->codecCtx->rc_buffer_size = this->bitRate;
  this->codecCtx->thread_count = 5;
  if (this->codecCtx->codec_id == CODEC_ID_MPEG1VIDEO)
  {
    // Needed to avoid using macroblocks in which some coeffs overflow.
    // This does not happen with normal video, it just happens here as
    // the motion of the chroma plane does not match the luma plane.
    this->codecCtx->mb_decision = 2;
  }

  // open it
  if (avcodec_open2(this->codecCtx, codec, NULL) < 0)
  {
    gzerr << "Could not open codec\n";
    return;
  }

  this->fileHandle = fopen(tmpFileNameFull.c_str(), "wb");
  if (!this->fileHandle)
  {
    gzerr << "Could not open '" << tmpFileNameFull << "' for encoding\n";
    return;
  }

  this->avOutFrame = avcodec_alloc_frame();
  int size = avpicture_get_size(this->codecCtx->pix_fmt, this->codecCtx->width,
      this->codecCtx->height);
  this->pictureBuf = new unsigned char[size];
  avpicture_fill(reinterpret_cast<AVPicture *>(this->avOutFrame),
      this->pictureBuf, this->codecCtx->pix_fmt, this->codecCtx->width,
      this->codecCtx->height);

  av_dump_format(this->formatCtx, 0, tmpFileNameFull.c_str(), 1);

  // setting mux preload and max delay avoids buffer underflow when writing to
  // mpeg format
  double muxMaxDelay = 0.7f;
  this->formatCtx->max_delay = static_cast<int>(muxMaxDelay * AV_TIME_BASE);

  if (!(this->outputFormat->flags & AVFMT_NOFILE))
  {
    if (avio_open(&this->formatCtx->pb, tmpFileNameFull.c_str(),
        AVIO_FLAG_WRITE) < 0)
    {
      gzerr << "Could not open '" << tmpFileNameFull << "'\n";
      return;
    }
  }
  // Write the stream header, if any.
  if (avformat_write_header(this->formatCtx, NULL) < 0)
    gzerr << " Error occured when opening output file" << std::endl;

  // alloc image and output buffer
  this->outBufferSize = this->codecCtx->width * this->codecCtx->height;
  this->outbuf = new unsigned char[this->outBufferSize];

  this->initialized = true;
}
#else
void Encoder::Init()
{
  gzwarn << "Encoding capability not available! "
      << "Please install libavcodec, libavformat and libswscale dev packages."
      << std::endl;
}
#endif

////////////////////////////////////////////////
bool Encoder::IsInitialized()
{
  return this->initialized;
}

/////////////////////////////////////////////////
void Encoder::AddFrame(unsigned char *_frame, unsigned int _width,
    unsigned int _height)
{
  this->AddFrame(_frame, _width, _height,
      common::Time::GetWallTime());
}

/////////////////////////////////////////////////
#ifdef HAVE_FFMPEG
void Encoder::AddFrame(unsigned char *_frame, unsigned int _width,
    unsigned int _height, common::Time _timestamp)
{
  if (!this->initialized)
    this->Init();

  double dt = (_timestamp - this->timePrev).Double();
  if ( dt < 1.0/this->sampleRate)
    return;

  this->timePrev = _timestamp;

  int pts = 0;
  if (this->videoPts != -1)
  {
    pts = this->fps * this->totalTime;
    this->totalTime += dt;

    if (this->videoPts == pts)
      return;
  }

  this->videoPts = pts;

  // recreate the sws context on image resize
  if (this->swsCtx)
  {
    if (this->inFrameWidth != _width || this->inFrameHeight != _height)
    {
      sws_freeContext(this->swsCtx);
      this->swsCtx = NULL;
      if (this->avInPicture)
      {
        av_free(this->avInPicture);
        this->avInPicture = NULL;
      }
    }
  }
  if (!this->swsCtx)
  {
    if (!this->avInPicture)
    {
      this->avInPicture = new AVPicture;
      avpicture_alloc(this->avInPicture, PIX_FMT_RGB24, _width, _height);
    }
    this->inFrameWidth = _width;
    this->inFrameHeight = _height;
    this->swsCtx = sws_getContext(_width, _height, PIX_FMT_RGB24,
        this->codecCtx->width, this->codecCtx->height, this->codecCtx->pix_fmt,
        SWS_BICUBIC, NULL, NULL, NULL);
    if (this->swsCtx == NULL)
    {
      gzerr << "Error while calling sws_getContext\n";
      return;
    }
  }

  // encode
  memcpy(this->avInPicture->data[0], _frame, _width * _height * 3);
  sws_scale(this->swsCtx, this->avInPicture->data, this->avInPicture->linesize,
      0, _height, this->avOutFrame->data, this->avOutFrame->linesize);
  this->codecCtx->coded_frame->pts = this->videoPts;

  this->outSize = avcodec_encode_video(this->codecCtx, this->outbuf,
      this->outBufferSize, this->avOutFrame);

  this->codecCtx->coded_frame->pts = this->videoPts;

  // write the frame
  if (outSize > 0)
  {
    AVPacket avPacket;
    av_init_packet(&avPacket);
    avPacket.pts= av_rescale_q(this->codecCtx->coded_frame->pts,
        this->codecCtx->time_base, this->videoStream->time_base);

    if (this->codecCtx->coded_frame->key_frame)
       avPacket.flags |= AV_PKT_FLAG_KEY;

    avPacket.stream_index= this->videoStream->index;
    avPacket.data= this->outbuf;
    avPacket.size= this->outSize;
    int ret = av_interleaved_write_frame(this->formatCtx, &avPacket);

    if (ret < 0)
      gzerr << "Error writing frame" << std::endl;
    av_free_packet(&avPacket);
  }
}
#else
void Encoder::AddFrame(unsigned char */*_frame*/, unsigned int /*_w*/,
    unsigned int /*_h*/)
{
}
#endif

/////////////////////////////////////////////////
void Encoder::Fini()
{
#ifdef HAVE_FFMPEG
  if (this->formatCtx)
    av_write_trailer(this->formatCtx);
#endif
}

/////////////////////////////////////////////////
void Encoder::SaveToFile(const std::string &_filename)
{
#ifdef HAVE_FFMPEG
  this->Fini();

  boost::filesystem::rename(this->tmpFilename + "." + this->format,
      _filename + "." + this->format);

  this->Cleanup();
#else
  gzwarn << _filename << " not saved" << std::endl;
#endif
}

/////////////////////////////////////////////////
void Encoder::Reset()
{
  this->Cleanup();
  // set default values
  this->bitRate = 2000000;
  this->frameWidth = 1024;
  this->frameHeight = 768;
  this->fps = 25;
  this->tmpFilename = "tmp_recording";
  this->format = "ogv";
  this->timePrev = 0;
  this->sampleRate = this->fps * 2;
  this->totalTime = 0;
  this->videoPts = -1;
  this->outbuf = NULL;
  this->pictureBuf = NULL;
  this->inFrameWidth = 0;
  this->inFrameHeight = 0;
  this->initialized = false;

#ifdef HAVE_FFMPEG
  this->codecCtx = NULL;
  this->swsCtx = NULL;
  this->avInPicture = NULL;
  this->avOutFrame = NULL;
  this->formatCtx = NULL;
#endif
}
