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
#include "libavutil/mathematics.h"

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
  this->outbuf = NULL;
  this->pictureBuf = NULL;
  this->codecCtx = NULL;
  this->swsCtx = NULL;
  this->pic = NULL;
  this->avFrame = NULL;
  this->pFormatCtx = NULL;

#ifdef HAVE_FFMPEG
  static bool first = true;
  if (first)
  {
    first = false;
    av_register_all();
  }

  this->Reset();
//  this->pic = new AVthis->avFrame;
#endif
}

/////////////////////////////////////////////////
Encoder::~Encoder()
{
  this->Cleanup();
#ifdef HAVE_FFMPEG
  delete this->pic;
#endif
}

/////////////////////////////////////////////////
void Encoder::Cleanup()
{
  if (!this->initialized)
    return;


#ifdef HAVE_FFMPEG
  if (this->pFormatCtx)
  {
    for(unsigned int i = 0; i < this->pFormatCtx->nb_streams; ++i)
    {
      avcodec_close(this->pFormatCtx->streams[i]->codec);
      av_freep(&this->pFormatCtx->streams[i]->codec);
      av_freep(&this->pFormatCtx->streams[i]);
    }
    av_free(this->pFormatCtx);
    this->pFormatCtx = NULL;
  }

  if (this->avFrame)
  {
    av_free(this->avFrame);
    this->avFrame = NULL;
  }

  if (this->pic)
  {
    av_free(this->pic);
    this->pic = NULL;
  }
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
  this->pOutputFormat = av_guess_format(NULL, tmpFileNameFull.c_str(), NULL);
  if (!this->pOutputFormat)
  {
    gzerr << "Could not deduce output format from file extension: "
        << "using MPEG.\n";
    this->pOutputFormat = av_guess_format("mpeg", NULL, NULL);
  }

  AVCodec *codec;

  // find the video encoder
  codec = avcodec_find_encoder(this->pOutputFormat->video_codec);
  if (!codec)
  {
    gzerr << "Codec not found\n";
    return;
  }

  this->pFormatCtx = avformat_alloc_context();
  this->pFormatCtx->oformat = this->pOutputFormat;
  snprintf(this->pFormatCtx->filename,
      sizeof(this->pFormatCtx->filename),
      "%s", tmpFileNameFull.c_str());
  this->pVideoStream = avformat_new_stream(this->pFormatCtx, codec);

  this->codecCtx = this->pVideoStream->codec;
//  this->codecCtx->codec_id = this->pOutputFormat->video_codec;
//  this->codecCtx->codec_type = AVMEDIA_TYPE_VIDEO;

  // some formats want stream headers to be separate
  if(this->pFormatCtx->oformat->flags & AVFMT_GLOBALHEADER)
    this->codecCtx->flags |= CODEC_FLAG_GLOBAL_HEADER;

  // put sample parameters
  this->codecCtx->bit_rate = this->bitRate;
  // resolution must be a multiple of two
  this->codecCtx->width = this->frameWidth;
  this->codecCtx->height = this->frameHeight;
  // frames per second
  this->codecCtx->time_base.den= this->fps;
  this->codecCtx->time_base.num= 1;
  this->codecCtx->gop_size = 10; // emit one intra frame every ten frames
  this->codecCtx->max_b_frames = 1;
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
  this->pic = new AVPicture;

  this->fileHandle = fopen(tmpFileNameFull.c_str(), "wb");
  if (!this->fileHandle)
  {
    gzerr << "Could not open '" << tmpFileNameFull << "' for encoding\n";
    return;
  }

  this->avFrame = avcodec_alloc_frame();
  int size = avpicture_get_size(this->codecCtx->pix_fmt, this->codecCtx->width,
      this->codecCtx->height);
  this->pictureBuf = new unsigned char[size];
  avpicture_fill((AVPicture *)this->avFrame, this->pictureBuf,
      this->codecCtx->pix_fmt, this->codecCtx->width, this->codecCtx->height);

  av_dump_format(this->pFormatCtx, 0, tmpFileNameFull.c_str(), 1);

  // setting mux preload and max delay avoids buffer underflow when writing to
  // mpeg format
  double muxPreload  = 0.5f;
  double muxMaxDelay = 0.7f;
  this->pFormatCtx->preload = static_cast<int>(muxPreload * AV_TIME_BASE);
  this->pFormatCtx->max_delay = static_cast<int>(muxMaxDelay * AV_TIME_BASE);

  if (!(this->pOutputFormat->flags & AVFMT_NOFILE))
  {
    if (avio_open(&this->pFormatCtx->pb, tmpFileNameFull.c_str(),
        AVIO_FLAG_WRITE) < 0)
    {
      gzerr << "Could not open '" << tmpFileNameFull << "'\n";
      return;
    }
  }
  // Write the stream header, if any.
  avformat_write_header(this->pFormatCtx, NULL);

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
#ifdef HAVE_FFMPEG
void Encoder::AddFrame(unsigned char *_frame, unsigned int _w,
    unsigned int _h)
{
  if (!this->initialized)
    this->Init();

  Time timeNow = common::Time::GetWallTime();

  double dt = (timeNow - this->timePrev).Double();
  if ( dt < 1.0/this->sampleRate)
    return;

  this->timePrev = timeNow;

  int pts = 0;
  if (this->videoPts != -1)
  {
    pts = this->fps * this->totalTime;
    this->totalTime += dt;

    if (this->videoPts == pts)
      return;
  }

  this->videoPts = pts;

  if (!this->swsCtx)
  {
    avpicture_alloc(this->pic, PIX_FMT_RGB24, _w, _h);
    this->swsCtx = sws_getContext(_w, _h, PIX_FMT_RGB24, this->codecCtx->width,
        this->codecCtx->height, this->codecCtx->pix_fmt, SWS_BICUBIC, NULL,
        NULL, NULL);
    if (this->swsCtx == NULL)
    {
      gzerr << "Error while calling sws_getContext\n";
      return;
    }
  }

  memcpy(this->pic->data[0], _frame, _w * _h * 3);

  sws_scale(this->swsCtx, this->pic->data, this->pic->linesize, 0,
      _h, this->avFrame->data, this->avFrame->linesize);



  this->codecCtx->coded_frame->pts = this->videoPts;

  this->outSize = avcodec_encode_video(this->codecCtx, this->outbuf,
      this->outBufferSize, this->avFrame);

  this->codecCtx->coded_frame->pts = this->videoPts;

  AVPacket avPacket;
  av_init_packet(&avPacket);

  if (outSize > 0)
  {
    //if (this->codecCtx->coded_frame->pts != (0x8000000000000000LL))
    //if (this->codecCtx->coded_frame->pts != AV_NOPTS_VALUE)
    {
      avPacket.pts= av_rescale_q(this->codecCtx->coded_frame->pts,
          this->codecCtx->time_base, this->pVideoStream->time_base);
    }
    if(this->codecCtx->coded_frame->key_frame)
       avPacket.flags |= AV_PKT_FLAG_KEY;

    avPacket.stream_index= this->pVideoStream->index;
    avPacket.data= this->outbuf;
    avPacket.size= this->outSize;
    int ret = av_interleaved_write_frame(this->pFormatCtx, &avPacket);
    av_free_packet(&avPacket);
    if (ret < 0)
      gzerr << "Error writing frame" << std::endl;
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
  if (this->pFormatCtx)
    av_write_trailer(this->pFormatCtx);
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
  this->frameWidth = 800;
  this->frameHeight = 600;
  this->fps = 25;
  this->tmpFilename = "tmp_recording";
  this->initialized = false;
  this->format = "avi";
  this->swsCtx = NULL;
  this->timePrev = 0;
  this->sampleRate = this->fps * 2;
  this->totalTime = 0;
  this->videoPts = -1;
}
