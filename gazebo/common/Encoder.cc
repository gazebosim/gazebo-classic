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

#include <gazebo/gazebo_config.h>

//#ifdef HAVE_FFMPEG
#ifndef INT64_C
#define INT64_C(codec) (codec ## LL)
#define UINT64_C(codec) (codec ## ULL)
#endif

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}
//#endif

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Encoder.hh"

using namespace gazebo;
using namespace common;


/////////////////////////////////////////////////
Encoder::Encoder()
{
//  this->formatCtx = NULL;
  this->codecCtx = NULL;
  this->swsCtx = NULL;
  this->pic = NULL;

//#ifdef HAVE_FFMPEG
  static bool first = true;
  if (first)
  {
    first = false;
    av_register_all();
  }

  this->Reset();

//  this->pic = new AVthis->avFrame;
//#endif
}

/////////////////////////////////////////////////
Encoder::~Encoder()
{
  this->Cleanup();
//#ifdef HAVE_FFMPEG
  delete this->pic;
//#endif
}

/////////////////////////////////////////////////
void Encoder::Cleanup()
{
  if (!this->initialized)
    return;
//#ifdef HAVE_FFMPEG
  av_free(this->pic);
//#endif

  if (this->outbuf)
  {
    delete[] this->outbuf;
    this->outbuf = NULL;
  }

  // Close the codec
  avcodec_close(this->codecCtx);
  av_free(this->codecCtx);

  this->initialized = false;
}

/////////////////////////////////////////////////
void Encoder::SetBitRate(unsigned int _bitRate)
{
  this->bitRate = _bitRate;
}

/////////////////////////////////////////////////
void Encoder:: SetFrameWidth(unsigned int _width)
{
  this->frameWidth = _width;
}

/////////////////////////////////////////////////
void Encoder:: SetFrameHeight(unsigned int _height)
{
  this->frameHeight = _height;
}
/////////////////////////////////////////////////
void Encoder::Init()
{
  if (this->initialized)
    return;

  AVCodec *codec;

  printf("Video encoding\n");

  this->pic = new AVPicture;

  // find the mpeg1 video encoder
  codec = avcodec_find_encoder(CODEC_ID_MPEG1VIDEO);
  if (!codec)
  {
    gzerr << "Codec not found\n";
    return;
  }

  this->codecCtx= avcodec_alloc_context();
  // put sample parameters
  this->codecCtx->bit_rate = this->bitRate;
  // resolution must be a multiple of two
  this->codecCtx->width = this->frameWidth;
  this->codecCtx->height = this->frameHeight;
  // frames per second
  this->codecCtx->time_base= (AVRational){1,25};
  this->codecCtx->gop_size = 10; // emit one intra frame every ten frames
  this->codecCtx->max_b_frames=1;
  this->codecCtx->pix_fmt = PIX_FMT_YUV420P;

  // open it
  if (avcodec_open(this->codecCtx, codec) < 0)
  {
    gzerr << "Could not open codec\n";
    return;
  }

  // alloc image and output buffer
  this->outBufferSize = 100000;
  this->outbuf = new unsigned char[this->outBufferSize];
  this->outputFrameSize = this->codecCtx->width * this->codecCtx->height;

  this->initialized = true;
}

////////////////////////////////////////////////
bool Encoder::IsInitialized()
{
  return this->initialized;
}

////////////////////////////////////////////////
// #ifdef HAVE_FFMPEG
 static void pgm_save(unsigned char *buf, int wrap, int xsize, int ysize,
                      char *filename)
 {
   FILE *f;
   int i;

   f = fopen(filename, "w");
   fprintf(f, "P6\n%d %d\n%d\n", xsize, ysize, 255);
   for(i = 0; i < ysize; i++)
     fwrite(buf + i * wrap, 1, xsize * 3, f);
   fclose(f);
 }
// #endif

/////////////////////////////////////////////////
void Encoder::AddFrame(unsigned char *_frame, unsigned int _w,
    unsigned int _h)
{
  if (!this->initialized)
    this->Init();

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

//  sws_scale(this->swsCtx, this->pic->data, this->pic->linesize, 0,
//  _h, this->avFrame->data, this->avFrame->linesize);

  AVFrame *frame = avcodec_alloc_frame();
  // size for YUV 420
  unsigned char *pictureBuf
      = new unsigned char[(this->outputFrameSize * 3) / 2];
  frame->data[0] = pictureBuf;
  frame->data[1] = frame->data[0] + this->outputFrameSize;
  frame->data[2] = frame->data[1] + this->outputFrameSize / 4;
  frame->linesize[0] = this->codecCtx->width;
  frame->linesize[1] = this->codecCtx->width / 2;
  frame->linesize[2] = this->codecCtx->width / 2;

  sws_scale(this->swsCtx, this->pic->data, this->pic->linesize, 0,
    _h, frame->data, frame->linesize);

  this->frames.push_back(frame);

/*///////////////////////////
  /// TODO testing, take me out later
  AVPicture *pic2 = new AVPicture;
  avpicture_alloc(pic2, PIX_FMT_RGB24, this->codecCtx->width,
                this->codecCtx->height);

  SwsContext *swsCtx2;
  swsCtx2 = sws_getContext(
      this->codecCtx->width,
      this->codecCtx->height,
      PIX_FMT_YUV420P,
      this->codecCtx->width,
      this->codecCtx->height,
      PIX_FMT_RGB24,
      SWS_BICUBIC, NULL, NULL, NULL);

  sws_scale(swsCtx2, this->avFrame->data, this->avFrame->linesize, 0,
    this->codecCtx->height, pic2->data, pic2->linesize);

//   Debug:
   pgm_save(pic2->data[0], pic2->linesize[0],
            this->codecCtx->width, this->codecCtx->height, "encode_debug");
*/
/*  SwsContext *swsCtx2;
  swsCtx2 = sws_getContext(
      _w,
      _h,
      PIX_FMT_RGB24,
      this->codecCtx->width,
      this->codecCtx->height,
      PIX_FMT_RGB24,
      SWS_BICUBIC, NULL, NULL, NULL);


//   pgm_save(pic->data[0], pic->linesize[0],
//            _w, _h, "encode_debug");
///////////////////////////*/

  // encode the image
/*  this->outSize = avcodec_encode_video(this->codecCtx, this->outbuf,
      this->outBufferSize, this->avFrame);

  currentBufferSize += outSize;

//  this->frames.push_back(std::make_pair(this->outbuf, this->outSize);
//  printf("encoding frame %3d (size=%5d)\n", i, this->outSize);
  fwrite(this->outbuf, 1, this->outSize, this->fileHandle);*/
}

/////////////////////////////////////////////////
void Encoder::Encode()
{
  FILE *outFile = fopen(this->filename.c_str(), "wb");
  if (!outFile)
  {
    gzerr << "Could not open '" << this->filename.c_str() << "' for encoding\n";
    return;
  }

  unsigned int outSize;
  for (int i = 0; i < this->frames.size(); ++i)
  {
    outSize = avcodec_encode_video(this->codecCtx, this->outbuf,
        this->outBufferSize, this->frames[i]);
    fwrite(this->outbuf, 1, outSize, outFile);

    // free resource
    delete[] this->frames[i]->data[0];
    av_free(this->frames[i]);
  }

  // get the delayed frames
  for (; outSize; )
  {
    fflush(stdout);
    outSize = avcodec_encode_video(this->codecCtx, this->outbuf,
        this->outBufferSize, NULL);
    currentBufferSize += outSize;
//    printf("write frame %3d (size=%5d)\n", i, outSize);
    fwrite(this->outbuf, 1, outSize, outFile);
  }

  // add sequence end code to have a real mpeg file
  this->outbuf[0] = 0x00;
  this->outbuf[1] = 0x00;
  this->outbuf[2] = 0x01;
  this->outbuf[3] = 0xb7;

  fwrite(this->outbuf, 1, 4, outFile);
  fclose(outFile);
}

/////////////////////////////////////////////////
void Encoder::SaveToFile(const std::string &_filename)
{
//  if (this->initialized)
//    this->Fini();

/*  this->fileHandle = fopen(_filename.c_str(), "wb");
  if (!this->fileHandle)
  {
    gzerr << "Could not open '" << _filename << "' for encoding\n";
    return;
  }

fwrite(bufferHead, 1, currentBufferSize, this->fileHandle);

  for (int i = 0; i < 4; i++)
    this->outbuf--;


  fclose(this->fileHandle);*/

  this->filename = _filename;
  this->Encode();

  this->Cleanup();
}

/////////////////////////////////////////////////
void Encoder::Reset()
{
  this->Cleanup();
  // set default values
  this->bitRate = 400000;
  this->frameWidth = 800;
  this->frameHeight = 600;
  this->filename = "EncodedVideo";
  this->outputFrameSize = this->frameWidth * this->frameHeight;
  this->initialized = false;
}
