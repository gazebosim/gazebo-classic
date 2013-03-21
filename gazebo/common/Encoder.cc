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
#include "libavutil/mathematics.h"

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
  this->outbuf = NULL;
  this->pictureBuf = NULL;
  this->codecCtx = NULL;
  this->swsCtx = NULL;
  this->pic = NULL;
  this->avFrame = NULL;
  this->pFormatCtx = NULL;

//#ifdef HAVE_FFMPEG
  static bool first = true;
  if (first)
  {
    first = false;
    av_register_all();
  }

  this->Reset();
//  this->time = common::Time();

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


/*  // Close the codec
  if (this->codecCtx)
  {
    avcodec_close(this->codecCtx);
    av_free(this->codecCtx);
    this->codecCtx = NULL;
  }*/

  if (this->pFormatCtx)
  {
    for(unsigned int i = 0; i < this->pFormatCtx->nb_streams; ++i)
    {
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

//#ifdef HAVE_FFMPEG
  if (this->pic)
  {
    av_free(this->pic);
    this->pic = NULL;
  }
//#endif

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

  printf("Video encoding\n");

  this->filename = "test_encode.mpeg";

  this->pOutputFormat = av_guess_format(NULL, this->filename.c_str(), NULL);
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
      "%s", this->filename.c_str());
  this->pVideoStream = avformat_new_stream(this->pFormatCtx, codec);

  // some formats want stream headers to be separate
  if(this->pFormatCtx->oformat->flags & AVFMT_GLOBALHEADER)
    this->codecCtx->flags |= CODEC_FLAG_GLOBAL_HEADER;

  this->codecCtx = this->pVideoStream->codec;
//  this->codecCtx->codec_id = this->pOutputFormat->video_codec;
//  this->codecCtx->codec_type = AVMEDIA_TYPE_VIDEO;


//  this->codecCtx = avcodec_alloc_context3(codec);
  // put sample parameters
  this->codecCtx->bit_rate = this->bitRate;
  // resolution must be a multiple of two
  this->codecCtx->width = this->frameWidth;
  this->codecCtx->height = this->frameHeight;
  // frames per second
  this->codecCtx->time_base.den= this->fps;
  this->codecCtx->time_base.num= 1;
  this->codecCtx->gop_size = 12; // emit one intra frame every ten frames
  this->codecCtx->max_b_frames=1;
  this->codecCtx->pix_fmt = PIX_FMT_YUV420P;
  this->codecCtx->thread_count = 3;
  if (this->codecCtx->codec_id == CODEC_ID_MPEG1VIDEO)
  {
    // Needed to avoid using macroblocks in which some coeffs overflow.
    // This does not happen with normal video, it just happens here as
    // the motion of the chroma plane does not match the luma plane.
    this->codecCtx->mb_decision = 2;
  }
//  avcodec_thread_init(this->codecCtx, 10);




/*  if (av_set_parameters(this->pFormatCtx, NULL) < 0)
  {
    gzerr << "Invalid output format parameters\n";
    return;
  }
  dump_format(this->pFormatCtx, 0, filename.c_str(), 1);*/

  // open it
  if (avcodec_open2(this->codecCtx, codec, NULL) < 0)
  {
    gzerr << "Could not open codec\n";
    return;
  }
  this->pic = new AVPicture;

  this->fileHandle = fopen(filename.c_str(), "wb");
  if (!this->fileHandle)
  {
    gzerr << "Could not open '" << filename << "' for encoding\n";
    return;
  }

/*  if (url_fopen(&this->pFormatCtx->pb, filename.c_str(), URL_WRONLY) < 0)
  {
    gzerr << "Could not open '" << filename << "\n";
    return;
  }*/

/*  this->avFrame = avcodec_alloc_frame();
  // size for YUV 420
  this->pictureBuf = new unsigned char[(this->outputFrameSize * 3) / 2];
  this->avFrame->data[0] = this->pictureBuf;
  this->avFrame->data[1] = this->avFrame->data[0] + this->outputFrameSize;
  this->avFrame->data[2] = this->avFrame->data[1] + this->outputFrameSize / 4;
  this->avFrame->linesize[0] = this->codecCtx->width;
  this->avFrame->linesize[1] = this->codecCtx->width / 2;
  this->avFrame->linesize[2] = this->codecCtx->width / 2;*/
  this->avFrame = avcodec_alloc_frame();
  int size = avpicture_get_size(this->codecCtx->pix_fmt, this->codecCtx->width,
      this->codecCtx->height);
  this->pictureBuf = new unsigned char[size];
  avpicture_fill((AVPicture *)this->avFrame, this->pictureBuf,
      this->codecCtx->pix_fmt, this->codecCtx->width, this->codecCtx->height);

  av_dump_format(this->pFormatCtx, 0, this->filename.c_str(), 1);

  if (!(this->pOutputFormat->flags & AVFMT_NOFILE))
  {
    if (avio_open(&this->pFormatCtx->pb, this->filename.c_str(),
        AVIO_FLAG_WRITE) < 0)
    {
      gzerr << "Could not open '" << this->filename << "'\n";
      return;
    }
  }
  // Write the stream header, if any.
  avformat_write_header(this->pFormatCtx, NULL);

  // alloc image and output buffer
  this->outBufferSize = 100000;
  this->outbuf = new unsigned char[this->outBufferSize];
  this->outputFrameSize = this->codecCtx->width * this->codecCtx->height;

//  av_write_header(this->pFormatCtx);
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

int ppp = -1;

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

  sws_scale(this->swsCtx, this->pic->data, this->pic->linesize, 0,
      _h, this->avFrame->data, this->avFrame->linesize);

  ppp += 2;
  this->codecCtx->coded_frame->pts = ppp;

  this->outSize = avcodec_encode_video(this->codecCtx, this->outbuf,
      this->outBufferSize, this->avFrame);
//  fwrite(this->outbuf, 1, this->outSize, this->fileHandle);

  this->codecCtx->coded_frame->pts = ppp;
/*
  AVPacket avPacket;
  av_init_packet(&avPacket);
  avPacket.data = NULL;
  int ret = avcodec_encode_video2(this->codecCtx, &avPacket,
      this->avFrame, &this->outSize);
  if (ret < 0)
    gzerr << "Error encding frame\n";
  fwrite(avPacket.data, 1, avPacket.size, this->fileHandle);
  av_free_packet(&avPacket);*/


  AVPacket avPacket;
  av_init_packet(&avPacket);

  if (outSize > 0)
  {
    //if (pCodecCtx->coded_frame->pts != AV_NOPTS_VALUE)
    if (this->codecCtx->coded_frame->pts != (0x8000000000000000LL))
    {
      avPacket.pts= av_rescale_q(this->codecCtx->coded_frame->pts,
          this->codecCtx->time_base, this->pVideoStream->time_base);
    }
    if(this->codecCtx->coded_frame->key_frame)
       avPacket.flags |= AV_PKT_FLAG_KEY;

    //printf("c %d. pts %d. codedframepts: %ld pkt.pts: %ld\n",custompts,pts,pCodecCtx->coded_frame->pts,pkt.pts);

    avPacket.stream_index= this->pVideoStream->index;
    avPacket.data= this->outbuf;
    avPacket.size= outSize;
    int ret = av_interleaved_write_frame(this->pFormatCtx, &avPacket);
    //printf("Wrote %d\n",ret);
//    if(ret<0)
//       return -1;
  }



//  this->frames.push_back(frame);

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

/*/////////////////////////////////////////////////
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
}*/

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

//   av_write_trailer(this->pFormatCtx);
   // Close file
//   url_fclose(this->pFormatCtx->pb);

/*  // get the delayed frames
  for(; this->outSize; /)
  {
    fflush(stdout);
    this->outSize = avcodec_encode_video(this->codecCtx, this->outbuf,
        this->outBufferSize, NULL);
    fwrite(this->outbuf, 1, this->outSize, this->fileHandle);
  }

  // add sequence end code to have a real mpeg file
  this->outbuf[0] = 0x00;
  this->outbuf[1] = 0x00;
  this->outbuf[2] = 0x01;
  this->outbuf[3] = 0xb7;

  fwrite(this->outbuf, 1, 4, this->fileHandle);
  fclose(this->fileHandle);*/

  av_write_trailer(this->pFormatCtx);

//  this->filename = _filename;
//  this->Encode();

  this->Cleanup();
}

/////////////////////////////////////////////////
void Encoder::Reset()
{
  this->Cleanup();
  // set default values
  this->bitRate = 20000000;
  this->frameWidth = 800;
  this->frameHeight = 600;
  this->fps = 25;
  this->filename = "EncodedVideo.mpeg";
  this->outputFrameSize = this->frameWidth * this->frameHeight;
  this->initialized = false;
  this->format = "mpeg";
  this->swsCtx = NULL;
}
