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
  this->avFrame = NULL;
  this->swsCtx = NULL;
  this->pic = NULL;
  this->initialized = false;
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
  // Close the video file
//  av_close_input_file(this->formatCtx);

  av_free(this->pic);
//#endif

  if (this->picture_buf)
  {
    delete[] this->picture_buf;
    this->picture_buf = NULL;
  }

  if (this->outbuf)
  {
    delete[] this->outbuf;
    this->outbuf = NULL;
  }

  // Close the codec
  avcodec_close(this->codecCtx);
  av_free(this->codecCtx);

  // Free the YUV frame
  av_free(this->avFrame);
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

  std::string filename = "test_encode";

  AVCodec *codec;
  int i, size, x, y;
//  int outSize;

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
  this->avFrame= avcodec_alloc_frame();

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

  this->fileHandle = fopen(filename.c_str(), "wb");
  if (!this->fileHandle)
  {
    gzerr << "Could not open '" << filename << "' for encoding\n";
    return;
  }

  // alloc image and output buffer
  outBufferSize = 100000;
  this->outbuf = new unsigned char[outBufferSize];
  size = this->codecCtx->width * this->codecCtx->height;
  this->picture_buf = new unsigned char[(size * 3) / 2];
//    this->avFrame_buf = malloc((size * 3) / 2); // size for YUV 420

  this->avFrame->data[0] = this->picture_buf;
  this->avFrame->data[1] = this->avFrame->data[0] + size;
  this->avFrame->data[2] = this->avFrame->data[1] + size / 4;
  this->avFrame->linesize[0] = this->codecCtx->width;
  this->avFrame->linesize[1] = this->codecCtx->width / 2;
  this->avFrame->linesize[2] = this->codecCtx->width / 2;

  /*// encode 1 second of video
  for(i=0;i<25;i++) {
    fflush(stdout);
    // prepare a dummy image
    // Y
    for(y=0;y<codecCtx->height;y++) {
        for(x=0;x<codecCtx->width;x++) {
            this->avFrame->data[0][y * this->avFrame->linesize[0] + x] = x + y + i * 3;
        }
    }

    // Cb and Cr
    for(y=0;y<codecCtx->height/2;y++) {
        for(x=0;x<codecCtx->width/2;x++) {
            this->avFrame->data[1][y * this->avFrame->linesize[1] + x]
                = 128 + y + i * 2;
            this->avFrame->data[2][y * this->avFrame->linesize[2] + x]
                = 64 + x + i * 5;
        }
    }

    // encode the image
    this->outSize = avcodec_encode_video(this->codecCtx, this->outbuf,
        this->outBufferSize, this->avFrame);
    printf("encoding frame %3d (size=%5d)\n", i, this->outSize);
    fwrite(this->outbuf, 1, this->outSize, this->fileHandle);
  }*/
  this->initialized = true;
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
void Encoder::EncodeFrame(unsigned char *_frame, unsigned int _w,
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
  this->outSize = avcodec_encode_video(this->codecCtx, this->outbuf,
      this->outBufferSize, this->avFrame);
//  printf("encoding frame %3d (size=%5d)\n", i, this->outSize);
  fwrite(this->outbuf, 1, this->outSize, this->fileHandle);
}

/////////////////////////////////////////////////
void Encoder::Fini()
{
  if (!this->initialized)
    return;

  // get the delayed frames
  for(; outSize; /*i++*/)
  {
    fflush(stdout);
    outSize = avcodec_encode_video(this->codecCtx, this->outbuf,
        this->outBufferSize, NULL);
//    printf("write frame %3d (size=%5d)\n", i, outSize);
    fwrite(this->outbuf, 1, this->outSize, this->fileHandle);
  }

  // add sequence end code to have a real mpeg file
  this->outbuf[0] = 0x00;
  this->outbuf[1] = 0x00;
  this->outbuf[2] = 0x01;
  this->outbuf[3] = 0xb7;
  fwrite(this->outbuf, 1, 4, this->fileHandle);
  fclose(this->fileHandle);

  this->Cleanup();
}

/////////////////////////////////////////////////
void Encoder::Reset()
{
  this->Cleanup();
  // set default values
  this->bitRate = 400000;
  this->frameWidth = 352;
  this->frameHeight = 288;
}

/////////////////////////////////////////////////
static void video_encode_example(const char *filename)
{

    AVCodec *codec;
    AVCodecContext *c= NULL;
    int i, out_size, size, x, y, outbuf_size;
    FILE *f;
    AVFrame *picture;
//    uint8_t *outbuf, *picture_buf;
    unsigned char *outbuf, *picture_buf;

    printf("Video encoding\n");

    /* find the mpeg1 video encoder */
    codec = avcodec_find_encoder(CODEC_ID_MPEG1VIDEO);
    if (!codec) {
        fprintf(stderr, "codec not found\n");
        exit(1);
    }

    c= avcodec_alloc_context();
    picture= avcodec_alloc_frame();

    /* put sample parameters */
    c->bit_rate = 400000;
    /* resolution must be a multiple of two */
    c->width = 352;
    c->height = 288;
    /* frames per second */
    c->time_base= (AVRational){1,25};
    c->gop_size = 10; /* emit one intra frame every ten frames */
    c->max_b_frames=1;
    c->pix_fmt = PIX_FMT_YUV420P;

    /* open it */
    if (avcodec_open(c, codec) < 0) {
        fprintf(stderr, "could not open codec\n");
        exit(1);
    }

    f = fopen(filename, "wb");
    if (!f) {
        fprintf(stderr, "could not open %s\n", filename);
        exit(1);
    }

    /* alloc image and output buffer */
    outbuf_size = 100000;
//    outbuf = malloc(outbuf_size);
    outbuf = new unsigned char[outbuf_size];
    size = c->width * c->height;
//    picture_buf = malloc((size * 3) / 2); /* size for YUV 420 */
    picture_buf = new unsigned char[(size * 3) / 2]; /* size for YUV 420 */

    picture->data[0] = picture_buf;
    picture->data[1] = picture->data[0] + size;
    picture->data[2] = picture->data[1] + size / 4;
    picture->linesize[0] = c->width;
    picture->linesize[1] = c->width / 2;
    picture->linesize[2] = c->width / 2;

    /* encode 1 second of video */
    for(i=0;i<25;i++) {
        fflush(stdout);
        /* prepare a dummy image */
        /* Y */
        for(y=0;y<c->height;y++) {
            for(x=0;x<c->width;x++) {
                picture->data[0][y * picture->linesize[0] + x] = x + y + i * 3;
            }
        }

        /* Cb and Cr */
        for(y=0;y<c->height/2;y++) {
            for(x=0;x<c->width/2;x++) {
                picture->data[1][y * picture->linesize[1] + x] = 128 + y + i * 2;
                picture->data[2][y * picture->linesize[2] + x] = 64 + x + i * 5;
            }
        }

        /* encode the image */
        out_size = avcodec_encode_video(c, outbuf, outbuf_size, picture);
        printf("encoding frame %3d (size=%5d)\n", i, out_size);
        fwrite(outbuf, 1, out_size, f);
    }

    /* get the delayed frames */
    for(; out_size; i++) {
        fflush(stdout);

        printf("out size %5d ", out_size);
        out_size = avcodec_encode_video(c, outbuf, outbuf_size, NULL);
        printf("write frame %3d (size=%5d)\n", i, out_size);
        fwrite(outbuf, 1, out_size, f);
    }

    /* add sequence end code to have a real mpeg file */
    outbuf[0] = 0x00;
    outbuf[1] = 0x00;
    outbuf[2] = 0x01;
    outbuf[3] = 0xb7;
    fwrite(outbuf, 1, 4, f);
    fclose(f);
//    free(picture_buf);
//    free(outbuf);
    delete[] picture_buf;
    delete[] outbuf;

    avcodec_close(c);
    av_free(c);
    av_free(picture);
    printf("\n");

/*//    int codec_id = AV_CODEC_ID_MPEG1VIDEO;
    int codec_id = CODEC_ID_MPEG1VIDEO;
    AVCodec *codec;
    AVCodecContext *codec= NULL;
    int i, ret, x, y, got_output;
    FILE *this->fileHandle;
    this->avFrame *frame;
    AVPacket pkt;
    uint8_t endcode[] = { 0, 0, 1, 0xb7 };

    printf("Encode video file %s\n", filename);

    // find the mpeg1 video encoder
    codec = avcodec_find_encoder(codec_id);
    if (!codec) {
        fprintf(stderr, "Codec not found\n");
        exit(1);
    }

    codec = avcodec_alloc_context3(codec);
    if (!codec) {
        fprintf(stderr, "Could not allocate video codec context\n");
        exit(1);
    }

    // put sample parameters
    codec->bit_rate = 400000;
    // resolution must be a multiple of two
    codec->width = 352;
    codec->height = 288;
    // frames per second
    codec->time_base= (AVRational){1,25};
    codec->gop_size = 10; // emit one intra frame every ten frames
    codec->max_b_frames=1;
//    codec->pix_fmt = AV_PIX_FMT_YUV420P;
    codec->pix_fmt = PIX_FMT_YUV420P;

//    if(codec_id == AV_CODEC_ID_H264)
//    if(codec_id == CODEC_ID_H264)
//        av_opt_set(codec->priv_data, "preset", "slow", 0);

    // open it
    if (avcodec_open2(codec, codec, NULL) < 0) {
        fprintf(stderr, "Could not open codec\n");
        exit(1);
    }

    this->fileHandle = fopen(filename, "wb");
    if (!this->fileHandle) {
        fprintf(stderr, "Could not open %s\n", filename);
        exit(1);
    }

    frame = avcodec_alloc_frame();
    if (!frame) {
        fprintf(stderr, "Could not allocate video frame\n");
        exit(1);
    }
    frame->format = codec->pix_fmt;
    frame->width  = codec->width;
    frame->height = codec->height;

    // the image can be allocated by any means and av_image_alloc() is
    // just the most convenient way if av_malloc() is to be used
    ret = av_image_alloc(frame->data, frame->linesize, codec->width, codec->height, codec->pix_fmt, 32);
    if (ret < 0) {
        fprintf(stderr, "Could not allocate raw this->avFrame buffer\n");
        exit(1);
    }

    // encode 1 second of video
    for(i=0;i<25;i++) {
        av_init_packet(&pkt);
        pkt.data = NULL;    // packet data will be allocated by the encoder
        pkt.size = 0;

        fflush(stdout);
        // prepare a dummy image
        // Y
        for(y=0;y<codec->height;y++) {
            for(x=0;x<codec->width;x++) {
                frame->data[0][y * frame->linesize[0] + x] = x + y + i * 3;
            }
        }

        // Cb and Cr
        for(y=0;y<codec->height/2;y++) {
            for(x=0;x<codec->width/2;x++) {
                frame->data[1][y * frame->linesize[1] + x] = 128 + y + i * 2;
                frame->data[2][y * frame->linesize[2] + x] = 64 + x + i * 5;
            }
        }

        frame->pts = i;

        // encode the image
        ret = avcodec_encode_video2(codec, &pkt, frame, &got_output);
        if (ret < 0) {
            fprintf(stderr, "Error encoding frame\n");
            exit(1);
        }

        if (got_output) {
            printf("Write frame %3d (size=%5d)\n", i, pkt.size);
            fwrite(pkt.data, 1, pkt.size, this->fileHandle);
            av_free_packet(&pkt);
        }
    }

    // get the delayed frames
    for (got_output = 1; got_output; i++) {
        fflush(stdout);

        ret = avcodec_encode_video2(codec, &pkt, NULL, &got_output);
        if (ret < 0) {
            fprintf(stderr, "Error encoding frame\n");
            exit(1);
        }

        if (got_output) {
            printf("Write frame %3d (size=%5d)\n", i, pkt.size);
            fwrite(pkt.data, 1, pkt.size, this->fileHandle);
            av_free_packet(&pkt);
        }
    }

    // add sequence end code to have a real mpeg file
    fwrite(endcode, 1, sizeof(endcode), this->fileHandle);
    fclose(this->fileHandle);

    avcodec_close(codec);
    av_free(codec);
    av_freep(&frame->data[0]);
//    avcodec_free_frame(&frame);
    av_free(frame);
    printf("\n");*/

}

/////////////////////////////////////////////////
void Encoder::Init2()
{
  video_encode_example("test_encode");
}
