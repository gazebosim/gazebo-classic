/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include <gazebo/common/Video.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/gazebo_config.h>

#ifdef HAVE_FFMPEG
#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}
#endif

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
// Declaring this function here to avoid including libavcodec/format headers
// in Video.hh
#ifdef HAVE_FFMPEG
static void pgm_save(unsigned char *buf, int wrap, int xsize, int ysize,
                     char *filename)
{
  FILE *f;
  int i;

  f=fopen(filename,"w");
  fprintf(f,"P6\n%d %d\n%d\n",xsize,ysize,255);
  for(i=0;i<ysize;i++)
    fwrite(buf + i * wrap,1,xsize*3,f);
  fclose(f);
}
#endif

/////////////////////////////////////////////////
Video::Video()
{
}

/////////////////////////////////////////////////
Video::~Video()
{
}

/////////////////////////////////////////////////
void Video::Init()
{
#ifdef HAVE_FFMPEG
  av_register_all();
#endif
}

/////////////////////////////////////////////////
#ifdef HAVE_FFMPEG
bool Video::Read(const std::string &_filename)
{
  AVCodec *codec;
  AVPacket packet, tmpPacket;
  AVFrame *avFrame = avcodec_alloc_frame();
  AVFormatContext *formatCtx = NULL;
  AVCodecContext *codecCtx = NULL;
  AVPicture pic;

  char buf[1024];
  int videoStream = -1;
  int frame = 0;
  int frameAvailable = 0;
  int processedLength = 0;

  av_init_packet(&packet);

  // Open video file
  if (avformat_open_input(&formatCtx, _filename.c_str(), NULL, NULL) < 0)
  {
    gzerr << "Unable to read video file[" << _filename << "]\n";
    return false;
  }

  // Retrieve stream information
  if (av_find_stream_info(formatCtx) < 0)
  {
    gzerr << "Couldn't find stream information\n";
    return false;
  }

  // Find the first video stream
  for (unsigned int i = 0; i < formatCtx->nb_streams; ++i)
  {
    if (formatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO)
    {
      videoStream = static_cast<int>(i);
      break;
    }
  }

  if (videoStream == -1)
  {
    gzerr << "Unable to find a video stream\n";
    return false;
  }

  // Get a pointer to the codec context for the video stream
  codecCtx = formatCtx->streams[videoStream]->codec;

  // Find the decoder for the video stream
  codec = avcodec_find_decoder(codecCtx->codec_id);
  if (codec == NULL)
  {
    gzerr << "Codec not found\n";
    return false;
  }

  // Inform the codec that we can handle truncated bitstreams -- i.e.,
  // bitstreams where frame boundaries can fall in the middle of packets
  if (codec->capabilities & CODEC_CAP_TRUNCATED)
    codecCtx->flags |= CODEC_FLAG_TRUNCATED;

  // Open codec
  if (avcodec_open2(codecCtx, codec, NULL) < 0)
  {
    gzerr << "Could not open codec\n";
    return false;
  }

  avpicture_alloc(&pic, PIX_FMT_RGB24, codecCtx->width, codecCtx->height);
  SwsContext *ctxt = sws_getContext(
      codecCtx->width,
      codecCtx->height,
      codecCtx->pix_fmt,
      codecCtx->width,
      codecCtx->height,
      PIX_FMT_RGB24,
      SWS_BICUBIC, NULL, NULL, NULL);

  if (ctxt == NULL)
  {
    gzerr << "Error while calling sws_getContext\n";
    return false;
  }

  // the decoding loop, running until EOF
  while (av_read_frame(formatCtx, &packet) >= 0)
  {
    if (packet.stream_index == videoStream)
    {
      processedLength = 0;
      tmpPacket.data = packet.data;
      tmpPacket.size = packet.size;

      // Process all the data in the frame
      while (tmpPacket.size > 0)
      {
        // sending data to libavcodec
        processedLength = avcodec_decode_video2(codecCtx, avFrame,
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
          sws_scale(ctxt, avFrame->data, avFrame->linesize, 0,
                    codecCtx->height, pic.data, pic.linesize);

          printf("WH[%d %d]\n",codecCtx->width, codecCtx->height);
          snprintf(buf, sizeof(buf), "/tmp/test_%3d.ppm", frame++);
          pgm_save(pic.data[0], pic.linesize[0], codecCtx->width,
                   codecCtx->height, buf);
        }
      }
    }
    av_free_packet(&packet);
  }

  // Free the YUV frame
  av_free(avFrame);

  // Close the codec
  avcodec_close(codecCtx);

  // Close the video file
  av_close_input_file(formatCtx);

  return true;
}
#else
bool Video::Read(const std::string &/*_filename*/)
{
  return false;
}
#endif
