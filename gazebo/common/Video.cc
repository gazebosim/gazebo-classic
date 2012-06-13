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
#include <gazebo/gazebo_config.h>

#ifdef HAVE_FFMPEG
#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#endif


using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
// Declaring this function here to avoid including libavcodec/format headers
// in Video.hh
#ifdef HAVE_FFMPEG
bool GetNextFrame(AVFormatContext *_formatCtx, AVCodecContext *_codecCtx,
                  int _videoStream, AVFrame *_frame)
{
  static AVPacket packet;
  static int bytesRemaining = 0;
  static uint8_t *rawData;
  static bool fFirstTime=true;
  int bytesDecoded;
  int frameFinished;
  bool done = false;

  // First time we're called, set packet.data to NULL to indicate it
  // doesn't have to be freed
  if (fFirstTime)
  {
    fFirstTime = false;
    packet.data = NULL;
  }

  // Decode packets until we have decoded a complete frame
  while (!done)
  {
    // Work on the current packet until we have decoded all of it
    while (bytesRemaining > 0)
    {
      // Decode the next chunk of data
      bytesDecoded = avcodec_decode_video(_codecCtx, _frame, &frameFinished,
                                          rawData, bytesRemaining);

      // Was there an error?
      if (bytesDecoded < 0)
      {
        gzerr << "Error while decoding frame\n";
        return false;
      }

      bytesRemaining -= bytesDecoded;
      rawData += bytesDecoded;

      // Did we finish the current frame? Then we can return
      if (frameFinished)
        return true;
    }

    // Read the next packet, skipping all packets that aren't for this stream
    do
    {
      // Free old packet
      if (packet.data != NULL)
        av_free_packet(&packet);

      // Read new packet
      if (av_read_packet(_formatCtx, &packet) < 0)
      {
        done = true;
        break;
      }
    } while (packet.stream_index != _videoStream);

    if (!done)
    {
      bytesRemaining = packet.size;
      rawData = packet.data;
    }
  }

  // Decode the rest of the last frame
  bytesDecoded = avcodec_decode_video(_codecCtx, _frame, &frameFinished, 
                                      rawData, bytesRemaining);

  // Free last packet
  if (packet.data != NULL)
    av_free_packet(&packet);

  return frameFinished != 0;
}

void SaveFrame(AVFrame *pFrame, int width, int height, int iFrame)
{
  FILE *pFile;
  char szFilename[32];
  int  y;

  // Open file
  sprintf(szFilename, "frame%d.ppm", iFrame);
  pFile=fopen(szFilename, "wb");
  if(pFile==NULL)
    return;

  // Write header
  fprintf(pFile, "P6\n%d %d\n255\n", width, height);

  // Write pixel data
  for(y=0; y<height; y++)
    fwrite(pFrame->data[0]+y*pFrame->linesize[0], 1, width*3, pFile);

  // Close file
  fclose(pFile);
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
bool Video:Read(const std::string &_filename)
{
  AVFormatContext *pFormatCtx;
  AVCodecContext  *pCodecCtx;
  AVCodec *pCodec;
  int i, videoStream;

  // Open video file
  if(av_open_input_file(&pFormatCtx, _filename.c_str(), NULL, 0, NULL)!=0)
  {
    gzerr << "Unable to read video file[" << _filename << "]\n";
    return false;
  }

  // Retrieve stream information
  if (av_find_stream_info(pFormatCtx) < 0)
  {
    gzerr << "Couldn't find stream information\n";
    return false;
  }

  // Find the first video stream
  videoStream = -1;
  for (i = 0; i < pFormatCtx->nb_streams; ++i)
  {
    if (pFormatCtx->streams[i]->codec.codec_type == CODEC_TYPE_VIDEO)
    {
      videoStream = i;
      break;
    }
  }

  if (videoStream == -1)
  {
    gzerr << "Unable to find a video stream\n";
    return false;
  }

  // Get a pointer to the codec context for the video stream
  pCodecCtx=&pFormatCtx->streams[videoStream]->codec;

  // Find the decoder for the video stream
  pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
  if (pCodec == NULL)
  {
    gzerr << "Codec not found\n";
    return;
  }

  // Inform the codec that we can handle truncated bitstreams -- i.e.,
  // bitstreams where frame boundaries can fall in the middle of packets
  if (pCodec->capabilities & CODEC_CAP_TRUNCATED)
    pCodecCtx->flags |= CODEC_FLAG_TRUNCATED;

  // Open codec
  if (avcodec_open(pCodecCtx, pCodec) < 0)
    gzerr << "Could not open codec\n";

  // Allocate video frame
  pFrame = avcodec_alloc_frame();

  // Allocate an AVFrame structure
  pFrameRGB = avcodec_alloc_frame();

  if (pFrameRGB == NULL)
    return false;

  // Determine required buffer size and allocate buffer
  numBytes = avpicture_get_size(PIX_FMT_RGB24, pCodecCtx->width,
                                pCodecCtx->height);
  buffer = new uint8_t[numBytes];

  // Assign appropriate parts of buffer to image planes in pFrameRGB
  avpicture_fill((AVPicture *)pFrameRGB, buffer, PIX_FMT_RGB24,
                 pCodecCtx->width, pCodecCtx->height);

  // Read frames and save first five frames to disk
  i = 0;
  while (GetNextFrame(pFormatCtx, pCodecCtx, videoStream, pFrame))
  {
    img_convert((AVPicture *)pFrameRGB, PIX_FMT_RGB24, (AVPicture*)pFrame, 
        pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height);

    // Save the frame to disk
    if(++i<=5)
      SaveFrame(pFrameRGB, pCodecCtx->width, pCodecCtx->height, i);
  }

  // Free the RGB image
  delete [] buffer;
  av_free(pFrameRGB);

  // Free the YUV frame
  av_free(pFrame);

  // Close the codec
  avcodec_close(pCodecCtx);

  // Close the video file
  av_close_input_file(pFormatCtx);

  return true;
}
#else
bool Video::Read(const std::string &/*_filename*/)
{
  return false;
}
#endif
