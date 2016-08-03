/*
 * Copyright 2016 Open Source Robotics Foundation
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
#include <mutex>
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
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
}
#endif

#include "gazebo/math/Helpers.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/VideoEncoder.hh"

using namespace gazebo;
using namespace common;

// Private data class
class gazebo::common::VideoEncoderPrivate
{
  /// \brief Get the full filename of the temporary video file
  public: std::string TmpFilename() const;

  /// \brief libav muxing
  public: AVOutputFormat *outputFormat = nullptr;

  /// \brief libav format I/O context
  public: AVFormatContext *formatCtx = nullptr;

  /// \brief libav audio video stream
  public: AVStream *videoStream = nullptr;

  /// \brief libav image data (used for storing RGB data)
  public: AVPicture *avInPicture = nullptr;

  /// \brief libav audio or video data (used for storing YUV data)
  public: AVFrame *avOutFrame = nullptr;

  /// \brief Software scaling context
  public: SwsContext *swsCtx = nullptr;

  /// \brief Size of the picture buffer
  public: uint8_t *pictureBuf = nullptr;

  /// \brief Handle to the output video file.
  public: FILE *fileHandle = nullptr;

  /// \brief True if the encoder is initialized
  public: bool encoding = false;

  /// \brief Video encoding bit rate
  public: unsigned int bitRate = VIDEO_ENCODER_BITRATE_DEFAULT;

  /// \brief Output frame width
  public: unsigned int outWidth = VIDEO_ENCODER_WIDTH_DEFAULT;

  /// \brief Output frame height
  public: unsigned int outHeight = VIDEO_ENCODER_HEIGHT_DEFAULT;

  /// \brief Input frame width
  public: unsigned int inWidth = 0;

  /// \brief Input frame height
  public: unsigned int inHeight = 0;

  /// \brief Temporary filename to write the file to.
  public: std::string tmpFilename = "TMP_RECORDING";

  /// \brief Encoding format
  public: std::string format = VIDEO_ENCODER_FORMAT_DEFAULT;

  /// \brief Target framerate.
  public: unsigned int fps = VIDEO_ENCODER_FPS_DEFAULT;

  /// \brief Previous time when the frame is added.
  public: std::chrono::steady_clock::time_point timePrev;

  public: gazebo::common::Time timePrev2;

  /// \brief Encoding sample rate.
  public: int sampleRate = VIDEO_ENCODER_FPS_DEFAULT * 2;

  /// \brief total time elapsed, in seconds.
  public: std::chrono::duration<double> totalTime;

  public: common::Time totalTime2;

  /// \brief Video presentation time stamp.
  public: int videoPts = -1;

  public: unsigned int frameCount = 0;

  public: std::mutex mutex;
};

/////////////////////////////////////////////////
VideoEncoder::VideoEncoder()
: dataPtr(new VideoEncoderPrivate)
{
#ifdef HAVE_FFMPEG
  // Register av once.
  static bool first = true;
  if (first)
  {
    first = false;
    av_register_all();
  }
#endif
}

/////////////////////////////////////////////////
VideoEncoder::~VideoEncoder()
{
  this->Reset();
}

/////////////////////////////////////////////////
std::string VideoEncoder::Format() const
{
  return this->dataPtr->format;
}

/////////////////////////////////////////////////
std::string VideoEncoderPrivate::TmpFilename() const
{
  return common::cwd() + "/" + this->tmpFilename + "." + this->format;
}

/////////////////////////////////////////////////
bool VideoEncoder::Start(const unsigned int _width,
                         const unsigned int _height,
                         const std::string &_format,
                         const unsigned int _fps,
                         const unsigned int _bitRate)
{
  if (this->dataPtr->encoding)
    return false;

  // Remove old temp file, if it exists.
  if (common::exists(this->dataPtr->TmpFilename()))
    std::remove(this->dataPtr->TmpFilename().c_str());

  this->dataPtr->bitRate = _bitRate;

  // Must be divisible by two
  this->dataPtr->outWidth = _width % 2 == 0 ? _width : _width + 1;
  this->dataPtr->outHeight = _height % 2 == 0 ? _height : _height + 1;

  std::cout << "Start width x height[" << this->dataPtr->outWidth << " " << this->dataPtr->outHeight << "]\n";
  //this->dataPtr->outWidth = std::ceil(_width/8.0) * 8.0;
  //this->dataPtr->outHeight = std::ceil(_height/8.0) * 8.0;

  this->dataPtr->format = _format;
  this->dataPtr->fps = _fps;
  this->dataPtr->frameCount = 0;
  this->dataPtr->totalTime2 = 0;

#ifdef HAVE_FFMPEG
  printf("A\n");
  std::string tmpFileNameFull = this->dataPtr->TmpFilename();

  this->dataPtr->outputFormat =
    av_guess_format(NULL, tmpFileNameFull.c_str(), NULL);
  printf("B\n");

  if (!this->dataPtr->outputFormat)
  {
    gzerr << "Could not deduce output format from file extension: "
        << "using MPEG.\n";
    this->dataPtr->outputFormat = av_guess_format("mpeg", NULL, NULL);
  }

  AVCodec *encoder = nullptr;

  printf("C\n");
  // find the video encoder
  encoder = avcodec_find_encoder(this->dataPtr->outputFormat->video_codec);
  if (!encoder)
  {
    gzerr << "Codec for[" << this->dataPtr->outputFormat->name
          << "] not found. Video encoding is not started.\n";
    this->Reset();
    return false;
  }

  printf("D\n");
  this->dataPtr->formatCtx = avformat_alloc_context();
  this->dataPtr->formatCtx->oformat = this->dataPtr->outputFormat;
  snprintf(this->dataPtr->formatCtx->filename,
      sizeof(this->dataPtr->formatCtx->filename),
      "%s", tmpFileNameFull.c_str());

  this->dataPtr->videoStream =
    avformat_new_stream(this->dataPtr->formatCtx, encoder);

  printf("E\n");
  // some formats want stream headers to be separate
  if (this->dataPtr->formatCtx->oformat->flags & AVFMT_GLOBALHEADER)
    this->dataPtr->videoStream->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;

  AVCodecContext *ctx = this->dataPtr->videoStream->codec;//avcodec_alloc_context3(encoder);
  if (!ctx)
  {
    gzerr << "Could not allocated video codex context."
      << "Video encoding is not started\n";
    this->Reset();
    return false;
  }

  printf("F\n");
  // Sample parameters
  ctx->bit_rate = this->dataPtr->bitRate;

  // Resolution
  ctx->width = this->dataPtr->outWidth;
  ctx->height = this->dataPtr->outHeight;

  // frames per second
  this->dataPtr->videoStream->time_base.den = this->dataPtr->fps;
  this->dataPtr->videoStream->time_base.num = 1;
  this->dataPtr->videoStream->codec->time_base.den = this->dataPtr->fps;
  this->dataPtr->videoStream->codec->time_base.num = 1;

  // Emit one intra frame every ten frames
  ctx->gop_size = 10;
  ctx->max_b_frames = 1;
  ctx->pix_fmt = AV_PIX_FMT_YUV420P;

  printf("G\n");
  if (ctx->codec_id == AV_CODEC_ID_H264)
    av_opt_set(ctx->priv_data, "preset", "slow", 0);

  if (ctx->codec_id == CODEC_ID_MPEG1VIDEO)
  {
    // Needed to avoid using macroblocks in which some coeffs overflow.
    // This does not happen with normal video, it just happens here as
    // the motion of the chroma plane does not match the luma plane.
    this->dataPtr->videoStream->codec->mb_decision = 2;
  }
  printf("H\n");

  // Open it
  if (avcodec_open2(ctx, encoder, nullptr) < 0)
  {
    gzerr << "Could not open codec."
      << "Video encoding is not started\n";
    this->Reset();
    return false;
  }

  // Open output file
  this->dataPtr->fileHandle = fopen(tmpFileNameFull.c_str(), "wb");
  if (!this->dataPtr->fileHandle)
  {
    gzerr << "Could not open '" << tmpFileNameFull << "' for encoding"
      << "Video encoding is not started\n";
    this->Reset();
    return false;
  }
  printf("I\n");

  this->dataPtr->avOutFrame = av_frame_alloc();
  if (!this->dataPtr->avOutFrame)
  {
    gzerr << "Could not allocate video frame."
      << "Video encoding is not started\n";
    this->Reset();
    return false;
  }

  printf("J\n");
  this->dataPtr->avOutFrame->format = ctx->pix_fmt;
  this->dataPtr->avOutFrame->width  = ctx->width;
  this->dataPtr->avOutFrame->height = ctx->height;

  // the image can be allocated by any means and av_image_alloc() is
  // just the most convenient way if av_malloc() is to be used
  if (av_image_alloc(this->dataPtr->avOutFrame->data,
      this->dataPtr->avOutFrame->linesize,
      ctx->width, ctx->height, ctx->pix_fmt, 32) < 0)
  {
    gzerr << "Could not allocate raw picture buffer."
      << "Video encoding is not started\n";
    this->Reset();
    return false;
  }

  printf("K\n");
  // setting mux preload and max delay avoids buffer underflow when writing to
  // mpeg format
  double muxMaxDelay = 0.7f;
  this->dataPtr->formatCtx->max_delay =
    static_cast<int>(muxMaxDelay * AV_TIME_BASE);

  if (!(this->dataPtr->outputFormat->flags & AVFMT_NOFILE))
  {
    if (avio_open(&this->dataPtr->formatCtx->pb, tmpFileNameFull.c_str(),
        AVIO_FLAG_WRITE) < 0)
    {
      gzerr << "Could not open '" << tmpFileNameFull << "'\n";
      this->Reset();
      return false;
    }
  }

  // Write the stream header, if any.
  if (avformat_write_header(this->dataPtr->formatCtx, nullptr) < 0)
  {
    gzerr << " Error occured when opening output file" << std::endl;
    this->Reset();
    return false;
  }
  printf("L\n");

/*


  this->dataPtr->formatCtx = avformat_alloc_context();
  this->dataPtr->formatCtx->oformat = this->dataPtr->outputFormat;
  snprintf(this->dataPtr->formatCtx->filename,
      sizeof(this->dataPtr->formatCtx->filename),
      "%s", tmpFileNameFull.c_str());

  this->dataPtr->videoStream =
    avformat_new_stream(this->dataPtr->formatCtx, encoder);

  // this->dataPtr->videoStream->id = this->dataPtr->formatCtx->nb_streams-1;

  // some formats want stream headers to be separate
  if (this->dataPtr->formatCtx->oformat->flags & AVFMT_GLOBALHEADER)
    this->dataPtr->videoStream->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;

  //---------------------
  this->dataPtr->videoStream->time_base.den = this->dataPtr->fps;
  this->dataPtr->videoStream->time_base.num = 1;
  this->dataPtr->videoStream->codec->codec_id = encoder->id;
  this->dataPtr->videoStream->codec->bit_rate = this->dataPtr->bitRate;
  this->dataPtr->videoStream->codec->gop_size = 100;
  this->dataPtr->videoStream->codec->keyint_min = 1;
  //this->dataPtr->videoStream->codec->max_b_frames = 3;
  //this->dataPtr->videoStream->codec->b_frame_strategy = 1;
  //this->dataPtr->videoStream->codec->scenechange_threshold = 40;
  //this->dataPtr->videoStream->codec->refs = 6;
  //this->dataPtr->videoStream->codec->qmin = 0;
  //this->dataPtr->videoStream->codec->qmax = 69;
  //this->dataPtr->videoStream->codec->qcompress = 0.6;
  //this->dataPtr->videoStream->codec->max_qdiff = 4;
  //this->dataPtr->videoStream->codec->i_quant_factor = 1.4;
  //this->dataPtr->videoStream->codec->refs = 1;
  //this->dataPtr->videoStream->codec->chromaoffset = -2;
  this->dataPtr->videoStream->codec->thread_count = 5;
  //this->dataPtr->videoStream->codec->trellis = 1;
  //this->dataPtr->videoStream->codec->me_range = 16;
  //this->dataPtr->videoStream->codec->me_method = ME_HEX;
  //this->dataPtr->videoStream->codec->flags |= CODEC_FLAG2_FAST;
  //this->dataPtr->videoStream->codec->coder_type = AVMEDIA_TYPE_VIDEO;
  this->dataPtr->videoStream->codec->width = this->dataPtr->outWidth;
  this->dataPtr->videoStream->codec->height = this->dataPtr->outHeight;
  this->dataPtr->videoStream->codec->pix_fmt = PIX_FMT_YUV420P;

  // put sample parameters
  //this->dataPtr->videoStream->codec->bit_rate = this->dataPtr->bitRate;
  this->dataPtr->videoStream->codec->rc_max_rate = this->dataPtr->bitRate;
  this->dataPtr->videoStream->codec->rc_min_rate = this->dataPtr->bitRate;


  // frames per second
  // DEPRECATED this->dataPtr->videoStream->codec->time_base.den = this->dataPtr->fps;
  // DEPRECATED this->dataPtr->videoStream->codec->time_base.num= 1;

  // this removes VBV buffer size not set warning msg
  this->dataPtr->videoStream->codec->rc_initial_buffer_occupancy = this->dataPtr->bitRate;
  //this->dataPtr->videoStream->codec->rc_max_rate = this->dataPtr->bitRate;
  //this->dataPtr->videoStream->codec->rc_buffer_size = this->dataPtr->bitRate;
  //this->dataPtr->videoStream->codec->thread_count = 5;

  if (this->dataPtr->videoStream->codec->codec_id == CODEC_ID_MPEG1VIDEO)
  {
    // Needed to avoid using macroblocks in which some coeffs overflow.
    // This does not happen with normal video, it just happens here as
    // the motion of the chroma plane does not match the luma plane.
    this->dataPtr->videoStream->codec->mb_decision = 2;
  }

  //std::cout << "Codec[" << codec->name << "] Id[" << codec->id << "]\n";
  if (this->dataPtr->videoStream->codec->codec_id == AV_CODEC_ID_H264)
  {
    av_opt_set(this->dataPtr->videoStream->codec->priv_data,
        "preset", "slow", 0);
  }

  // open it
  if (avcodec_open2(this->dataPtr->videoStream->codec, encoder, nullptr) < 0)
  {
    gzerr << "Could not open codec\n";
    this->Reset();
    return false;
  }

  this->dataPtr->fileHandle = fopen(tmpFileNameFull.c_str(), "wb");
  if (!this->dataPtr->fileHandle)
  {
    gzerr << "Could not open '" << tmpFileNameFull << "' for encoding\n";
    this->Reset();
    return false;
  }

  // DEPRECATED: this->dataPtr->avOutFrame = avcodec_alloc_frame();
  this->dataPtr->avOutFrame = av_frame_alloc();
  int size = avpicture_get_size(
      this->dataPtr->videoStream->codec->pix_fmt,
      this->dataPtr->videoStream->codec->width,
      this->dataPtr->videoStream->codec->height);

  //this->dataPtr->pictureBuf = new unsigned char[size];
  this->dataPtr->pictureBuf = (uint8_t*)av_malloc(size);

  std::cout << "AV OUt1[" << this->dataPtr->avOutFrame->linesize[0] << "]\n";
  //avpicture_fill(reinterpret_cast<AVPicture *>(this->dataPtr->avOutFrame),
  //    this->dataPtr->pictureBuf, this->dataPtr->videoStream->codec->pix_fmt,
  //    this->dataPtr->videoStream->codec->width, this->dataPtr->videoStream->codec->height);
  //
  av_image_fill_arrays(this->dataPtr->avOutFrame->data,
      this->dataPtr->avOutFrame->linesize,
      this->dataPtr->pictureBuf, this->dataPtr->videoStream->codec->pix_fmt,
      this->dataPtr->videoStream->codec->width, this->dataPtr->videoStream->codec->height, 1);
  std::cout << "AV OUt2[" << this->dataPtr->avOutFrame->linesize[0] << "]\n";

  this->dataPtr->avOutFrame->format = this->dataPtr->videoStream->codec->pix_fmt;
  this->dataPtr->avOutFrame->width = this->dataPtr->videoStream->codec->width;
  this->dataPtr->avOutFrame->height = this->dataPtr->videoStream->codec->height;

  // setting mux preload and max delay avoids buffer underflow when writing to
  // mpeg format
  double muxMaxDelay = 0.7f;
  this->dataPtr->formatCtx->max_delay =
    static_cast<int>(muxMaxDelay * AV_TIME_BASE);

  if (!(this->dataPtr->outputFormat->flags & AVFMT_NOFILE))
  {
    if (avio_open(&this->dataPtr->formatCtx->pb, tmpFileNameFull.c_str(),
        AVIO_FLAG_WRITE) < 0)
    {
      gzerr << "Could not open '" << tmpFileNameFull << "'\n";
      this->Reset();
      return false;
    }
  }

  // Write the stream header, if any.
  if (avformat_write_header(this->dataPtr->formatCtx, nullptr) < 0)
  {
    gzerr << " Error occured when opening output file" << std::endl;
    this->Reset();
    return false;
  }
  */

  this->dataPtr->encoding = true;
  return true;
#else
  gzwarn << "Encoding capability not available! "
      << "Please install libavcodec, libavformat and libswscale dev packages."
      << std::endl;
  return false;
#endif
}

////////////////////////////////////////////////
bool VideoEncoder::IsEncoding()
{
  return this->dataPtr->encoding;
}

/////////////////////////////////////////////////
bool VideoEncoder::AddFrame(const unsigned char *_frame,
    const unsigned int _width,
    const unsigned int _height)
{

  std::cout << "Add Frame\n";
  return this->AddFrame(_frame, _width, _height, common::Time::GetWallTime());
      //std::chrono::steady_clock::now());
}

/////////////////////////////////////////////////
bool VideoEncoder::AddFrame(const unsigned char *_frame,
    const unsigned int _width,
    const unsigned int _height,
    const common::Time &_timestamp)
{
  std::cout << "************************************************\n";
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->dataPtr->encoding)
  {
    gzerr << "Start encoding before adding a frame\n";
    return false;
  }

  common::Time dt = (_timestamp - this->dataPtr->timePrev2);
  // std::cout << "DT[" << dt << "]\n";
  if (dt.Double() < 1.0/(this->dataPtr->fps))
    return false;

  // std::cout << "VideoPts[" << this->dataPtr->videoPts << "]\n";
  //int pts = 0;
  int pts = this->dataPtr->frameCount++;
  /*if (this->dataPtr->videoPts != -1)
  {
    //pts = this->dataPtr->fps * this->dataPtr->totalTime2.Double();
    //pts = (1.0 / this->dataPtr->fps) * 90 * this->dataPtr->totalTime2.Double();
    this->dataPtr->totalTime2 += dt;
    // std::cout << "TotalTime[" << this->dataPtr->totalTime2 << "]\n";

    if (this->dataPtr->videoPts == pts)
      return false;
  }*/

  this->dataPtr->timePrev2 = _timestamp;
  //this->dataPtr->videoPts = pts;

  std::cout << "1\n";
  // Cause the sws to be recreated on image resize
  if (this->dataPtr->swsCtx &&
      (this->dataPtr->inWidth != _width || this->dataPtr->inHeight != _height))
  {
    sws_freeContext(this->dataPtr->swsCtx);
    this->dataPtr->swsCtx = nullptr;
    if (this->dataPtr->avInPicture)
    {
      av_free(this->dataPtr->avInPicture);
      this->dataPtr->avInPicture = nullptr;
    }
  }

  if (!this->dataPtr->swsCtx)
  {
    this->dataPtr->inWidth = _width;
    this->dataPtr->inHeight = _height;

    if (!this->dataPtr->avInPicture)
    {
      this->dataPtr->avInPicture = new AVPicture;
      avpicture_alloc(this->dataPtr->avInPicture,
          PIX_FMT_RGB24, this->dataPtr->inWidth,
          this->dataPtr->inHeight);
    }

    this->dataPtr->swsCtx = sws_getContext(
        this->dataPtr->inWidth,
        this->dataPtr->inHeight,
        PIX_FMT_RGB24,
        this->dataPtr->videoStream->codec->width,
        this->dataPtr->videoStream->codec->height,
        this->dataPtr->videoStream->codec->pix_fmt,
        SWS_BICUBIC, nullptr, nullptr, nullptr);

    if (this->dataPtr->swsCtx == nullptr)
    {
      gzerr << "Error while calling sws_getContext\n";
      return false;
    }
  }

  std::cout << "2\n";
  // encode
  memcpy(this->dataPtr->avInPicture->data[0], _frame,
         this->dataPtr->inWidth * this->dataPtr->inHeight * 3);

  std::cout << "Width x Height[" << this->dataPtr->avOutFrame->width << " " <<
    this->dataPtr->avOutFrame->height << "]\n";

  std::cout << "In line["
    << this->dataPtr->avInPicture->linesize[0] << " "
    << this->dataPtr->avInPicture->linesize[1] << " "
    << this->dataPtr->avInPicture->linesize[2]
    << "] Out line["
    << this->dataPtr->avOutFrame->linesize[0] << " "
    << this->dataPtr->avOutFrame->linesize[1] << " "
    << this->dataPtr->avOutFrame->linesize[2]
    << "]\n";

  sws_scale(this->dataPtr->swsCtx,
      this->dataPtr->avInPicture->data,
      this->dataPtr->avInPicture->linesize,
      0, this->dataPtr->inHeight,
      this->dataPtr->avOutFrame->data,
      this->dataPtr->avOutFrame->linesize);

  std::cout << "3\n";
  this->dataPtr->avOutFrame->pts = pts;
  int gotOutput = 0;
  AVPacket avPacket;
  av_init_packet(&avPacket);
  avPacket.data = nullptr;
  avPacket.size = 0;

  // std::cout << "Encode video. pts=" << this->dataPtr->videoStream->codec->coded_frame->pts << "\n";
  int ret = avcodec_encode_video2(this->dataPtr->videoStream->codec, &avPacket,
                                  this->dataPtr->avOutFrame, &gotOutput);

  if (ret >= 0)
  {
    // write the frame
    if (gotOutput)
    {
      if (this->dataPtr->videoStream->codec->coded_frame->key_frame)
        avPacket.flags |= AV_PKT_FLAG_KEY;

      std::cout << "Codec["
        << this->dataPtr->videoStream->codec->time_base.num << " "
        << this->dataPtr->videoStream->codec->time_base.den << "]"
        << " Stream["
        << this->dataPtr->videoStream->time_base.num << " "
        << this->dataPtr->videoStream->time_base.den << "]\n";

      avPacket.stream_index = this->dataPtr->videoStream->index;
      if (avPacket.pts != AV_NOPTS_VALUE)
      {
        avPacket.pts = av_rescale_q(avPacket.pts,
            this->dataPtr->videoStream->codec->time_base,
            this->dataPtr->videoStream->time_base);
      }
      if (avPacket.dts != AV_NOPTS_VALUE)
      {
        avPacket.dts = av_rescale_q(
            avPacket.dts,
            this->dataPtr->videoStream->codec->time_base,
            this->dataPtr->videoStream->time_base);
      }

      //fwrite(avPacket.data, 1, avPacket.size, this->dataPtr->fileHandle);

  std::cout << "4\n";
      ret = av_interleaved_write_frame(this->dataPtr->formatCtx, &avPacket);

      if (ret < 0)
      {
        gzerr << "Error writing frame" << std::endl;
        return false;
      }
    }
    else
      gzerr << "Not got output\n";
  }
  else
  {
    gzerr << "Error encoding frame[" << ret << "]\n";
    return false;
  }

  std::cout << "5\n";
  av_free_packet(&avPacket);
  return true;
}

/////////////////////////////////////////////////
#ifdef HAVE_FFMPEG
bool VideoEncoder::AddFrame(const unsigned char *_frame,
    const unsigned int _width,
    const unsigned int _height,
    const std::chrono::steady_clock::time_point &_timestamp)
{
  if (!this->dataPtr->encoding)
  {
    gzerr << "Start encoding before adding a frame\n";
    return false;
  }

  auto dt = _timestamp - this->dataPtr->timePrev;
  if (dt < std::chrono::duration<double>(1.0/this->dataPtr->sampleRate))
  {
    gzwarn << "Time delta["
      << std::chrono::duration_cast<std::chrono::milliseconds>(dt).count()
      << " ms] lower than Hz rate[" << 1.0/this->dataPtr->sampleRate << "]\n";
    return false;
  }

  this->dataPtr->timePrev = _timestamp;

  int pts = 0;
  if (this->dataPtr->videoPts != -1)
  {
    //pts = (1.0 / this->dataPtr->fps) * 90 * this->dataPtr->totalTime.count();
    pts = (1.0 / this->dataPtr->fps) * 9000 * (this->dataPtr->frameCount++);
    this->dataPtr->totalTime += dt;

    if (this->dataPtr->videoPts == pts)
    {
      gzwarn << "Video presentation timestamp matches current timestamp\n";
      return false;
    }
  }

  this->dataPtr->videoPts = pts;
  std::cout << "PTS[" << this->dataPtr->videoPts << "]\n";

  // Cause the sws to be recreated on image resize
  if (this->dataPtr->swsCtx &&
      (this->dataPtr->inWidth != _width || this->dataPtr->inHeight != _height))
  {
    sws_freeContext(this->dataPtr->swsCtx);
    this->dataPtr->swsCtx = nullptr;
    if (this->dataPtr->avInPicture)
    {
      av_free(this->dataPtr->avInPicture);
      this->dataPtr->avInPicture = nullptr;
    }
  }

  if (!this->dataPtr->swsCtx)
  {
    if (!this->dataPtr->avInPicture)
    {
      this->dataPtr->avInPicture = new AVPicture;
      avpicture_alloc(this->dataPtr->avInPicture,
          PIX_FMT_RGB24, this->dataPtr->inWidth, this->dataPtr->inHeight);
    }

    this->dataPtr->swsCtx = sws_getContext(this->dataPtr->inWidth,
        this->dataPtr->inHeight, PIX_FMT_RGB24,
        this->dataPtr->videoStream->codec->width, this->dataPtr->videoStream->codec->height,
        this->dataPtr->videoStream->codec->pix_fmt,
        SWS_BICUBIC, nullptr, nullptr, nullptr);

    if (this->dataPtr->swsCtx == nullptr)
    {
      gzerr << "Error while calling sws_getContext\n";
      return false;
    }
  }

  // encode
  memcpy(this->dataPtr->avInPicture->data[0], _frame,
      this->dataPtr->inWidth * this->dataPtr->inHeight * 3);

  /*sws_scale(this->dataPtr->swsCtx, this->dataPtr->avInPicture->data,
      this->dataPtr->avInPicture->linesize,
      0, this->dataPtr->inHeight, this->dataPtr->avOutFrame->data,
      this->dataPtr->avOutFrame->linesize);
      **/

  int gotOutput = 0;
  AVPacket avPacket;
  av_init_packet(&avPacket);
  avPacket.data = nullptr;
  avPacket.size = 0;

  int ret = avcodec_encode_video2(this->dataPtr->videoStream->codec, &avPacket,
                                  this->dataPtr->avOutFrame, &gotOutput);

  if (ret >= 0)
  {
    // write the frame
    if (gotOutput)
    {
      ret = av_interleaved_write_frame(this->dataPtr->formatCtx, &avPacket);

      if (ret < 0)
      {
        gzerr << "Error writing frame" << std::endl;
        return false;
      }
    }
  }
  else
  {
    gzerr << "Error encoding frame[" << ret << "]\n";
    return false;
  }

  av_free_packet(&avPacket);
  return true;
}
#else
bool VideoEncoder::AddFrame(const unsigned char */*_frame*/,
    const unsigned int /*_width*/,
    const unsigned int /*_height*/,
    const std::chrono::steady_clock::time_point &_timestamp)
{
  return false;
}
#endif

/////////////////////////////////////////////////
bool VideoEncoder::Stop()
{
  std::cout << "Stop\n";
#ifdef HAVE_FFMPEG
  if (this->dataPtr->encoding && this->dataPtr->formatCtx)
    av_write_trailer(this->dataPtr->formatCtx);
  this->dataPtr->formatCtx = nullptr;
  this->dataPtr->encoding = false;

  // get the delayed frames
  /*for (int got_output = 1; got_output; i++)
  {
    ret = avcodec_encode_video2(this->dataPtr->ctx, &pkt, nullptr, &got_output);

    if (ret < 0) {
      fprintf(stderr, "Error encoding frame\n");
      exit(1);
    }

    if (got_output) {
      fwrite(pkt.data, 1, pkt.size, f);
      av_free_packet(&pkt);
    }
  }*/
  return true;
#endif
  return false;
}

/////////////////////////////////////////////////
bool VideoEncoder::SaveToFile(const std::string &_filename)
{
  std::cout << "Save to file\n";
#ifdef HAVE_FFMPEG
  this->Stop();

  bool result = common::moveFile(this->dataPtr->TmpFilename(), _filename);

  if (!result)
  {
    gzerr << "Unable to rename file from[" << this->dataPtr->TmpFilename()
      << "] to [" << _filename << "]\n";
  }

  this->Reset();

  return result;
#else
  gzwarn << _filename << " not saved" << std::endl;
  return false;
#endif
}

/////////////////////////////////////////////////
void VideoEncoder::Reset()
{
  std::cout << "Reset\n";
  this->Stop();

#ifdef HAVE_FFMPEG
  if (this->dataPtr->formatCtx)
  {
    for (unsigned int i = 0; i < this->dataPtr->formatCtx->nb_streams; ++i)
    {
      avcodec_close(this->dataPtr->formatCtx->streams[i]->codec);
      av_freep(&this->dataPtr->formatCtx->streams[i]->codec);
      av_freep(&this->dataPtr->formatCtx->streams[i]);
    }
    av_free(this->dataPtr->formatCtx);
    this->dataPtr->formatCtx = nullptr;
  }

  if (this->dataPtr->avOutFrame)
  {
    av_free(this->dataPtr->avOutFrame);
    this->dataPtr->avOutFrame = nullptr;
  }

  if (this->dataPtr->avInPicture)
  {
    av_free(this->dataPtr->avInPicture);
    this->dataPtr->avInPicture = nullptr;
  }

  if (this->dataPtr->swsCtx)
    sws_freeContext(this->dataPtr->swsCtx);
#endif

  // Remove the tmp video file, if it exists
  std::string tmpFileNameFull = this->dataPtr->TmpFilename();

  if (common::isFile(tmpFileNameFull))
  {
    if (remove(tmpFileNameFull.c_str()) != 0)
      gzerr << "Unable to remove file[" << tmpFileNameFull << "].\n";
  }

  if (this->dataPtr->pictureBuf)
  {
    delete [] this->dataPtr->pictureBuf;
    this->dataPtr->pictureBuf = nullptr;
  }

  // set default values
  this->dataPtr->frameCount = 0;
  this->dataPtr->bitRate = VIDEO_ENCODER_BITRATE_DEFAULT;
  this->dataPtr->outWidth = VIDEO_ENCODER_WIDTH_DEFAULT;
  this->dataPtr->outHeight = VIDEO_ENCODER_HEIGHT_DEFAULT;
  this->dataPtr->inWidth = 0;
  this->dataPtr->inHeight = 0;
  this->dataPtr->fps = VIDEO_ENCODER_FPS_DEFAULT;
  this->dataPtr->format = VIDEO_ENCODER_FORMAT_DEFAULT;
  this->dataPtr->sampleRate = VIDEO_ENCODER_FPS_DEFAULT * 2;
  this->dataPtr->tmpFilename = "TMP_RECORDING";
  this->dataPtr->timePrev = {};
  this->dataPtr->totalTime = std::chrono::duration<double>(0);
  this->dataPtr->videoPts = -1;

  this->dataPtr->pictureBuf = nullptr;
  this->dataPtr->videoStream->codec = nullptr;
  this->dataPtr->swsCtx = nullptr;
  this->dataPtr->avInPicture = nullptr;
  this->dataPtr->avOutFrame = nullptr;
  this->dataPtr->formatCtx = nullptr;
}
