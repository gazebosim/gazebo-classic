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
#include <stdio.h>
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
#include <libavutil/error.h>
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
  /// \return Full filename of the temporary video file.
  public: std::string TmpFilename() const;

  /// \brief Helper function that opens the video codec
  /// \param[in] _codec Pointer to the codec
  public: bool OpenVideo(AVCodec *_codec);

  /// \brief Helper function that creates a video stream
  /// \param[in] _codec Pointer to the codec's address
  /// \param[in] _width Output frame width
  /// \param[in] _height Output frame height
  public: bool AddStream(AVCodec **_codec,
                         unsigned int _width, unsigned int _height);

  /// \brief libav audio video stream
  public: AVStream *videoStream = nullptr;

  /// \brief libav codec  context
  public: AVCodecContext *codecCtx = nullptr;

  /// \brief libav format I/O context
  public: AVFormatContext *formatCtx = nullptr;

  /// \brief libav output video frame
  public: AVFrame *avOutFrame = nullptr;

  /// \brief libav input image data
  public: AVFrame *avInFrame = nullptr;

  /// \brief Software scaling context
  public: SwsContext *swsCtx = nullptr;

  /// \brief True if the encoder is running
  public: bool encoding = false;

  /// \brief Video encoding bit rate
  public: unsigned int bitRate = VIDEO_ENCODER_BITRATE_DEFAULT;

  /// \brief Input frame width
  public: unsigned int inWidth = 0;

  /// \brief Input frame height
  public: unsigned int inHeight = 0;

  /// \brief Encoding format
  public: std::string format = VIDEO_ENCODER_FORMAT_DEFAULT;

  /// \brief Target framerate.
  public: unsigned int fps = VIDEO_ENCODER_FPS_DEFAULT;

  /// \brief Previous time when the frame is added.
  public: std::chrono::steady_clock::time_point timePrev;

  /// \brief Number of frames in the video
  public: uint64_t frameCount = 0;

  /// \brief Mutex for thread safety.
  public: std::mutex mutex;
};

/////////////////////////////////////////////////
VideoEncoder::VideoEncoder()
: dataPtr(new VideoEncoderPrivate)
{
  // Make sure libav is loaded.
  common::load();
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
unsigned int VideoEncoder::BitRate() const
{
  return this->dataPtr->bitRate;
}

/////////////////////////////////////////////////
bool VideoEncoder::Start(const unsigned int _width,
                         const unsigned int _height,
                         const std::string &_format,
                         const unsigned int _fps,
                         const unsigned int _bitRate)
{
  // Do not allow Start to be called more than once without Stop or Reset
  // being called first.
  if (this->dataPtr->encoding)
    return false;

  // This will be true if Stop has been called, but not reset. We will reset
  // automatically to prevent any errors.
  if (this->dataPtr->formatCtx || this->dataPtr->avInFrame ||
      this->dataPtr->avOutFrame || this->dataPtr->swsCtx)
  {
    this->Reset();
  }

  // Remove old temp file, if it exists.
  if (common::exists(this->dataPtr->TmpFilename()))
    std::remove(this->dataPtr->TmpFilename().c_str());

  // Calculate a good bitrate if the _bitRate argument is zero
  if (_bitRate == 0)
  {
    // 240p
    if (_width * _height <= 424*240)
      this->dataPtr->bitRate = 100000;
    // 360p
    else if (_width * _height <= 640*360)
      this->dataPtr->bitRate = 230000;
    // 432p
    else if (_width * _height <= 768*432)
      this->dataPtr->bitRate = 330000;
    // 480p (SD or NTSC widescreen)
    else if (_width * _height <= 848*480)
      this->dataPtr->bitRate = 410000;
    // 576p (PAL widescreen)
    else if (_width * _height <= 1024*576)
      this->dataPtr->bitRate = 590000;
    // 720p (HD)
    else if (_width * _height <= 1280*720)
      this->dataPtr->bitRate = 920000;
    // >720P(Full HD)
    else
      this->dataPtr->bitRate = 2070000;
  }
  else
  {
    this->dataPtr->bitRate = _bitRate;
  }

  // Store some info and reset the frame count.
  this->dataPtr->format = _format;
  this->dataPtr->fps = _fps;
  this->dataPtr->frameCount = 0;

#ifdef HAVE_FFMPEG
  AVCodec *videoCodec = nullptr;

  // The resolution must be divisible by two
  unsigned int outWidth = _width % 2 == 0 ? _width : _width + 1;
  unsigned int outHeight = _height % 2 == 0 ? _height : _height + 1;

  std::string tmpFileNameFull = this->dataPtr->TmpFilename();

  avformat_alloc_output_context2(&this->dataPtr->formatCtx, nullptr, nullptr,
                                 tmpFileNameFull.c_str());
  if (!this->dataPtr->formatCtx)
  {
    gzerr << "Unable to allocate format context. Video encoding not started\n";
    this->Reset();
    return false;
  }

  // Create the video stream
  if (!this->dataPtr->AddStream(&videoCodec, outWidth, outHeight))
  {
    this->Reset();
    return false;
  }

  // Opene the video stream
  if (!this->dataPtr->OpenVideo(videoCodec))
  {
    this->Reset();
    return false;
  }

  // setting mux preload and max delay avoids buffer underflow when writing to
  // mpeg format
  double muxMaxDelay = 0.7f;
  this->dataPtr->formatCtx->max_delay =
    static_cast<int>(muxMaxDelay * AV_TIME_BASE);

  if (!(this->dataPtr->formatCtx->oformat->flags & AVFMT_NOFILE))
  {
    int ret = avio_open(&this->dataPtr->formatCtx->pb, tmpFileNameFull.c_str(),
        AVIO_FLAG_WRITE);
    if (ret < 0)
    {
      char errBuff[AV_ERROR_MAX_STRING_SIZE];
      av_strerror(ret, errBuff, AV_ERROR_MAX_STRING_SIZE);
      gzerr << "Could not open '" << tmpFileNameFull << "'. "
            << errBuff
            << "Video encoding is not started\n";
      this->Reset();
      return false;
    }
  }

  // Write the stream header, if any.
  int ret = avformat_write_header(this->dataPtr->formatCtx, nullptr);
  if (ret < 0)
  {
    char errBuff[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(ret, errBuff, AV_ERROR_MAX_STRING_SIZE);

    gzerr << "Error occured when opening output file. "
          << errBuff
          << "Video encoding is not started\n";
    this->Reset();
    return false;
  }

  this->dataPtr->encoding = true;

  return true;
#else
  gzwarn << "Encoding capability not available. "
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
  return this->AddFrame(_frame, _width, _height,
                        std::chrono::steady_clock::now());
}

/////////////////////////////////////////////////
#ifdef HAVE_FFMPEG
bool VideoEncoder::AddFrame(const unsigned char *_frame,
    const unsigned int _width,
    const unsigned int _height,
    const std::chrono::steady_clock::time_point &_timestamp)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->dataPtr->encoding)
  {
    gzerr << "Start encoding before adding a frame\n";
    return false;
  }

  auto dt = _timestamp - this->dataPtr->timePrev;

  // Skip frames that arrive faster than the video's fps
  if (dt < std::chrono::duration<double>(1.0/this->dataPtr->fps))
    return false;

  this->dataPtr->timePrev = _timestamp;

  // Cause the sws to be recreated on image resize
  if (this->dataPtr->swsCtx &&
      (this->dataPtr->inWidth != _width || this->dataPtr->inHeight != _height))
  {
    sws_freeContext(this->dataPtr->swsCtx);
    this->dataPtr->swsCtx = nullptr;

    if (this->dataPtr->avInFrame)
      av_frame_free(&this->dataPtr->avInFrame);
    this->dataPtr->avInFrame = nullptr;
  }

  if (!this->dataPtr->swsCtx)
  {
    this->dataPtr->inWidth = _width;
    this->dataPtr->inHeight = _height;

    if (!this->dataPtr->avInFrame)
    {
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55, 28, 1)
      this->dataPtr->avInFrame = avcodec_alloc_frame();
#else
      this->dataPtr->avInFrame = av_frame_alloc();
#endif

      av_image_alloc(this->dataPtr->avInFrame->data,
          this->dataPtr->avInFrame->linesize,
          this->dataPtr->inWidth, this->dataPtr->inHeight,
          AV_PIX_FMT_RGB24, 1);
    }

    this->dataPtr->swsCtx = sws_getContext(
        this->dataPtr->inWidth,
        this->dataPtr->inHeight,
        AV_PIX_FMT_RGB24,
        this->dataPtr->codecCtx->width,
        this->dataPtr->codecCtx->height,
        this->dataPtr->codecCtx->pix_fmt,
        SWS_BICUBIC, nullptr, nullptr, nullptr);

    if (this->dataPtr->swsCtx == nullptr)
    {
      gzerr << "Error while calling sws_getContext\n";
      return false;
    }
  }

  // encode
  memcpy(this->dataPtr->avInFrame->data[0], _frame,
         this->dataPtr->inWidth * this->dataPtr->inHeight * 3);

  sws_scale(this->dataPtr->swsCtx,
            this->dataPtr->avInFrame->data,
            this->dataPtr->avInFrame->linesize,
            0, this->dataPtr->inHeight,
            this->dataPtr->avOutFrame->data,
            this->dataPtr->avOutFrame->linesize);

  this->dataPtr->avOutFrame->pts = this->dataPtr->frameCount++;

  AVPacket *avPacket = av_packet_alloc();
  av_init_packet(avPacket);
  avPacket->data = nullptr;
  avPacket->size = 0;

  int ret = avcodec_send_frame(this->dataPtr->codecCtx,
                               this->dataPtr->avOutFrame);

  // This loop will retrieve and write available packets
  while (ret >= 0)
  {
    ret = avcodec_receive_packet(this->dataPtr->codecCtx, avPacket);

    // Potential performance improvement: Queue the packets and write in
    // a separate thread.
    if (ret >= 0)
    {
      avPacket->stream_index = this->dataPtr->videoStream->index;

      // Scale timestamp appropriately.
      if (avPacket->pts != static_cast<int64_t>(AV_NOPTS_VALUE))
      {
        avPacket->pts = av_rescale_q(avPacket->pts,
            this->dataPtr->codecCtx->time_base,
            this->dataPtr->videoStream->time_base);
      }

      if (avPacket->dts != static_cast<int64_t>(AV_NOPTS_VALUE))
      {
        avPacket->dts = av_rescale_q(
            avPacket->dts,
            this->dataPtr->codecCtx->time_base,
            this->dataPtr->videoStream->time_base);
      }

      // Write frame to disk
      if (av_interleaved_write_frame(this->dataPtr->formatCtx, avPacket) < 0)
        gzerr << "Error writing frame" << std::endl;
    }
  }

  av_packet_free(&avPacket);
  return true;
}
#else
bool VideoEncoder::AddFrame(const unsigned char */*_frame*/,
    const unsigned int /*_width*/,
    const unsigned int /*_height*/,
    const std::chrono::steady_clock::time_point & /*_timestamp*/)
{
  return false;
}
#endif

/////////////////////////////////////////////////
bool VideoEncoder::Stop()
{
#ifdef HAVE_FFMPEG
  if (this->dataPtr->encoding && this->dataPtr->formatCtx)
    av_write_trailer(this->dataPtr->formatCtx);

  if (this->dataPtr->codecCtx)
    avcodec_free_context(&this->dataPtr->codecCtx);
  this->dataPtr->codecCtx = nullptr;

  if (this->dataPtr->avInFrame)
    av_frame_free(&this->dataPtr->avInFrame);
  this->dataPtr->avInFrame = nullptr;

  if (this->dataPtr->avOutFrame)
    av_frame_free(&this->dataPtr->avOutFrame);
  this->dataPtr->avOutFrame = nullptr;

  if (this->dataPtr->swsCtx)
    sws_freeContext(this->dataPtr->swsCtx);
  this->dataPtr->swsCtx = nullptr;

  // This frees the context and all the streams
  if (this->dataPtr->formatCtx)
    avformat_free_context(this->dataPtr->formatCtx);
  this->dataPtr->formatCtx = nullptr;
  this->dataPtr->videoStream = nullptr;

  this->dataPtr->encoding = false;
  return true;
#endif
  this->dataPtr->encoding = false;
  return false;
}

/////////////////////////////////////////////////
bool VideoEncoder::SaveToFile(const std::string &_filename)
{
  // First stop the recording
  this->Stop();

#ifdef HAVE_FFMPEG

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
  // Make sure the video has been stopped.
  this->Stop();

  // Remove old temp file, if it exists.
  if (common::exists(this->dataPtr->TmpFilename()))
    std::remove(this->dataPtr->TmpFilename().c_str());

  // set default values
  this->dataPtr->frameCount = 0;
  this->dataPtr->inWidth = 0;
  this->dataPtr->inHeight = 0;
  this->dataPtr->timePrev = {};
  this->dataPtr->bitRate = VIDEO_ENCODER_BITRATE_DEFAULT;
  this->dataPtr->fps = VIDEO_ENCODER_FPS_DEFAULT;
  this->dataPtr->format = VIDEO_ENCODER_FORMAT_DEFAULT;
}

/////////////////////////////////////////////////
std::string VideoEncoderPrivate::TmpFilename() const
{
  return common::cwd() + "/TMP_RECORDING." + this->format;
}

/////////////////////////////////////////////////
bool VideoEncoderPrivate::AddStream(AVCodec **_codec,
    unsigned int _width, unsigned int _height)
{
  // find the video encoder
  *_codec = avcodec_find_encoder(this->formatCtx->oformat->video_codec);
  if (!(*_codec))
  {
    gzerr << "Codec for["
      << avcodec_get_name(this->formatCtx->oformat->video_codec)
      << "] not found. Video encoding is not started.\n";
    return false;
  }

  // Create a new video stream
  this->videoStream = avformat_new_stream(this->formatCtx, nullptr);
  if (!this->videoStream)
  {
    gzerr << "Could not allocate stream. Video encoding is not started\n";
    return false;
  }
  this->videoStream->id = this->formatCtx->nb_streams-1;

  // Allocate a new video context
  this->codecCtx = avcodec_alloc_context3(*_codec);
  if (!this->codecCtx)
  {
    gzerr << "Could not allocate an encoding context."
      << "Video encoding is not started\n";
    return false;
  }

  // Set the context parameters
  this->codecCtx->codec_id = this->formatCtx->oformat->video_codec;
  this->codecCtx->bit_rate = this->bitRate;
  this->codecCtx->width = _width;
  this->codecCtx->height = _height;
  this->codecCtx->time_base.den = this->fps;
  this->codecCtx->time_base.num = 1;
  this->codecCtx->gop_size = 10;
  this->codecCtx->pix_fmt = AV_PIX_FMT_YUV420P;

  // The video stream must have the same time base as the context
  this->videoStream->time_base.den = this->fps;
  this->videoStream->time_base.num = 1;

  if (this->codecCtx->codec_id == AV_CODEC_ID_MPEG2VIDEO)
    this->codecCtx->max_b_frames = 2;

  if (this->codecCtx->codec_id == AV_CODEC_ID_MPEG1VIDEO)
    this->codecCtx->mb_decision = 2;

  if (this->codecCtx->codec_id == AV_CODEC_ID_H264)
  {
    av_opt_set(this->videoStream->priv_data, "preset", "slow", 0);
    av_opt_set(this->codecCtx->priv_data, "preset", "slow", 0);
  }

  if (this->formatCtx->oformat->flags & AVFMT_GLOBALHEADER)
    this->codecCtx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

  return true;
}

/////////////////////////////////////////////////
bool VideoEncoderPrivate::OpenVideo(AVCodec *_codec)
{
  int ret = 0;

  // Open the video context
  ret = avcodec_open2(this->codecCtx, _codec, 0);
  if (ret < 0)
  {
    char errBuff[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(ret, errBuff, AV_ERROR_MAX_STRING_SIZE);

    gzerr << "Could not open video codec: " << errBuff << std::endl;
    return false;
  }

  // Allocate the output frame
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55, 28, 1)
  this->avOutFrame = avcodec_alloc_frame();
#else
  this->avOutFrame = av_frame_alloc();
#endif

  if (!this->avOutFrame)
  {
    gzerr << "Cloud not allocate video frame\n";
    return false;
  }

  this->avOutFrame->format = this->codecCtx->pix_fmt;
  this->avOutFrame->width = this->codecCtx->width;
  this->avOutFrame->height = this->codecCtx->height;

  av_image_alloc(this->avOutFrame->data, this->avOutFrame->linesize,
                 this->codecCtx->width, this->codecCtx->height,
                 this->codecCtx->pix_fmt, 32);

  // Copy parameters from the context to the video stream
  ret = avcodec_parameters_from_context(
      this->videoStream->codecpar, this->codecCtx);

  if (ret < 0)
  {
    gzerr << "Could not copy the stream parameters\n";
    return false;
  }

  return true;
}
