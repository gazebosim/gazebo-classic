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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#ifndef _WIN32
  #include <sys/ioctl.h>
#endif
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

#if defined(__linux__) && defined(HAVE_AVDEVICE)
#include <libavdevice/avdevice.h>
#endif
}
#endif

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/VideoEncoder.hh"

using namespace gazebo;
using namespace common;

#ifndef AV_ERROR_MAX_STRING_SIZE
#define AV_ERROR_MAX_STRING_SIZE 64
#endif

// Private data class
class gazebo::common::VideoEncoderPrivate
{
  /// \brief Name of the file which stores the video while it is being
  ///        recorded.
  public: std::string filename;

#ifdef HAVE_FFMPEG
  /// \brief libav audio video stream
  public: AVStream *videoStream = nullptr;

  /// \brief libav codec  context
  public: AVCodecContext *codecCtx = nullptr;

  /// \brief libav format I/O context
  public: AVFormatContext *formatCtx = nullptr;

  /// \brief libav output video frame
  public: AVFrame *avOutFrame = nullptr;

  /// \brief libav input image data
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(57, 24, 1)
  public: AVPicture *avInFrame = nullptr;
#else
  public: AVFrame *avInFrame = nullptr;
#endif

  /// \brief Software scaling context
  public: SwsContext *swsCtx = nullptr;
#endif

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
#ifdef HAVE_FFMPEG
bool VideoEncoder::Start(const std::string &_format,
                         const std::string &_filename,
                         const unsigned int _width,
                         const unsigned int _height,
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
  if (common::exists(this->dataPtr->filename))
    std::remove(this->dataPtr->filename.c_str());

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
  this->dataPtr->format = _format.compare("v4l") == 0 ? "v4l2" : _format;
  this->dataPtr->fps = _fps;
  this->dataPtr->frameCount = 0;
  this->dataPtr->filename = _filename;

  // Create a default filenamae if the provided filename is empty.
  if (this->dataPtr->filename.empty())
  {
    if (this->dataPtr->format.compare("v4l2") == 0)
    {
      gzerr << "A video4linux loopback device filename must be specified on "
        << "Start\n";
      this->Reset();
      return false;
    }
    else
    {
      this->dataPtr->filename = common::cwd() + "/TMP_RECORDING." +
                                this->dataPtr->format;
    }
  }

  // The remainder of this function handles FFMPEG initialization of a video
  // stream
  AVOutputFormat *outputFormat = nullptr;

  // This 'if' and 'free' are just for safety. We chech the value of formatCtx
  // below.
  if (this->dataPtr->formatCtx)
    avformat_free_context(this->dataPtr->formatCtx);
  this->dataPtr->formatCtx = nullptr;

  // Special case for video4linux2. Here we attempt to find the v4l2 device
  if (this->dataPtr->format.compare("v4l2") == 0)
  {
#if LIBAVDEVICE_VERSION_INT >= AV_VERSION_INT(56, 4, 100)
    while ((outputFormat = av_output_video_device_next(outputFormat))
           != nullptr)
    {
      // Break when the output device name matches 'v4l2'
      if (this->dataPtr->format.compare(outputFormat->name) == 0)
      {
        // Allocate the context using the correct outputFormat
        avformat_alloc_output_context2(&this->dataPtr->formatCtx,
            outputFormat, nullptr, this->dataPtr->filename.c_str());
        break;
      }
    }
#else
    gzerr << "libavdevice version >= 56.4.100 is required for v4l2 recording. "
          << "This version is available on Ubuntu Xenial or greater.\n";
    return false;
#endif
  }
  else
  {
    outputFormat = av_guess_format(nullptr,
                                   this->dataPtr->filename.c_str(), nullptr);

    if (!outputFormat)
    {
      gzwarn << "Could not deduce output format from file extension."
        << "Using MPEG.\n";
      outputFormat = av_guess_format("mpeg", nullptr, nullptr);
    }

#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(56, 40, 1)
        this->dataPtr->formatCtx = avformat_alloc_context();
        this->dataPtr->formatCtx->oformat = outputFormat;
#ifdef WIN32
        _sprintf(this->dataPtr->formatCtx->filename,
                 sizeof(this->dataPtr->formatCtx->filename),
                 "%s", _filename.c_str());
#else
        snprintf(this->dataPtr->formatCtx->filename,
                sizeof(this->dataPtr->formatCtx->filename),
                "%s", _filename.c_str());
#endif

#else
    avformat_alloc_output_context2(&this->dataPtr->formatCtx, nullptr, nullptr,
        this->dataPtr->filename.c_str());
#endif
  }

  // Make sure allocation occurred.
  if (!this->dataPtr->formatCtx)
  {
    gzerr << "Unable to allocate format context. Video encoding not started\n";
    this->Reset();
    return false;
  }

  // find the video encoder
  AVCodec *encoder = avcodec_find_encoder(
      this->dataPtr->formatCtx->oformat->video_codec);
  if (!encoder)
  {
    gzerr << "Codec for["
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(57, 24, 1)
          << this->dataPtr->formatCtx->oformat->name
#else
          << avcodec_get_name(this->dataPtr->formatCtx->oformat->video_codec)
#endif
          << "] not found. Video encoding is not started.\n";
    this->Reset();
    return false;
  }

  // Create a new video stream
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(57, 24, 1)
  this->dataPtr->videoStream = avformat_new_stream(this->dataPtr->formatCtx,
    encoder);
#else
  this->dataPtr->videoStream = avformat_new_stream(this->dataPtr->formatCtx,
      nullptr);
#endif

  if (!this->dataPtr->videoStream)
  {
    gzerr << "Could not allocate stream. Video encoding is not started\n";
    this->Reset();
    return false;
  }
  this->dataPtr->videoStream->id = this->dataPtr->formatCtx->nb_streams-1;

  // Allocate a new video context
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(57, 24, 1)
  this->dataPtr->codecCtx = this->dataPtr->videoStream->codec;
#else
  this->dataPtr->codecCtx = avcodec_alloc_context3(encoder);
#endif

  if (!this->dataPtr->codecCtx)
  {
    gzerr << "Could not allocate an encoding context."
          << "Video encoding is not started\n";
    this->Reset();
    return false;
  }

  // some formats want stream headers to be separate
  if (this->dataPtr->formatCtx->oformat->flags & AVFMT_GLOBALHEADER)
  {
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(57, 24, 1)
    this->dataPtr->codecCtx->flags |= CODEC_FLAG_GLOBAL_HEADER;
#else
    this->dataPtr->codecCtx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
#endif
  }

  // Frames per second
  this->dataPtr->codecCtx->time_base.den = this->dataPtr->fps;
  this->dataPtr->codecCtx->time_base.num = 1;

  // The video stream must have the same time base as the context
  this->dataPtr->videoStream->time_base.den = this->dataPtr->fps;
  this->dataPtr->videoStream->time_base.num = 1;

  // Bitrate
  this->dataPtr->codecCtx->bit_rate = this->dataPtr->bitRate;

  // The resolution must be divisible by two
  this->dataPtr->codecCtx->width = _width % 2 == 0 ? _width : _width + 1;
  this->dataPtr->codecCtx->height = _height % 2 == 0 ? _height : _height + 1;

  // Emit one intra-frame every 10 frames
  this->dataPtr->codecCtx->gop_size = 10;
  this->dataPtr->codecCtx->max_b_frames = 1;
  this->dataPtr->codecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
  this->dataPtr->codecCtx->thread_count = 5;

  // Set the codec id
  this->dataPtr->codecCtx->codec_id =
    this->dataPtr->formatCtx->oformat->video_codec;

  if (this->dataPtr->codecCtx->codec_id == AV_CODEC_ID_MPEG1VIDEO)
  {
    // Needed to avoid using macroblocks in which some coeffs overflow.
    // This does not happen with normal video, it just happens here as
    // the motion of the chroma plane does not match the luma plane.
    this->dataPtr->codecCtx->mb_decision = 2;
  }

  if (this->dataPtr->codecCtx->codec_id == AV_CODEC_ID_H264)
  {
    av_opt_set(this->dataPtr->codecCtx->priv_data, "preset", "slow", 0);

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(57, 24, 1)
    av_opt_set(this->dataPtr->videoStream->codec->priv_data,
        "preset", "slow", 0);
#else
    av_opt_set(this->dataPtr->videoStream->priv_data, "preset", "slow", 0);
#endif
  }

  // Open the video context
  int ret = avcodec_open2(this->dataPtr->codecCtx, encoder, 0);
  if (ret < 0)
  {
    char errBuff[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(ret, errBuff, AV_ERROR_MAX_STRING_SIZE);

    gzerr << "Could not open video codec: " << errBuff
          << "Video encoding is not started\n";
    this->Reset();
    return false;
  }

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55, 28, 1)
  this->dataPtr->avOutFrame = avcodec_alloc_frame();
#else
  this->dataPtr->avOutFrame = av_frame_alloc();
#endif

  if (!this->dataPtr->avOutFrame)
  {
    gzerr << "Could not allocate video frame. Video encoding is not started\n";
    this->Reset();
    return false;
  }

  this->dataPtr->avOutFrame->format = this->dataPtr->codecCtx->pix_fmt;
  this->dataPtr->avOutFrame->width = this->dataPtr->codecCtx->width;
  this->dataPtr->avOutFrame->height = this->dataPtr->codecCtx->height;

  // the image can be allocated by any means and av_image_alloc() is
  // just the most convenient way if av_malloc() is to be used
  if (av_image_alloc(this->dataPtr->avOutFrame->data,
                     this->dataPtr->avOutFrame->linesize,
                     this->dataPtr->codecCtx->width,
                     this->dataPtr->codecCtx->height,
                     this->dataPtr->codecCtx->pix_fmt, 32) < 0)
  {
    gzerr << "Could not allocate raw picture buffer."
          << "Video encoding is not started\n";
    this->Reset();
    return false;
  }

  // Copy parameters from the context to the video stream
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(57, 24, 1)
//  ret = avcodec_copy_context(this->dataPtr->videoStream->codec,
//                       this->dataPtr->codecCtx);
#else
  ret = avcodec_parameters_from_context(
      this->dataPtr->videoStream->codecpar, this->dataPtr->codecCtx);
#endif
  if (ret < 0)
  {
    char errBuff[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(ret, errBuff, AV_ERROR_MAX_STRING_SIZE);

    gzerr << "Could not copy the stream parameters:" << errBuff
          << "Video encoding not started\n";
    return false;
  }

  // setting mux preload and max delay avoids buffer underflow when writing to
  // mpeg format
  double muxMaxDelay = 0.7f;
  this->dataPtr->formatCtx->max_delay =
    static_cast<int>(muxMaxDelay * AV_TIME_BASE);

  // Open the video stream
  if (!(this->dataPtr->formatCtx->oformat->flags & AVFMT_NOFILE))
  {
    ret = avio_open(&this->dataPtr->formatCtx->pb,
        this->dataPtr->filename.c_str(), AVIO_FLAG_WRITE);

    if (ret < 0)
    {
      char errBuff[AV_ERROR_MAX_STRING_SIZE];
      av_strerror(ret, errBuff, AV_ERROR_MAX_STRING_SIZE);
      gzerr << "Could not open '" << this->dataPtr->filename << "'. "
            << errBuff
            << "Video encoding is not started\n";
      this->Reset();
      return false;
    }
  }

  // Write the stream header, if any.
  ret = avformat_write_header(this->dataPtr->formatCtx, nullptr);
  if (ret < 0)
  {
    char errBuff[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(ret, errBuff, AV_ERROR_MAX_STRING_SIZE);

    gzerr << "Error occured when opening output file: " << errBuff
          << ". Video encoding is not started\n";
    this->Reset();
    return false;
  }

  this->dataPtr->encoding = true;
  return true;
}
// #else for HAVE_FFMPEG version check
#else
bool VideoEncoder::Start(const std::string &/*_format*/,
                         const std::string &/*_filename*/,
                         const unsigned int /*_width*/,
                         const unsigned int /*_height*/,
                         const unsigned int /*_fps*/,
                         const unsigned int /*_bitRate*/)
{
  gzwarn << "Encoding capability not available. "
      << "Please install libavcodec, libavformat and libswscale dev packages."
      << std::endl;
  return false;
}
#endif

////////////////////////////////////////////////
bool VideoEncoder::IsEncoding() const
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

#ifdef HAVE_FFMPEG
/////////////////////////////////////////////////
// This function supports ffmpeg2
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
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(57, 24, 1)
      av_free(this->dataPtr->avInFrame);
#else
      av_frame_free(&this->dataPtr->avInFrame);
#endif
    this->dataPtr->avInFrame = nullptr;
  }

  if (!this->dataPtr->swsCtx)
  {
    this->dataPtr->inWidth = _width;
    this->dataPtr->inHeight = _height;

    if (!this->dataPtr->avInFrame)
    {
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(57, 24, 1)
      this->dataPtr->avInFrame = new AVPicture;
      avpicture_alloc(this->dataPtr->avInFrame,
          AV_PIX_FMT_RGB24, this->dataPtr->inWidth,
          this->dataPtr->inHeight);
#else
      this->dataPtr->avInFrame = av_frame_alloc();

      av_image_alloc(this->dataPtr->avInFrame->data,
          this->dataPtr->avInFrame->linesize,
          this->dataPtr->inWidth, this->dataPtr->inHeight,
          AV_PIX_FMT_RGB24, 1);
#endif
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

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(57, 24, 1)
  int gotOutput = 0;
  AVPacket avPacket;
  av_init_packet(&avPacket);
  avPacket.data = nullptr;
  avPacket.size = 0;

  int ret = avcodec_encode_video2(this->dataPtr->codecCtx, &avPacket,
      this->dataPtr->avOutFrame, &gotOutput);

  if (ret >= 0 && gotOutput == 1)
  {
    avPacket.stream_index = this->dataPtr->videoStream->index;

    // Scale timestamp appropriately.
    if (avPacket.pts != static_cast<int64_t>(AV_NOPTS_VALUE))
    {
      avPacket.pts = av_rescale_q(avPacket.pts,
          this->dataPtr->codecCtx->time_base,
          this->dataPtr->videoStream->time_base);
    }

    if (avPacket.dts != static_cast<int64_t>(AV_NOPTS_VALUE))
    {
      avPacket.dts = av_rescale_q(
          avPacket.dts,
          this->dataPtr->codecCtx->time_base,
          this->dataPtr->videoStream->time_base);
    }

    // Write frame to disk
    ret = av_interleaved_write_frame(this->dataPtr->formatCtx, &avPacket);

    if (ret < 0)
    {
      gzerr << "Error writing frame" << std::endl;
      return false;
    }
  }

  av_free_packet(&avPacket);

// #else for libavcodec version check
#else

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
#endif
  return true;
}
// #else for HAVE_FFMPEG check
#else
bool VideoEncoder::AddFrame(const unsigned char */*_frame*/,
    const unsigned int /*_width*/,
    const unsigned int /*_height*/,
    const std::chrono::steady_clock::time_point & /*_timestamp*/)
{
  gzwarn << "Encoding capability not available. "
      << "Please install libavcodec, libavformat and libswscale dev packages."
      << std::endl;
  return false;
}
#endif

/////////////////////////////////////////////////
bool VideoEncoder::Stop()
{
#ifdef HAVE_FFMPEG
  if (this->dataPtr->encoding && this->dataPtr->formatCtx)
    av_write_trailer(this->dataPtr->formatCtx);

#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(57, 24, 1)
  if (this->dataPtr->codecCtx)
    avcodec_free_context(&this->dataPtr->codecCtx);
#endif
  this->dataPtr->codecCtx = nullptr;

  if (this->dataPtr->avInFrame)
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(57, 24, 1)
    av_free(this->dataPtr->avInFrame);
#else
    av_frame_free(&this->dataPtr->avInFrame);
#endif
  this->dataPtr->avInFrame = nullptr;

  if (this->dataPtr->avOutFrame)
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(57, 24, 1)
    av_free(this->dataPtr->avOutFrame);
#else
    av_frame_free(&this->dataPtr->avOutFrame);
#endif
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
  bool result = true;

  if (this->dataPtr->format != "v4l2")
  {
    result = common::moveFile(this->dataPtr->filename, _filename);

    if (!result)
    {
      gzerr << "Unable to rename file from[" << this->dataPtr->filename
        << "] to [" << _filename << "]\n";
    }
  }

  this->dataPtr->filename = "";

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
  if (common::exists(this->dataPtr->filename))
    std::remove(this->dataPtr->filename.c_str());

  // set default values
  this->dataPtr->frameCount = 0;
  this->dataPtr->inWidth = 0;
  this->dataPtr->inHeight = 0;
  this->dataPtr->timePrev = {};
  this->dataPtr->bitRate = VIDEO_ENCODER_BITRATE_DEFAULT;
  this->dataPtr->fps = VIDEO_ENCODER_FPS_DEFAULT;
  this->dataPtr->format = VIDEO_ENCODER_FORMAT_DEFAULT;
}
