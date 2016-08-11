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

  /// \brief True if the encoder is initialized
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
  // Do not all Start to be called more than once, without Stop or Reset
  // being called first.
  if (this->dataPtr->encoding)
    return false;

  // This will be true if Stop has been called, but not reset. We will reset
  // automatically to prevent any errors.
  if (this->dataPtr->formatCtx || this->dataPtr->avInPicture ||
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
  // The resolution must be divisible by two
  unsigned int outWidth = _width % 2 == 0 ? _width : _width + 1;
  unsigned int outHeight = _height % 2 == 0 ? _height : _height + 1;

  std::string tmpFileNameFull = this->dataPtr->TmpFilename();

  AVOutputFormat *outputFormat =
    av_guess_format(NULL, tmpFileNameFull.c_str(), NULL);

  if (!outputFormat)
  {
    gzerr << "Could not deduce output format from file extension: "
        << "using MPEG.\n";
    outputFormat = av_guess_format("mpeg", NULL, NULL);
  }

  AVCodec *encoder = nullptr;

  // find the video encoder
  encoder = avcodec_find_encoder(outputFormat->video_codec);
  if (!encoder)
  {
    gzerr << "Codec for[" << outputFormat->name
          << "] not found. Video encoding is not started.\n";
    this->Reset();
    return false;
  }

  this->dataPtr->formatCtx = avformat_alloc_context();
  this->dataPtr->formatCtx->oformat = outputFormat;

#ifdef WIN32
  _snprintf(this->dataPtr->formatCtx->filename,
      sizeof(this->dataPtr->formatCtx->filename),
      "%s", tmpFileNameFull.c_str());
#else
  snprintf(this->dataPtr->formatCtx->filename,
      sizeof(this->dataPtr->formatCtx->filename),
      "%s", tmpFileNameFull.c_str());
#endif

  this->dataPtr->videoStream =
    avformat_new_stream(this->dataPtr->formatCtx, encoder);

  // some formats want stream headers to be separate
  if (this->dataPtr->formatCtx->oformat->flags & AVFMT_GLOBALHEADER)
    this->dataPtr->videoStream->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;

  // Make sure the codec is valid
  if (!this->dataPtr->videoStream->codec)
  {
    gzerr << "Could not allocated video codex context."
          << "Video encoding is not started\n";
    this->Reset();
    return false;
  }

  // frames per second
  this->dataPtr->videoStream->time_base.den = this->dataPtr->fps;
  this->dataPtr->videoStream->time_base.num = 1;
  this->dataPtr->videoStream->codec->time_base.den = this->dataPtr->fps;
  this->dataPtr->videoStream->codec->time_base.num = 1;

  // Sample parameters
  this->dataPtr->videoStream->codec->bit_rate = this->dataPtr->bitRate;

  // Resolution
  this->dataPtr->videoStream->codec->width = outWidth;
  this->dataPtr->videoStream->codec->height = outHeight;

  // Emit one intra frame every ten frames
  this->dataPtr->videoStream->codec->gop_size = 10;
  this->dataPtr->videoStream->codec->max_b_frames = 1;
  this->dataPtr->videoStream->codec->pix_fmt = AV_PIX_FMT_YUV420P;
  this->dataPtr->videoStream->codec->thread_count = 5;

  if (this->dataPtr->videoStream->codec->codec_id == AV_CODEC_ID_H264)
  {
    av_opt_set(this->dataPtr->videoStream->codec->priv_data,
        "preset", "slow", 0);
  }

  if (this->dataPtr->videoStream->codec->codec_id == AV_CODEC_ID_MPEG1VIDEO)
  {
    // Needed to avoid using macroblocks in which some coeffs overflow.
    // This does not happen with normal video, it just happens here as
    // the motion of the chroma plane does not match the luma plane.
    this->dataPtr->videoStream->codec->mb_decision = 2;
  }

  // Open it
  if (avcodec_open2(this->dataPtr->videoStream->codec, encoder, nullptr) < 0)
  {
    gzerr << "Could not open codec."
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
    gzerr << "Could not allocate video frame."
      << "Video encoding is not started\n";
    this->Reset();
    return false;
  }

  this->dataPtr->avOutFrame->format =
    this->dataPtr->videoStream->codec->pix_fmt;
  this->dataPtr->avOutFrame->width  = this->dataPtr->videoStream->codec->width;
  this->dataPtr->avOutFrame->height = this->dataPtr->videoStream->codec->height;

  // the image can be allocated by any means and av_image_alloc() is
  // just the most convenient way if av_malloc() is to be used
  if (av_image_alloc(this->dataPtr->avOutFrame->data,
      this->dataPtr->avOutFrame->linesize,
      this->dataPtr->videoStream->codec->width,
      this->dataPtr->videoStream->codec->height,
      this->dataPtr->videoStream->codec->pix_fmt, 32) < 0)
  {
    gzerr << "Could not allocate raw picture buffer."
          << "Video encoding is not started\n";
    this->Reset();
    return false;
  }

  // setting mux preload and max delay avoids buffer underflow when writing to
  // mpeg format
  double muxMaxDelay = 0.7f;
  this->dataPtr->formatCtx->max_delay =
    static_cast<int>(muxMaxDelay * AV_TIME_BASE);

  if (!(outputFormat->flags & AVFMT_NOFILE))
  {
    if (avio_open(&this->dataPtr->formatCtx->pb, tmpFileNameFull.c_str(),
        AVIO_FLAG_WRITE) < 0)
    {
      gzerr << "Could not open '" << tmpFileNameFull << "'."
            << "Video encoding is not started\n";
      this->Reset();
      return false;
    }
  }

  // Write the stream header, if any.
  if (avformat_write_header(this->dataPtr->formatCtx, nullptr) < 0)
  {
    gzerr << "Error occured when opening output file."
          << "Video encoding is not started\n";
    this->Reset();
    return false;
  }

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
          AV_PIX_FMT_RGB24, this->dataPtr->inWidth,
          this->dataPtr->inHeight);
    }

    this->dataPtr->swsCtx = sws_getContext(
        this->dataPtr->inWidth,
        this->dataPtr->inHeight,
        AV_PIX_FMT_RGB24,
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

  // encode
  memcpy(this->dataPtr->avInPicture->data[0], _frame,
         this->dataPtr->inWidth * this->dataPtr->inHeight * 3);

  sws_scale(this->dataPtr->swsCtx,
      this->dataPtr->avInPicture->data,
      this->dataPtr->avInPicture->linesize,
      0, this->dataPtr->inHeight,
      this->dataPtr->avOutFrame->data,
      this->dataPtr->avOutFrame->linesize);

  this->dataPtr->avOutFrame->pts = this->dataPtr->frameCount++;
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
      avPacket.stream_index = this->dataPtr->videoStream->index;

      // Scale timestamp appropriately.
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

      // Wriate frame to disk
      ret = av_interleaved_write_frame(this->dataPtr->formatCtx, &avPacket);

      if (ret < 0)
      {
        gzerr << "Error writing frame" << std::endl;
        return false;
      }
    }
    // gotOutput == false is not an error. It is an indicator that we do not
    // need to call av_interleaved_write_frame.
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

  this->dataPtr->encoding = false;

  return true;
#endif
  return false;
}

/////////////////////////////////////////////////
bool VideoEncoder::SaveToFile(const std::string &_filename)
{
#ifdef HAVE_FFMPEG
  // First stop the recording
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
  // Make sure the video has been stopped.
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
    // This frees the context and all the streams
    av_free(this->dataPtr->formatCtx);
    this->dataPtr->formatCtx = nullptr;
    this->dataPtr->videoStream = nullptr;
  }

  if (this->dataPtr->avInPicture)
  {
    av_free(this->dataPtr->avInPicture);
    this->dataPtr->avInPicture = nullptr;
  }

  if (this->dataPtr->avOutFrame)
  {
    av_free(this->dataPtr->avOutFrame);
    this->dataPtr->avOutFrame = nullptr;
  }

  if (this->dataPtr->swsCtx)
  {
    sws_freeContext(this->dataPtr->swsCtx);
    this->dataPtr->swsCtx = nullptr;
  }
#endif

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
