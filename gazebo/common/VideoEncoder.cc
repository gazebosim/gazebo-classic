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
}
#endif

#include "gazebo/math/Helpers.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/VideoEncoder.hh"

using namespace gazebo;
using namespace common;

#define VIDEO_ENCODER_TMPFILENAME_DEFAULT "TMP_RECORDING"
#define VIDEO_ENCODER_SAMPLERATE_DEFAULT VIDEO_ENCODER_FPS_DEFAULT * 2
#define VIDEO_ENCODER_VIDEOPTS_DEFAULT -1
//#define INFRAMEWIDTH_DEFAULT 0
//#define INFRAMEHEIGHT_DEFAULT 0

// Private data class
class gazebo::common::VideoEncoderPrivate
{
  /// \brief Get the full filename of the temporary video file
  public: std::string TmpFilename() const;

  /// \brief libav main external API structure
  public: AVCodecContext *codecCtx = nullptr;

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
  public: unsigned char *pictureBuf = nullptr;

  /// \brief Handl to the output video file.
  public: FILE *fileHandle = nullptr;

  /// \brief True if the encoder is initialized
  public: bool encoding = false;

  /// \brief Video encoding bit rate
  public: unsigned int bitRate = VIDEO_ENCODER_BITRATE_DEFAULT;

  /// \brief Output frame width
  public: unsigned int width = VIDEO_ENCODER_WIDTH_DEFAULT;

  /// \brief Output frame height
  public: unsigned int height = VIDEO_ENCODER_HEIGHT_DEFAULT;

  /// \brief Temporary filename to write the file to.
  public: std::string tmpFilename = VIDEO_ENCODER_TMPFILENAME_DEFAULT;

  /// \brief Encoding format
  public: std::string format = VIDEO_ENCODER_FORMAT_DEFAULT;

  /// \brief Target framerate.
  public: unsigned int fps = VIDEO_ENCODER_FPS_DEFAULT;

  /// \brief Previous time when the frame is added.
  public: std::chrono::steady_clock::time_point timePrev;

  public: gazebo::common::Time timePrev2;

  /// \brief Encoding sample rate.
  public: int sampleRate = VIDEO_ENCODER_SAMPLERATE_DEFAULT;

  /// \brief total time elapsed, in seconds.
  public: std::chrono::duration<double> totalTime;

  public: gazebo::common::Time totalTime2;

  /// \brief Video presentation time stamp.
  public: int videoPts = VIDEO_ENCODER_VIDEOPTS_DEFAULT;

  /// \brief Input frame width
  // public: unsigned int inFrameWidth = INFRAMEWIDTH_DEFAULT;

  /// \brief Input frame height
  // public: unsigned int inFrameHeight = INFRAMEHEIGHT_DEFAULT;

  public: unsigned int frameCount = 0;
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

  this->dataPtr->bitRate = _bitRate;
  this->dataPtr->width = _width;
  this->dataPtr->height = _height;
  this->dataPtr->format = _format;
  this->dataPtr->fps = _fps;
  this->dataPtr->frameCount = 0;

#ifdef HAVE_FFMPEG
  std::string tmpFileNameFull = this->dataPtr->TmpFilename();

  this->dataPtr->outputFormat =
    av_guess_format(NULL, tmpFileNameFull.c_str(), NULL);

  if (!this->dataPtr->outputFormat)
  {
    gzerr << "Could not deduce output format from file extension: "
        << "using MPEG.\n";
    this->dataPtr->outputFormat = av_guess_format("mpeg", NULL, NULL);
  }

  AVCodec *codec = nullptr;

  // find the video encoder
  codec = avcodec_find_encoder(this->dataPtr->outputFormat->video_codec);
  if (!codec)
  {
    gzerr << "Codec not found\n";
    return false;
  }

  this->dataPtr->formatCtx = avformat_alloc_context();
  this->dataPtr->formatCtx->oformat = this->dataPtr->outputFormat;
  snprintf(this->dataPtr->formatCtx->filename,
      sizeof(this->dataPtr->formatCtx->filename),
      "%s", tmpFileNameFull.c_str());

  this->dataPtr->videoStream =
    avformat_new_stream(this->dataPtr->formatCtx, codec);

  this->dataPtr->videoStream->id = this->dataPtr->formatCtx->nb_streams-1;

  this->dataPtr->codecCtx = this->dataPtr->videoStream->codec;

  // some formats want stream headers to be separate
  if (this->dataPtr->formatCtx->oformat->flags & AVFMT_GLOBALHEADER)
    this->dataPtr->codecCtx->flags |= CODEC_FLAG_GLOBAL_HEADER;

  // put sample parameters
  this->dataPtr->codecCtx->bit_rate = this->dataPtr->bitRate;
  this->dataPtr->codecCtx->rc_max_rate = this->dataPtr->bitRate;
  this->dataPtr->codecCtx->rc_min_rate = this->dataPtr->bitRate;

  // resolution must be a multiple of two
  this->dataPtr->codecCtx->width = this->dataPtr->width;
  this->dataPtr->codecCtx->height = this->dataPtr->height;

  // frames per second
  this->dataPtr->codecCtx->time_base.den = this->dataPtr->fps;
  this->dataPtr->codecCtx->time_base.num= 1;

  // emit one intra frame every ten frames
  this->dataPtr->codecCtx->gop_size = 10;
  this->dataPtr->codecCtx->pix_fmt = PIX_FMT_YUV420P;

  // this removes VBV buffer size not set warning msg
  this->dataPtr->codecCtx->rc_initial_buffer_occupancy = this->dataPtr->bitRate;
  this->dataPtr->codecCtx->rc_max_rate = this->dataPtr->bitRate;
  this->dataPtr->codecCtx->rc_buffer_size = this->dataPtr->bitRate;
  this->dataPtr->codecCtx->thread_count = 5;

  if (this->dataPtr->codecCtx->codec_id == CODEC_ID_MPEG1VIDEO)
  {
    // Needed to avoid using macroblocks in which some coeffs overflow.
    // This does not happen with normal video, it just happens here as
    // the motion of the chroma plane does not match the luma plane.
    this->dataPtr->codecCtx->mb_decision = 2;
  }

  // open it
  if (avcodec_open2(this->dataPtr->codecCtx, codec, NULL) < 0)
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

  this->dataPtr->avOutFrame = avcodec_alloc_frame();
  int size = avpicture_get_size(this->dataPtr->codecCtx->pix_fmt,
      this->dataPtr->codecCtx->width,
      this->dataPtr->codecCtx->height);

  this->dataPtr->pictureBuf = new unsigned char[size];

  avpicture_fill(reinterpret_cast<AVPicture *>(this->dataPtr->avOutFrame),
      this->dataPtr->pictureBuf, this->dataPtr->codecCtx->pix_fmt,
      this->dataPtr->codecCtx->width, this->dataPtr->codecCtx->height);

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
bool VideoEncoder::AddFrame(const unsigned char *_frame)
{
  return this->AddFrame(_frame, common::Time::GetWallTime());
      //std::chrono::steady_clock::now());
}

/////////////////////////////////////////////////
bool VideoEncoder::AddFrame(const unsigned char *_frame,
    const common::Time &_timestamp)
{
  if (!this->dataPtr->encoding)
  {
    gzerr << "Start encoding before adding a frame\n";
    return false;
  }

  auto dt = _timestamp - this->dataPtr->timePrev2;
  if (dt < 1.0/this->dataPtr->sampleRate)
  {
    gzwarn << "Time delta["
      //<< std::chrono::duration_cast<std::chrono::milliseconds>(dt).count()
      << " ms] lower than Hz rate[" << 1.0/this->dataPtr->sampleRate << "]\n";
    return false;
  }

  this->dataPtr->timePrev2 = _timestamp;

  int pts = 0;
  if (this->dataPtr->videoPts != -1)
  {
    //pts = (1.0 / this->dataPtr->fps) * 90 * this->dataPtr->totalTime.count();
    pts = (1.0 / this->dataPtr->fps) * 9000 * (this->dataPtr->frameCount++);
    this->dataPtr->totalTime2 += dt;

    if (this->dataPtr->videoPts == pts)
    {
      gzwarn << "Video presentation timestamp matches current timestamp\n";
      return false;
    }
  }

  this->dataPtr->videoPts = pts;
  std::cout << "PTS[" << this->dataPtr->videoPts << "]\n";

  // recreate the sws context on image resize
  /*if (this->dataPtr->swsCtx)
  {
    if (this->dataPtr->inFrameWidth != this->dataPtr->width ||
        this->dataPtr->inFrameHeight != this->dataPtr->height)
    {
      sws_freeContext(this->dataPtr->swsCtx);
      this->dataPtr->swsCtx = nullptr;
      if (this->dataPtr->avInPicture)
      {
        av_free(this->dataPtr->avInPicture);
        this->dataPtr->avInPicture = nullptr;
      }
    }
  }*/

  if (!this->dataPtr->swsCtx)
  {
    if (!this->dataPtr->avInPicture)
    {
      this->dataPtr->avInPicture = new AVPicture;
      avpicture_alloc(this->dataPtr->avInPicture,
          PIX_FMT_RGB24, this->dataPtr->width, this->dataPtr->height);
    }

    //this->dataPtr->inFrameWidth = this->dataPtr->width;
    //this->dataPtr->inFrameHeight = this->dataPtr->height;
    this->dataPtr->swsCtx = sws_getContext(
        this->dataPtr->width,
        this->dataPtr->height,
        PIX_FMT_RGB24,
        this->dataPtr->codecCtx->width, this->dataPtr->codecCtx->height,
        this->dataPtr->codecCtx->pix_fmt,
        SWS_BICUBIC, nullptr, nullptr, nullptr);

    if (this->dataPtr->swsCtx == nullptr)
    {
      gzerr << "Error while calling sws_getContext\n";
      return false;
    }
  }

  // encode
  memcpy(this->dataPtr->avInPicture->data[0], _frame,
      this->dataPtr->width * this->dataPtr->height * 3);

  sws_scale(this->dataPtr->swsCtx, this->dataPtr->avInPicture->data,
      this->dataPtr->avInPicture->linesize,
      0, this->dataPtr->height, this->dataPtr->avOutFrame->data,
      this->dataPtr->avOutFrame->linesize);

  this->dataPtr->codecCtx->coded_frame->pts = this->dataPtr->videoPts;

  int gotOutput = 0;
  AVPacket avPacket;
  av_init_packet(&avPacket);
  avPacket.data = nullptr;
  avPacket.size = 0;

  int ret = avcodec_encode_video2(this->dataPtr->codecCtx, &avPacket,
                                  this->dataPtr->avOutFrame, &gotOutput);

  if (ret >= 0)
  {
    this->dataPtr->codecCtx->coded_frame->pts = this->dataPtr->videoPts;

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

/////////////////////////////////////////////////
#ifdef HAVE_FFMPEG
bool VideoEncoder::AddFrame(const unsigned char *_frame,
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

  // recreate the sws context on image resize
  /*if (this->dataPtr->swsCtx)
  {
    if (this->dataPtr->inFrameWidth != this->dataPtr->width ||
        this->dataPtr->inFrameHeight != this->dataPtr->height)
    {
      sws_freeContext(this->dataPtr->swsCtx);
      this->dataPtr->swsCtx = nullptr;
      if (this->dataPtr->avInPicture)
      {
        av_free(this->dataPtr->avInPicture);
        this->dataPtr->avInPicture = nullptr;
      }
    }
  }*/

  if (!this->dataPtr->swsCtx)
  {
    if (!this->dataPtr->avInPicture)
    {
      this->dataPtr->avInPicture = new AVPicture;
      avpicture_alloc(this->dataPtr->avInPicture,
          PIX_FMT_RGB24, this->dataPtr->width, this->dataPtr->height);
    }

    //this->dataPtr->inFrameWidth = this->dataPtr->width;
    //this->dataPtr->inFrameHeight = this->dataPtr->height;
    this->dataPtr->swsCtx = sws_getContext(this->dataPtr->width,
        this->dataPtr->height, PIX_FMT_RGB24,
        this->dataPtr->codecCtx->width, this->dataPtr->codecCtx->height,
        this->dataPtr->codecCtx->pix_fmt,
        SWS_BICUBIC, nullptr, nullptr, nullptr);

    if (this->dataPtr->swsCtx == nullptr)
    {
      gzerr << "Error while calling sws_getContext\n";
      return false;
    }
  }

  // encode
  memcpy(this->dataPtr->avInPicture->data[0], _frame,
      this->dataPtr->width * this->dataPtr->height * 3);

  sws_scale(this->dataPtr->swsCtx, this->dataPtr->avInPicture->data,
      this->dataPtr->avInPicture->linesize,
      0, this->dataPtr->height, this->dataPtr->avOutFrame->data,
      this->dataPtr->avOutFrame->linesize);

  this->dataPtr->codecCtx->coded_frame->pts = this->dataPtr->videoPts;

  int gotOutput = 0;
  AVPacket avPacket;
  av_init_packet(&avPacket);
  avPacket.data = nullptr;
  avPacket.size = 0;

  int ret = avcodec_encode_video2(this->dataPtr->codecCtx, &avPacket,
                                  this->dataPtr->avOutFrame, &gotOutput);

  if (ret >= 0)
  {
    this->dataPtr->codecCtx->coded_frame->pts = this->dataPtr->videoPts;

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
    const std::chrono::steady_clock::time_point &_timestamp)
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
  this->dataPtr->formatCtx = nullptr;
  this->dataPtr->encoding = false;
  return true;
#endif
  return false;
}

/////////////////////////////////////////////////
bool VideoEncoder::SaveToFile(const std::string &_filename)
{
#ifdef HAVE_FFMPEG
  this->Stop();

  std::string oldName = this->dataPtr->TmpFilename();

  int result = rename(oldName.c_str(), _filename.c_str());
  if (result != 0)
  {
    gzerr << "Unable to rename file from[" << oldName << "] to ["
          << _filename << "]\n";
    return false;
  }

  this->Reset();
  return true;
#else
  gzwarn << _filename << " not saved" << std::endl;
  return false;
#endif
}

/////////////////////////////////////////////////
void VideoEncoder::Reset()
{
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

  std::ifstream f(tmpFileNameFull);
  bool good = f.good();
  f.close();
  if (good)
  {
    if (remove(tmpFileNameFull.c_str()) != 0)
      perror("Error deleting file");
    else
      std::cout << "Deleted file\n";
  }
  else
  {
    std::cout << "Not good[" << tmpFileNameFull  << "]\n";
  }

  if (this->dataPtr->pictureBuf)
  {
    delete [] this->dataPtr->pictureBuf;
    this->dataPtr->pictureBuf = nullptr;
  }

  // set default values
  this->dataPtr->frameCount = 0;
  this->dataPtr->bitRate = VIDEO_ENCODER_BITRATE_DEFAULT;
  this->dataPtr->width = VIDEO_ENCODER_WIDTH_DEFAULT;
  this->dataPtr->height = VIDEO_ENCODER_HEIGHT_DEFAULT;
  this->dataPtr->fps = VIDEO_ENCODER_FPS_DEFAULT;
  this->dataPtr->tmpFilename = VIDEO_ENCODER_TMPFILENAME_DEFAULT;
  this->dataPtr->format = VIDEO_ENCODER_FORMAT_DEFAULT;
  this->dataPtr->timePrev = {};
  this->dataPtr->sampleRate = VIDEO_ENCODER_SAMPLERATE_DEFAULT;
  this->dataPtr->totalTime = std::chrono::duration<double>(0);
  this->dataPtr->videoPts = VIDEO_ENCODER_VIDEOPTS_DEFAULT;
  //this->dataPtr->inFrameWidth = INFRAMEWIDTH_DEFAULT;
  //this->dataPtr->inFrameHeight = INFRAMEHEIGHT_DEFAULT;

  this->dataPtr->pictureBuf = nullptr;

  this->dataPtr->codecCtx = nullptr;
  this->dataPtr->swsCtx = nullptr;
  this->dataPtr->avInPicture = nullptr;
  this->dataPtr->avOutFrame = nullptr;
  this->dataPtr->formatCtx = nullptr;
}
