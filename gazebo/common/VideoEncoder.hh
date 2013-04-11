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
#ifndef _ENCODER_HH_
#define _ENCODER_HH_

#include <gazebo/common/Time.hh>
#include <string>

struct AVCodecContext;
struct AVFrame;
struct AVPicture;
struct SwsContext;
struct AVOutputFormat;
struct AVFormatContext;
struct AVStream;
struct AVPacket;

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common
    /// \{

    /// \class VideoEncoder VideoEncoder.hh common/common.hh
    /// \brief Handle video encoding using libavcodec
    class VideoEncoder
    {
      /// \brief Constructor
      public: VideoEncoder();

      /// \brief Destructor
      public: virtual ~VideoEncoder();

      /// \brief Initialize the encoder
      public: void Init();

      /// \brief True if the enoder is initialized, false otherwise
      public: bool IsInitialized();

      /// \brief Add a single frame to be encoded
      /// \param[in] _frame Image buffer to be encoded
      /// \param[in] _width Input frame width
      /// \param[in] _height Input frame height
      public: void AddFrame(unsigned char *_frame, unsigned int _width,
          unsigned int _height);

      /// \brief Add a single timestamped frame to be encoded
      /// \param[in] _frame Image buffer to be encoded
      /// \param[in] _width Input frame width
      /// \param[in] _height Input frame height
      /// \param[in] _timestamp Timestamp of the image frame
      public: void AddFrame(unsigned char *_frame, unsigned int _width,
          unsigned int _height, common::Time _timestamp);

      /// \brief Write data buffer to to disk
      /// param[in] _filename File in which to save the encoded data
      public: void SaveToFile(const std::string &_filename);

      /// \brief Set the video encoding bit rate
      /// \param[in] _bitrate Video encoding bit rate
      public: void SetBitRate(unsigned int _bitRate);

      /// \brief Set the output frame width
      /// \param[in] _width Frame width in pixels
      public: void SetFrameWidth(unsigned int _width);

      /// \brief Set the output frame height
      /// \param[in] _height Frame height in pixels
      public: void SetFrameHeight(unsigned int _height);

      /// \brief Set the encoding format
      /// \param[in] _height Frame height
      public: void SetFormat(const std::string &_format);

      /// \brief Get the encoding format
      /// \return Encoding format
      public: std::string GetFormat() const;

      /// \brief Reset to default video properties and clean up allocated
      /// memories.
      public: void Reset();

      /// \brief Finalize encoding
      public: void Fini();

      /// \brief free up open Video object, close files, streams
      private: void Cleanup();

      /// \brief libav main external API structure
      private: AVCodecContext *codecCtx;

      /// \brief libav muxing
      private: AVOutputFormat *outputFormat;

      /// \brief libav format I/O context
      private: AVFormatContext *formatCtx;

      /// \brief libav audio video stream
      private: AVStream *videoStream;

      /// \brief libav image data (used for storing RGB data)
      private: AVPicture *avInPicture;
//      private: AVPicture *pic;

      /// \brief libav audio or video data (used for storing YUV data)
      private: AVFrame *avOutFrame;

      /// \brief Software scaling context
      private: SwsContext *swsCtx;

      /// \brief Encoding buffer
      private: unsigned char *outbuf;

      /// \brief Size of the picture buffer
      private: unsigned char *pictureBuf;

      /// \brief True if the encoder is initialized
      private: bool initialized;

      /// \brief Video encoding bit rate
      private: unsigned int bitRate;

      /// \brief Output frame width
      private: unsigned int frameWidth;

      /// \brief Output frame height
      private: unsigned int frameHeight;

      /// \brief Size of the output buffer.
      private: int outBufferSize;

      /// \brief Temporary filename to write the file to.
      private: std::string tmpFilename;

      /// \brief Encoding format
      private: std::string format;

      /// \brief Handl to the output video file.
      private: FILE *fileHandle;

      /// \brief Number of bytes used from buffer.
      private: int outSize;

      /// \brief Target framerate.
      private: unsigned int fps;

      /// \brief Previous time when the frame is added.
      private: common::Time timePrev;

      /// \brief Encoding sample rate.
      private: int sampleRate;

      /// \brief total time elapsed.
      private: double totalTime;

      /// \brief Video presenetation time stamp.
      private: int videoPts;

      /// \brief Input frame width
      private: unsigned int inFrameWidth;

      /// \brief Input frame height
      private: unsigned int inFrameHeight;
    };
    /// \}
  }
}
#endif
