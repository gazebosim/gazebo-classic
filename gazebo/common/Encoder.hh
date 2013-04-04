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

//struct AVFormatContext;
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

    /// \class Encoder Encoder.hh common/common.hh
    /// \brief Handle video encoding using libavcodec
    class Encoder
    {
      /// \brief Constructor
      public: Encoder();

      /// \brief Destructor
      public: virtual ~Encoder();

      /// \brief Initialize the encoder
      public: void Init();

      /// \brief True if the enoder is initialized, false otherwise
      public: bool IsInitialized();

      /// \brief Add a single frame to be encoded
      /// \param[in] _frame Image buffer to be encoded
      /// \param[in] _w Input frame width
      /// \param[in] _h Input frame height
      public: void AddFrame(unsigned char *_frame, unsigned int _w,
          unsigned int _h);

      /// \brief Write data buffer to to disk
      /// param[in] _filename File in which to save the encoded data
      public: void SaveToFile(const std::string &_filename);

      /// \brief Set the video encoding bit rate
      /// \param[in] _bitrate Video encoding bit rate
      public: void SetBitRate(unsigned int _bitRate);

      /// \brief Set the frame width
      /// \param[in] _width Frame width
      public: void SetFrameWidth(unsigned int _width);

      /// \brief Set the frame height
      /// \param[in] _height Frame height
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

      private: AVOutputFormat *pOutputFormat;

      private: AVFormatContext *pFormatCtx;

      private: AVStream *pVideoStream;

      /// \brief audi video picture
      //private: AVPicture *pic;

      /// \brief Encoding buffer
      private: unsigned char *outbuf;

      private: unsigned char *pictureBuf;

      /// \brief True if the encoder is initialized
      private: bool initialized;

      /// \brief Video encoding bit rate
      private: unsigned int bitRate;

      /// \brief Frame width
      private: unsigned int frameWidth;

      /// \brief Frame height
      private: unsigned int frameHeight;

      private: int outBufferSize;

      /// \brief Audio video picture
      private: AVPicture *pic;

      /// \brief Software scaling context
      private: SwsContext *swsCtx;

      private: int currentBufferSize;

      private: AVFrame *avFrame;

      private: std::string tmpFilename;

      private: std::string format;

      /// \brief Handl to the output video file
      private: FILE *fileHandle;

      /// \brief Number of bytes used from buffer
      private: int outSize;

      private: unsigned int fps;

      private: common::Time timePrev;

      private: int sampleRate;

      private: double totalTime;

      private: int videoPts;
    };
    /// \}
  }
}
#endif
