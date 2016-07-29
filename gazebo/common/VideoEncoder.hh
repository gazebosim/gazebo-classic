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
#ifndef GAZEBO_COMMON_VIDEOENCODER_HH_
#define GAZEBO_COMMON_VIDEOENCODER_HH_

#include <chrono>
#include <string>
#include "gazebo/common/Time.hh"
#include <gazebo/util/system.hh>

#define VIDEO_ENCODER_BITRATE_DEFAULT 2000000
#define VIDEO_ENCODER_WIDTH_DEFAULT 1024
#define VIDEO_ENCODER_HEIGHT_DEFAULT 768
#define VIDEO_ENCODER_FPS_DEFAULT 25
#define VIDEO_ENCODER_FORMAT_DEFAULT "ogv"

namespace gazebo
{
  namespace common
  {
    // Forward declare private data class
    class VideoEncoderPrivate;

    /// \addtogroup gazebo_common
    /// \{

    /// \class VideoEncoder VideoEncoder.hh common/common.hh
    /// \brief Handle video encoding using libavcodec
    class GZ_COMMON_VISIBLE VideoEncoder
    {
      /// \brief Constructor
      public: VideoEncoder();

      /// \brief Destructor
      public: virtual ~VideoEncoder();

      /// \brief Initialize the encoder
      /// \return True on success
      public: bool Start(
                const unsigned int _width = VIDEO_ENCODER_WIDTH_DEFAULT,
                const unsigned int _height = VIDEO_ENCODER_HEIGHT_DEFAULT,
                const std::string &_format = VIDEO_ENCODER_FORMAT_DEFAULT,
                const unsigned int _fps = VIDEO_ENCODER_FPS_DEFAULT,
                const unsigned int _bitRate = VIDEO_ENCODER_BITRATE_DEFAULT);

      /// \brief Stop encoding. This will also automatically be called
      /// by SaveToFile
      public: bool Stop();

      /// \brief True if the enoder is initialized, false otherwise
      /// \return True if initialized.
      public: bool IsEncoding();

      public: bool AddFrame(const unsigned char *_frame);

      /// \brief Add a single timestamped frame to be encoded
      /// \param[in] _frame Image buffer to be encoded
      /// \param[in] _width Input frame width
      /// \param[in] _height Input frame height
      /// \param[in] _timestamp Timestamp of the image frame
      /// \return True on success.
      public: bool AddFrame(const unsigned char *_frame,
                  //const std::chrono::system_clock::time_point &_timestamp);
                  const std::chrono::steady_clock::time_point &_timestamp);

      public: bool AddFrame(const unsigned char *_frame,
                  const common::Time &_timestamp);

      /// \brief Write data buffer to to disk
      /// param[in] _filename File in which to save the encoded data
      /// \return True on success.
      public: bool SaveToFile(const std::string &_filename);

      /// \brief Get the encoding format
      /// \return Encoding format
      public: std::string Format() const;

      /// \brief Reset to default video properties and clean up allocated
      /// memories. This will also delete any temporary files.
      public: void Reset();

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<VideoEncoderPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
