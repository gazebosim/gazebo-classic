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
#include <gazebo/util/system.hh>

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
      public: bool Init();

      /// \brief True if the enoder is initialized, false otherwise
      /// \return True if initialized.
      public: bool IsInitialized();

      /// \brief Add a single frame to be encoded
      /// \param[in] _frame Image buffer to be encoded
      /// \param[in] _width Input frame width
      /// \param[in] _height Input frame height
      /// \return True on success.
      public: bool AddFrame(const unsigned char *_frame,
                  const unsigned int _width, const unsigned int _height);

      /// \brief Add a single timestamped frame to be encoded
      /// \param[in] _frame Image buffer to be encoded
      /// \param[in] _width Input frame width
      /// \param[in] _height Input frame height
      /// \param[in] _timestamp Timestamp of the image frame
      /// \return True on success.
      public: bool AddFrame(const unsigned char *_frame,
                  const unsigned int _width, const unsigned int _height,
                  const std::chrono::system_clock::time_point &_timestamp);

      /// \brief Write data buffer to to disk
      /// param[in] _filename File in which to save the encoded data
      /// \return True on success.
      public: bool SaveToFile(const std::string &_filename);

      /// \brief Set the video encoding bit rate
      /// \param[in] _bitrate Video encoding bit rate
      public: void SetBitRate(const unsigned int _bitRate);

      /// \brief Set the output frame width
      /// \param[in] _width Frame width in pixels
      public: void SetFrameWidth(const unsigned int _width);

      /// \brief Set the output frame height
      /// \param[in] _height Frame height in pixels
      public: void SetFrameHeight(const unsigned int _height);

      /// \brief Set the encoding format, such as "ogv" or "mp4".
      /// \param[in] _height Frame height
      public: void SetFormat(const std::string &_format);

      /// \brief Get the encoding format
      /// \return Encoding format
      public: std::string Format() const;

      /// \brief Finalize encoding. This will automatically be called
      /// by SaveToFile
      public: void End();

      /// \brief Reset to default video properties and clean up allocated
      /// memories.
      public: void Reset();

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<VideoEncoderPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
