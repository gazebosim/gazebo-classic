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

#define VIDEO_ENCODER_BITRATE_DEFAULT 800000
#define VIDEO_ENCODER_WIDTH_DEFAULT 1280
#define VIDEO_ENCODER_HEIGHT_DEFAULT 720
#define VIDEO_ENCODER_FPS_DEFAULT 25
#define VIDEO_ENCODER_FORMAT_DEFAULT "mp4"

namespace gazebo
{
  namespace common
  {
    // Forward declare private data class
    class VideoEncoderPrivate;

    /// \addtogroup gazebo_common
    /// \{

    /// \class VideoEncoder VideoEncoder.hh common/common.hh
    /// \brief The VideoEncoder class supports encoding a series of images
    /// to a video format, and then writing the video to disk.
    class GZ_COMMON_VISIBLE VideoEncoder
    {
      /// \brief Constructor
      public: VideoEncoder();

      /// \brief Destructor
      public: virtual ~VideoEncoder();

      /// \brief Start the encoder. This should be called once. Add new
      /// frames to the video using the AddFrame function.
      /// \param[in] _width Width in pixels of the output video.
      /// \param[in] _height Height in pixels of the output video.
      /// \param[in] _format String that represents the video type.
      /// Supported types include: "avi", "ogv", mp4".
      /// \param[in] _bitRate Bit rate to encode the video.
      /// \return True on success
      public: bool Start(
                const unsigned int _width = VIDEO_ENCODER_WIDTH_DEFAULT,
                const unsigned int _height = VIDEO_ENCODER_HEIGHT_DEFAULT,
                const std::string &_format = VIDEO_ENCODER_FORMAT_DEFAULT,
                const unsigned int _fps = VIDEO_ENCODER_FPS_DEFAULT,
                const unsigned int _bitRate = VIDEO_ENCODER_BITRATE_DEFAULT);

      /// \brief Stop the encoder. This will also automatically be called
      /// by SaveToFile
      /// \return True on success.
      public: bool Stop();

      /// \brief True if the enoder has been started, false otherwise
      /// \return True if started.
      public: bool IsEncoding();

      /// \brief Add a single frame to be encoded
      /// \param[in] _frame Image buffer to be encoded
      /// \param[in] _width Input frame width
      /// \param[in] _height Input frame height
      public: bool AddFrame(const unsigned char *_frame,
                  const unsigned int _width,
                  const unsigned int _height);

      /// \brief Add a single timestamped frame to be encoded
      /// \param[in] _frame Image buffer to be encoded
      /// \param[in] _width Input frame width
      /// \param[in] _height Input frame height
      /// \param[in] _timestamp Timestamp of the image frame
      /// \return True on success.
      public: bool AddFrame(const unsigned char *_frame,
                  const unsigned int _width,
                  const unsigned int _height,
                  //const std::chrono::system_clock::time_point &_timestamp);
                  const std::chrono::steady_clock::time_point &_timestamp);

      /// \brief Add a single frame to be encoded
      /// \param[in] _frame Image buffer to be encoded
      /// \param[in] _width Input frame width
      /// \param[in] _height Input frame height
      /// \param[in] _timestamp Timestamp of the image frame
      /// \return True on success.
      public: bool AddFrame(const unsigned char *_frame,
                  const unsigned int _width,
                  const unsigned int _height,
                  const common::Time &_timestamp);

      /// \brief Write the video to to disk
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
