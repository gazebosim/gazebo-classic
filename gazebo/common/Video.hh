/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_COMMON_VIDEO_HH_
#define _GAZEBO_COMMON_VIDEO_HH_

#include <string>
#include "gazebo/util/system.hh"

struct AVFormatContext;
struct AVCodecContext;
struct AVFrame;
struct AVPicture;
struct SwsContext;

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common
    /// \{

    /// \class Video Video.hh common/common.hh
    /// \brief Handle video encoding and decoding using libavcodec
    class GZ_COMMON_VISIBLE Video
    {
      /// \brief Constructor
      public: Video();

      /// \brief Destructor
      public: virtual ~Video();

      /// \brief Load a video file
      /// \param[in] _filename Full path of the video file
      /// \return false if HAVE_FFMPEG is not defined or if a video stream
      /// can't be found
      public: bool Load(const std::string &_filename);

      /// \brief Get the width of the video in pixels
      /// \return the width
      public: int GetWidth() const;

      /// \brief Get the height of the video in pixels
      /// \return the height
      public: int GetHeight() const;

      /// \brief Get the next frame of the video.
      /// \param[out] _img Image in which the frame is stored
      /// \return false if HAVE_FFMPEG is not defined, true otherwise
      public: bool GetNextFrame(unsigned char **_buffer);

      /// \brief free up open Video object, close files, streams
      private: void Cleanup();

      /// \brief libav Format I/O context
      private: AVFormatContext *formatCtx;

      /// \brief libav main external API structure
      private: AVCodecContext *codecCtx;

      /// \brief audio video frame
      private: AVFrame *avFrame;

      /// \brief Destination audio video frame
      private: AVFrame *avFrameDst;

      /// \brief software scaling context
      private: SwsContext *swsCtx;

      /// \brief index of first video stream or -1
      private: int videoStream;
    };
    /// \}
  }
}
#endif
