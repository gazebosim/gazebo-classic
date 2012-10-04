/*
 * Copyright 2011 Nate Koenig
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
#ifndef _VIDEO_HH_
#define _VIDEO_HH_

#include <string>

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

    /// \brief Handle video encoding and decoding using libavcodec
    class Video
    {
      /// \brief Constructor
      public: Video();

      /// \brief Destructor
      public: virtual ~Video();

      /// \brief Load a video file
      /// \param _filename Full path of the video file
      public: bool Load(const std::string &_filename);

      /// \brief Get the width of the video in pixels
      public: int GetWidth() const;

      /// \brief Get the height of the video in pixels
      public: int GetHeight() const;

      /// \brief Get the next frame of the video.
      /// \param _img Image in which the frame is stored
      public: bool GetNextFrame(unsigned char **_buffer);

      /// \brief free up open Video object, close files, streams
      private: void Cleanup();

      private: AVFormatContext *formatCtx;
      private: AVCodecContext *codecCtx;
      private: AVFrame *avFrame;
      private: AVPicture *pic;
      private: SwsContext *swsCtx;
      private: int videoStream;
    };
    /// \}
  }
}
#endif
