/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef _AUDIO_DECODER_HH_
#define _AUDIO_DECODER_HH_

#include <string>
#include <stdint.h>

struct AVFormatContext;
struct AVCodecContext;
struct AVCodec;

namespace gazebo
{
  namespace common
  {
    /// \class AudioDecoder AudioDecoder.hh common/common.hh
    /// \brief An audio decoder based on FFMPEG
    class AudioDecoder
    {
      /// \brief Constructor
      public: AudioDecoder();

      /// \brief Destructor
      public: virtual ~AudioDecoder();

      /// \brief Set the file to decode.
      /// \param[in] _filename Path to an audio file.
      /// \return True if the file was successfull opened.
      public: bool SetFile(const std::string &_filename);

      /// \brief Decode the loaded audio file.
      /// \sa AudioDecoder::SetFile
      /// \param[out] _outBuffer Buffer that holds the decoded audio data.
      /// \param[out] _outBufferSize Size of the _outBuffer
      public: int Decode(uint8_t **_outBuffer, unsigned int *_outBufferSize);

      /// \brief Get the sample rate from the latest decoded file
      public: int GetSampleRate();

      /// \brief libav Format I/O context
      private: AVFormatContext *formatCtx;

      /// \brief libav main external API structure
      private: AVCodecContext *codecCtx;

      /// \brief libavcodec audio codec
      private: AVCodec *codec;

      /// \brief Index of the audio stream
      private: int audioStream;

      /// \brief True when initialized. We just want to initialize once.
      private: static bool initialized;
    };
  }
}

#endif
