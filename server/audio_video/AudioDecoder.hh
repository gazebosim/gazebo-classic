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
/* Desc: Audio Decoder
 * Author: Nathan Koenig
 * Date: 20 Jan 2008
 * SVN: $Id:$
 */

#ifndef AUDIODECODER_HH
#define AUDIODECODER_HH

#include "gazebo_config.h"

#ifdef HAVE_FFMPEG

extern "C" {

#include <avformat.h>
#include <avcodec.h>
}

#include <string>

class AudioDecoder
{
  /// \brief Constructor
  public: AudioDecoder();

  /// \brief Destructor
  public: virtual ~AudioDecoder();

  /// \brief Set the file to decode
  public: int SetFile(const std::string &filename);

  /// \brief Decode and audio file
  public: int Decode(uint8_t **outBuffer, unsigned int *outBufferSize);

  /// \brief Get the sample rate from the latest decoded file
  public: int GetSampleRate();

  private: AVFormatContext *formatCtx;
  private: AVCodecContext *codecCtx;

  // libavcodec audio codec
  private: AVCodec *codec;

  // Index of the audio stream
  private: int audioStream;

  private: static bool initialized;
};

#endif
#endif
