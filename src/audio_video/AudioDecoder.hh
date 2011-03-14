/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Audio Decoder
 * Author: Nathan Koenig
 * Date: 20 Jan 2008
 * SVN: $Id:$
 */

#ifndef AUDIODECODER_HH
#define AUDIODECODER_HH

#include "gazebo_config.h"

#ifdef HAVE_FFMPEG

#include <inttypes.h>

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
