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
/* Desc: OpenAL Server
 * Author: Nathan Koenig
 * Date: 20 Jan 2008
 * SVN: $Id:$
 */

#ifndef OPENAL_HH
#define OPENAL_HH

#include <stdint.h>
#include <string>

#include "gazebo_config.h"
#include "common/Vector3.hh"

#ifdef HAVE_OPENAL
struct ALCcontext_struct;
struct ALCdevice_struct;
#endif

namespace gazebo
{

  class OpenALIface;
  class XMLConfigNode;
  class OpenALSource;
  
  class OpenAL
  {
    /// \brief Constructor
    private: OpenAL();
  
    /// \brief Destructor
    private: virtual ~OpenAL();

    /// \brief Load the OpenAL server
    public: void Load(XMLConfigNode *node);

    // \brief Get an instance of this class
    public: static OpenAL *Instance();
  
    /// \brief Initialize
    public: void Init();

    /// \brief Finalize
    public: void Fini();

    /// \brief Create an OpenALSource object from XML node
    /// \param node The XML config node containing the auiod information
    /// \return A pointer to an OpenALSource object
    public: OpenALSource *CreateSource( XMLConfigNode *node );

    /// \brief Set the listener position
    public: void SetListenerPos( const Vector3 pos );
  
    /// \brief Set the listener velocity
    public: void SetListenerVel( const Vector3 vel );
  
    /// \brief Set the listener orientation
    public: void SetListenerOrient(float cx, float cy, float cz,
                                        float ux, float uy, float uz);

#ifdef HAVE_OPENAL
    private: ALCcontext_struct *context;
    private: ALCdevice_struct *audioDevice;
#endif
  
    private: static OpenAL *myself;
  };

  /// \brief OpenAL Source. This can be thought of as a speaker.
  class OpenALSource
  {
    /// \brief Constructor
    public: OpenALSource();

    /// \brief Destructor
    public: virtual ~OpenALSource();

    /// \brief Load the source from xml
    public: void Load(XMLConfigNode *node);

    /// \brief Set the position of the source
    public: int SetPos(const Vector3 &pos);
  
    /// \brief Set the velocity of the source
    public: int SetVel(const Vector3 &vel);
  
    /// \brief Set the pitch of the source
    public: int SetPitch(float p);
  
    /// \brief Set the pitch of the source
    public: int SetGain(float g);
  
    /// \brief Set whether the source loops the audio 
    public: int SetLoop(bool state);
 
    /// \brief Play a sound
    public: void Play();

    /// \brief Pause a sound
    public: void Pause();

    /// \brief Stop a sound
    public: void Stop();

    /// \brief Rewind the sound to the beginning
    public: void Rewind();

    /// \brief Is the audio playing
    public: bool IsPlaying();

    /// \brief Fill the OpenAL audio buffer from PCM data
    public: void FillBufferFromPCM(uint8_t *pcmData, unsigned int dataCount, 
                                   int sampleRate );

    /// \brief Fill the OpenAL audio buffer with data from a sound file
    public: void FillBufferFromFile( const std::string &audioFile );

    // OpenAL source index
    private: unsigned int alSource;

    // OpenAL buffer index
    private: unsigned int alBuffer;
  };
}

#endif
