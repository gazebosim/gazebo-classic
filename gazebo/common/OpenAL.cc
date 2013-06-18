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

#include <gazebo/gazebo_config.h>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Exception.hh>

#ifdef HAVE_OPENAL

#include <stdio.h>
#include <unistd.h>
#include <iostream>

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

#include "gazebo/common/AudioDecoder.hh"
#include "gazebo/common/OpenAL.hh"

using namespace gazebo;
using namespace common;

////////////////////////////////////////////////////////////////////////////////
OpenAL::OpenAL()
{
  this->context = NULL;
  this->audioDevice = NULL;
}

////////////////////////////////////////////////////////////////////////////////
OpenAL::~OpenAL()
{
  this->Fini();
}

////////////////////////////////////////////////////////////////////////////////
void OpenAL::Load()
{
  std::string deviceName = "default";

  // Get the audio device name
  /*if (node)
  {
    deviceName = node->GetString("device", "default", 0);
  }*/

  // Open the default audio device
  if (deviceName == "default")
    this->audioDevice = alcOpenDevice(NULL);
  else
    this->audioDevice = alcOpenDevice(deviceName.c_str());

  // Make sure that we could open the audio device
  if (this->audioDevice == NULL)
  {
    gzerr << "Unable to open audio device["
      << deviceName << "]\n Audio will be disabled.\n";
    return;
  }

  // Create the audio context
  this->context = alcCreateContext(this->audioDevice, NULL);

  if (this->context == NULL)
  {
    gzerr << "Unable to create OpenAL Context.\nAudio will be disabled.\n";
    return;
  }

  // Make the context current
  alcMakeContextCurrent(this->context);

  //Clear error code
  alGetError();

  // TODO: put in function to set distance model
  //alDistanceModel(AL_EXPONENT_DISTANCE);
}

////////////////////////////////////////////////////////////////////////////////
void OpenAL::Init()
{
}

////////////////////////////////////////////////////////////////////////////////
void OpenAL::Fini()
{
  if (this->audioDevice)
  {
    alcCloseDevice(this->audioDevice);
  }

  if (this->context)
  {
    this->context = alcGetCurrentContext();
    alcMakeContextCurrent(NULL);
    alcDestroyContext(this->context);
  }
}

////////////////////////////////////////////////////////////////////////////////
OpenALSource *OpenAL::CreateSource()
{
  // Make sure the audio device has been opened
  if (!this->audioDevice)
  {
    gzerr << "Audio device not open\n";
    return NULL;
  }

  // Create a source
  OpenALSource *source = new OpenALSource();

  // Load the source
  source->Load();

  // Return a pointer to the source
  return source;
}

////////////////////////////////////////////////////////////////////////////////
void OpenAL::SetListenerPos(const math::Vector3 &_pos)
{
  ALenum error;

  // Make sure we have an audio device
  if (!this->audioDevice)
  {
    gzerr << "Audio disabled\n";
    return;
  }

  // Clear error state
  alGetError();

  alListener3f(AL_POSITION, _pos.x, _pos.y, _pos.z);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Error: [" << error << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
void OpenAL::SetListenerVel(const math::Vector3 &_vel)
{
  ALenum error;

  // Make sure we have an audio device
  if (!this->audioDevice)
  {
    gzerr << "Audio disabled\n";
  }

  // Clear error state
  alGetError();

  alListener3f(AL_VELOCITY, _vel.x, _vel.y, _vel.z);
  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Error: [" <<  error << "]\n";
  }

}

////////////////////////////////////////////////////////////////////////////////
void OpenAL::SetListenerOrient(float _cx, float _cy, float _cz,
                               float _ux, float _uy, float _uz)
{
  ALenum error;
  ALfloat orient[] = {_cx, _cy, _cz, _ux, _uy, _uz};

  // Make sure we have an audio device
  if (!this->audioDevice)
  {
    gzerr << "Audio disabled\n";
  }

  // Clear error state
  alGetError();

  alListenerfv(AL_ORIENTATION, orient);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Error: [" << error << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//                      OPENAL SOURCE
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
OpenALSource::OpenALSource()
{
  //Create 1 source
  alGenSources(1, &this->alSource);

  // Create 1 buffer
  alGenBuffers(1, &this->alBuffer);
}

////////////////////////////////////////////////////////////////////////////////
OpenALSource::~OpenALSource()
{
  alDeleteSources(1, &this->alSource);
  alDeleteBuffers(1, &this->alBuffer);
}

////////////////////////////////////////////////////////////////////////////////
void OpenALSource::Load()
{
  // Set the pitch of the source
  /*this->SetPitch( node->GetDouble("pitch",1.0,0) );

  // Set the gain of the source
  this->SetGain( node->GetDouble("gain",1.0,0) );

  // Set whether the source should loop when played 
  this->SetLoop( node->GetBool("loop",false,0) );

  if (node->GetChild("mp3") != NULL)
    this->FillBufferFromFile(node->GetString("mp3","",1));
    */

  this->SetPitch(0.5);
  this->SetGain(0.5);
  this->SetLoop(true);
  this->FillBufferFromFile("/home/nkoenig/Music/track.mp3");
}

////////////////////////////////////////////////////////////////////////////////
int OpenALSource::SetPos(const math::Vector3 &_pos)
{
  ALfloat p[3] = {static_cast<float>(_pos.x),
    static_cast<float>(_pos.y), static_cast<float>(_pos.z)};
  ALenum error;

  // Clear error state
  alGetError();

  alSourcefv( this->alSource, AL_POSITION, p);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << "Error: [" << error << "]\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
int OpenALSource::SetVel(const math::Vector3 &_vel)
{
  ALenum error;
  ALfloat v[3] = {static_cast<float>(_vel.x), 
    static_cast<float>(_vel.y), static_cast<float>(_vel.z)};

  // Clear error state
  alGetError();

  alSourcefv(this->alSource, AL_VELOCITY, v);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << "Error: [" << error << "]\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
int OpenALSource::SetPitch(float _pitch)
{
  ALenum error;

  // clear error state
  alGetError();

  alSourcef(this->alSource, AL_PITCH, _pitch);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Error: [" << error << "]\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
int OpenALSource::SetGain(float _gain)
{

  ALenum error;

  // clear error state
  alGetError();

  alSourcef(this->alSource, AL_GAIN, _gain);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << "Error: [" << error << "]\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
int OpenALSource::SetLoop(bool _state)
{
  ALenum error;

  // clear error state
  alGetError();

  // Set looping state 
  alSourcei(this->alSource, AL_LOOPING, _state);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Error: [" << error << "]\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
void OpenALSource::Play()
{
  int sourceState;
  alGetSourcei(this->alSource, AL_SOURCE_STATE, &sourceState);

  // Play the source, if it's not already playing
  if (sourceState != AL_PLAYING)
  {
    alSourcePlay( this->alSource );
  }
}

////////////////////////////////////////////////////////////////////////////////
void OpenALSource::Pause()
{
  int sourceState;
  alGetSourcei(this->alSource, AL_SOURCE_STATE, &sourceState);

  // Pause the source if it playing
  if (sourceState == AL_PLAYING)
    alSourcePause(this->alSource);
}

////////////////////////////////////////////////////////////////////////////////
void OpenALSource::Stop()
{
  int sourceState;
  alGetSourcei(this->alSource, AL_SOURCE_STATE, &sourceState);

  // Stop the source if it is not already stopped
  if (sourceState != AL_STOPPED)
    alSourcePause(this->alSource);
}

////////////////////////////////////////////////////////////////////////////////
void OpenALSource::Rewind()
{
  alSourceRewind(this->alSource);
}

////////////////////////////////////////////////////////////////////////////////
bool OpenALSource::IsPlaying()
{
  int sourceState;
  alGetSourcei(this->alSource, AL_SOURCE_STATE, &sourceState);

  return sourceState == AL_PLAYING;
}

////////////////////////////////////////////////////////////////////////////////
void OpenALSource::FillBufferFromPCM(uint8_t *_pcmData,
    unsigned int _dataCount, int _sampleRate)
{
  // First detach the buffer
  alSourcei(this->alSource, AL_BUFFER, 0);

  // Copy raw buffer into AL buffer
  // AL_FORMAT_MONO8, AL_FORMAT_MONO16, AL_FORMAT_STEREO8,
  // AL_FORMAT_STEREO16
  alBufferData(this->alBuffer, AL_FORMAT_MONO16, _pcmData, _dataCount, 
      _sampleRate);

  // Attach buffer to source
  alSourcei(this->alSource, AL_BUFFER, this->alBuffer);

  if (alGetError() != AL_NO_ERROR)
  {
    gzthrow("Unable to copy data into openAL buffer\n");
  }
}

////////////////////////////////////////////////////////////////////////////////
void OpenALSource::FillBufferFromFile(const std::string &_audioFile)
{
  std::string fullPathAudioFile = _audioFile;

  // Try to open the audio file in the current directory
  FILE *testFile = fopen(fullPathAudioFile.c_str(), "r");

  // If the audio file couldn't be opened, try the gazebo paths
  if (testFile == NULL)
  {
    gzerr << "Unable to open audio file[" << _audioFile << "]\n";

    /*std::list<std::string>::iterator iter;
    GazeboConfig *gzconfig = Simulator::Instance()->GetGazeboConfig();

    for (iter = gzconfig->GetGazeboPaths().begin();
        iter != gzconfig->GetGazeboPaths().end();
        iter++)
    {
      fullPathAudioFile = *iter + "/Media/audio/" + audioFile;
      testFile = fopen(fullPathAudioFile.c_str(), "r");

      if (testFile)
      {
        break;
      }
    }*/
  }

  uint8_t *dataBuffer = NULL;
  unsigned int dataBufferSize;

#ifdef HAVE_FFMPEG
  // Create an audio decoder
  AudioDecoder audioDecoder;

  // Set the audio file to decode
  audioDecoder.SetFile(fullPathAudioFile);
  audioDecoder.Decode(&dataBuffer, &dataBufferSize);

  // Copy raw buffer into AL buffer
  // AL_FORMAT_MONO8, AL_FORMAT_MONO16, AL_FORMAT_STEREO8,
  // AL_FORMAT_STEREO16
  this->FillBufferFromPCM( dataBuffer, dataBufferSize, 
                           audioDecoder.GetSampleRate());
#else
  std::cerr << "No FFMPEG audio decoder. Missing FFMPEG libraries.\n";
#endif

  if (dataBuffer)
    delete [] dataBuffer; 
}

#endif
