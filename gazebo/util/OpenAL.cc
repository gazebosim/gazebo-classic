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


#include <stdio.h>
#include <unistd.h>
#include <iostream>

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

#include "gazebo/common/Common.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/AudioDecoder.hh"
#include "gazebo/util/OpenAL.hh"

#ifdef HAVE_OPENAL
using namespace gazebo;
using namespace util;

/////////////////////////////////////////////////
OpenAL::OpenAL()
{
  this->context = NULL;
  this->audioDevice = NULL;
  this->listener = NULL;
}

/////////////////////////////////////////////////
OpenAL::~OpenAL()
{
  this->Fini();
}

/////////////////////////////////////////////////
bool OpenAL::Load()
{
  std::string deviceName = "default";

  // \todo Add back in the ability to set the hardwar device
  // Get the audio device name
  // if (node)
  // {
  //   deviceName = node->GetString("device", "default", 0);
  // }

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
    return false;
  }

  // Create the audio context
  this->context = alcCreateContext(this->audioDevice, NULL);

  if (this->context == NULL)
  {
    gzerr << "Unable to create OpenAL Context.\nAudio will be disabled.\n";
    return false;
  }

  // Make the context current
  alcMakeContextCurrent(this->context);

  // Clear error code
  alGetError();

  alDistanceModel(AL_EXPONENT_DISTANCE);

  return true;
}

/////////////////////////////////////////////////
void OpenAL::Init()
{
}

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////
OpenALSink *OpenAL::CreateListener(sdf::ElementPtr /*_sdf*/)
{
  OpenALSink *result = NULL;

  if (this->listener == NULL)
  {
    this->listener = new OpenALSink;
    result  = this->listener;
  }
  else
    gzerr << "An OpenALSink has already been created."
     << "Only one is allowed.\n";

  return result;
}

/////////////////////////////////////////////////
OpenALSource *OpenAL::CreateSource(sdf::ElementPtr _sdf)
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
  source->Load(_sdf);

  // Return a pointer to the source
  return source;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//                      OPENAL LISTENER
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////
OpenALSink::OpenALSink()
{
}

/////////////////////////////////////////////////
OpenALSink::~OpenALSink()
{
}

////////////////////////////////////////////////////////////////////////////////
void OpenALSink::SetPose(const math::Pose &_pose)
{
  ALenum error;

  // Clear error state
  alGetError();

  alListener3f(AL_POSITION, _pose.pos.x, _pose.pos.y, _pose.pos.z);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Error: [" << error << "]\n";
  }

  ALfloat orient[] = {1, 0, 0, 0, 0, 1};

  // Clear error state
  alGetError();

  alListenerfv(AL_ORIENTATION, orient);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Error: [" << error << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
void OpenALSink::SetVel(const math::Vector3 &_vel)
{
  ALenum error;

  // Clear error state
  alGetError();

  alListener3f(AL_VELOCITY, _vel.x, _vel.y, _vel.z);
  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Error: [" <<  error << "]\n";
  }
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//                      OPENAL SOURCE
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////
OpenALSource::OpenALSource()
{
  // Create 1 source
  alGenSources(1, &this->alSource);

  // Create 1 buffer
  alGenBuffers(1, &this->alBuffer);
}

/////////////////////////////////////////////////
OpenALSource::~OpenALSource()
{
  alDeleteSources(1, &this->alSource);
  alDeleteBuffers(1, &this->alBuffer);
}

/////////////////////////////////////////////////
void OpenALSource::Load(sdf::ElementPtr _sdf)
{
  if (_sdf->HasElement("pitch"))
    this->SetPitch(_sdf->Get<double>("pitch"));

  if (_sdf->HasElement("gain"))
    this->SetGain(_sdf->Get<double>("gain"));

  if (_sdf->HasElement("loop"))
    this->SetLoop(_sdf->Get<bool>("loop"));

  if (_sdf->HasElement("uri"))
    this->FillBufferFromFile(_sdf->Get<std::string>("uri"));
  else
    gzerr << "<audio_source> is missing <uri>...</uri> element\n";
}

/////////////////////////////////////////////////
int OpenALSource::SetPos(const math::Vector3 &_pos)
{
  ALfloat p[3] = {static_cast<float>(_pos.x),
    static_cast<float>(_pos.y), static_cast<float>(_pos.z)};
  ALenum error;

  // Clear error state
  alGetError();

  alSourcefv(this->alSource, AL_POSITION, p);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << "Error: [" << error << "]\n";
    return -1;
  }

  return 0;
}

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////
void OpenALSource::Play()
{
  int sourceState;
  alGetSourcei(this->alSource, AL_SOURCE_STATE, &sourceState);

  // Play the source, if it's not already playing
  if (sourceState != AL_PLAYING)
  {
    alSourcePlay(this->alSource);
  }
}

/////////////////////////////////////////////////
void OpenALSource::Pause()
{
  int sourceState;
  alGetSourcei(this->alSource, AL_SOURCE_STATE, &sourceState);

  // Pause the source if it playing
  if (sourceState == AL_PLAYING)
    alSourcePause(this->alSource);
}

/////////////////////////////////////////////////
void OpenALSource::Stop()
{
  int sourceState;
  alGetSourcei(this->alSource, AL_SOURCE_STATE, &sourceState);

  // Stop the source if it is not already stopped
  if (sourceState != AL_STOPPED)
    alSourcePause(this->alSource);
}

/////////////////////////////////////////////////
void OpenALSource::Rewind()
{
  alSourceRewind(this->alSource);
}

/////////////////////////////////////////////////
bool OpenALSource::IsPlaying()
{
  int sourceState;
  alGetSourcei(this->alSource, AL_SOURCE_STATE, &sourceState);

  return sourceState == AL_PLAYING;
}

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////
void OpenALSource::FillBufferFromFile(const std::string &_audioFile)
{
  std::string fullPathAudioFile = common::find_file(_audioFile);

  // Try to open the audio file in the current directory
  FILE *testFile = fopen(fullPathAudioFile.c_str(), "r");

  // If the audio file couldn't be opened, try the gazebo paths
  if (testFile == NULL)
  {
    gzerr << "Unable to open audio file[" << _audioFile << "]\n";
  }

  uint8_t *dataBuffer = NULL;

#ifdef HAVE_FFMPEG
  unsigned int dataBufferSize;
  // Create an audio decoder
  common::AudioDecoder audioDecoder;

  // Set the audio file to decode
  audioDecoder.SetFile(fullPathAudioFile);
  audioDecoder.Decode(&dataBuffer, &dataBufferSize);

  // Copy raw buffer into AL buffer
  // AL_FORMAT_MONO8, AL_FORMAT_MONO16, AL_FORMAT_STEREO8,
  // AL_FORMAT_STEREO16
  this->FillBufferFromPCM(dataBuffer, dataBufferSize,
      audioDecoder.GetSampleRate());
#else
  std::cerr << "No FFMPEG audio decoder. Missing FFMPEG libraries.\n";
#endif

  if (dataBuffer)
    delete [] dataBuffer;

  fclose(testFile);
}
#endif
