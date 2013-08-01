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

  // \todo Add back in the ability to set the hardware device
  // Get the audio device name
  // if (node)
  // {
  //   deviceName = node->GetString("device", "default", 0);
  // }

  // Open the default audio device
  // if (deviceName == "default")
  this->audioDevice = alcOpenDevice(NULL);
  // else
  //  this->audioDevice = alcOpenDevice(deviceName.c_str());

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
void OpenAL::Fini()
{
  if (this->audioDevice)
  {
    alcCloseDevice(this->audioDevice);
    this->audioDevice = NULL;
  }

  if (this->context)
  {
    this->context = alcGetCurrentContext();
    alcMakeContextCurrent(NULL);
    alcDestroyContext(this->context);
    this->context = NULL;
  }

  this->sink.reset();
}

/////////////////////////////////////////////////
OpenALSinkPtr OpenAL::CreateSink(sdf::ElementPtr /*_sdf*/)
{
  OpenALSinkPtr result;

  if (this->sink == NULL)
  {
    this->sink.reset(new OpenALSink);
    result  = this->sink;
  }
  else
    gzerr << "An OpenALSink has already been created."
     << "Only one is allowed.\n";

  return result;
}

/////////////////////////////////////////////////
OpenALSourcePtr OpenAL::CreateSource(sdf::ElementPtr _sdf)
{
  OpenALSourcePtr source;

  // Make sure the audio device has been opened
  if (!this->audioDevice)
  {
    gzerr << "Audio device not open\n";
    return source;
  }

  // Create a source
  source.reset(new OpenALSource());

  // Load the source
  if (!source->Load(_sdf))
  {
    gzerr << "Unable to load OpenAL source from SDF\n";
    source.reset();
  }

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
bool OpenALSink::SetPose(const math::Pose &_pose)
{
  ALenum error;

  // Clear error state
  alGetError();

  alListener3f(AL_POSITION, _pose.pos.x, _pose.pos.y, _pose.pos.z);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Unable to set pose. Error code[" <<  error << "]\n";
    return false;
  }

  math::Matrix3 rot = _pose.rot.GetAsMatrix3();

  // The first three values are the direction vector values.
  // The second three value are the up vector values.
  ALfloat orient[] = {rot[0][0], rot[0][1], rot[0][2],
                      rot[2][0], rot[2][1], rot[2][2]};

  // Clear error state
  alGetError();

  alListenerfv(AL_ORIENTATION, orient);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Unable to set pose. Error code[" <<  error << "]\n";
    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool OpenALSink::SetVelocity(const math::Vector3 &_vel)
{
  ALenum error;

  // Clear error state
  alGetError();

  alListener3f(AL_VELOCITY, _vel.x, _vel.y, _vel.z);
  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Unable to set velocity. Error code[" <<  error << "]\n";
    return false;
  }

  return true;
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
bool OpenALSource::Load(sdf::ElementPtr _sdf)
{
  if (_sdf && _sdf->HasElement("uri"))
    this->FillBufferFromFile(_sdf->Get<std::string>("uri"));
  else
  {
    gzerr << "<audio_source> is missing <uri>...</uri> element\n";
    return false;
  }

  bool result = true;

  if (_sdf->HasElement("pitch"))
    result = result && this->SetPitch(_sdf->Get<double>("pitch"));

  if (_sdf->HasElement("gain"))
    result = result && this->SetGain(_sdf->Get<double>("gain"));

  if (_sdf->HasElement("loop"))
    result = result && this->SetLoop(_sdf->Get<bool>("loop"));

  return result;
}

/////////////////////////////////////////////////
bool OpenALSource::SetPose(const math::Pose &_pose)
{
  ALfloat p[3] = {static_cast<float>(_pose.pos.x),
    static_cast<float>(_pose.pos.y), static_cast<float>(_pose.pos.z)};
  ALenum error;

  // Clear error state
  alGetError();

  alSourcefv(this->alSource, AL_POSITION, p);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Unable to set position. Error code[" << error << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool OpenALSource::SetVelocity(const math::Vector3 &_vel)
{
  ALenum error;
  ALfloat v[3] = {static_cast<float>(_vel.x),
    static_cast<float>(_vel.y), static_cast<float>(_vel.z)};

  // Clear error state
  alGetError();

  alSourcefv(this->alSource, AL_VELOCITY, v);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Unable to set velocity. Error code[" << error << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool OpenALSource::SetPitch(float _pitch)
{
  ALenum error;

  // clear error state
  alGetError();

  alSourcef(this->alSource, AL_PITCH, _pitch);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Unable to set pitch. Error code[" << error << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool OpenALSource::SetGain(float _gain)
{
  ALenum error;

  // clear error state
  alGetError();

  alSourcef(this->alSource, AL_GAIN, _gain);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Unable to set gain. Error code[" << error << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool OpenALSource::SetLoop(bool _state)
{
  ALenum error;

  // clear error state
  alGetError();

  // Set looping state
  alSourcei(this->alSource, AL_LOOPING, _state);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr << " Unable to set loop. Error code[" << error << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void OpenALSource::Play()
{
  int sourceState;
  alGetSourcei(this->alSource, AL_SOURCE_STATE, &sourceState);

  // Play the source, if it's not already playing
  if (sourceState != AL_PLAYING)
    alSourcePlay(this->alSource);
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
    alSourceStop(this->alSource);
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
bool OpenALSource::FillBufferFromPCM(uint8_t *_pcmData,
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
    gzerr << "Unable to copy audio data into openAL buffer.\n";
    return false;
  }

  return true;
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
