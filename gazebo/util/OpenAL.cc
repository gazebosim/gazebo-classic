/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <stdio.h>
#include <unistd.h>
#include <iostream>

#include <gazebo/gazebo_config.h>

#ifdef HAVE_OPENAL
#ifdef __APPLE__
# include <OpenAL/al.h>
# include <OpenAL/alc.h>
# include <OpenAL/MacOSX_OALExtensions.h>
#else
# include <AL/al.h>
# include <AL/alc.h>
# include <AL/alext.h>
#endif
#endif

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/AudioDecoder.hh"
#include "gazebo/util/OpenAL.hh"

using namespace gazebo;
using namespace util;

#ifdef HAVE_OPENAL
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
bool OpenAL::Load(sdf::ElementPtr _sdf)
{
  std::string deviceName = "default";

  if (_sdf && _sdf->HasElement("device"))
    deviceName = _sdf->Get<std::string>("device");

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
  ALfloat orient[] = {static_cast<ALfloat>(rot[0][0]),
                      static_cast<ALfloat>(rot[0][1]),
                      static_cast<ALfloat>(rot[0][2]),
                      static_cast<ALfloat>(rot[2][0]),
                      static_cast<ALfloat>(rot[2][1]),
                      static_cast<ALfloat>(rot[2][2])};

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

  if (_sdf->HasElement("contact"))
  {
    sdf::ElementPtr collisionElem =
      _sdf->GetElement("contact")->GetElement("collision");

    while (collisionElem)
    {
      this->collisionNames.push_back(collisionElem->Get<std::string>());
      collisionElem = collisionElem->GetNextElement("collision");
    }

    result = result && !this->collisionNames.empty();
  }
  else
    this->Play();

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
std::vector<std::string> OpenALSource::GetCollisionNames() const
{
  return this->collisionNames;
}

/////////////////////////////////////////////////
bool OpenALSource::GetOnContact() const
{
  return !this->collisionNames.empty();
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

/////////////////////////////////////////////////
bool OpenALSource::HasCollisionName(const std::string &_name) const
{
  for (std::vector<std::string>::const_iterator iter =
      this->collisionNames.begin(); iter != this->collisionNames.end(); ++iter)
  {
    if (*iter  == _name)
      return true;
  }

  return false;
}
#endif
