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
/* Desc: OpenAL Server
 * Author: Nathan Koenig
 * Date: 20 Jan 2008
 * SVN: $Id:$
 */

#include "config.h"

#ifdef HAVE_OPENAL

#include <stdio.h>
#include <unistd.h>
#include <iostream>


#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

#include "XMLConfig.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "AudioDecoder.hh"
#include "GazeboConfig.hh"
#include "Simulator.hh"
#include "OpenAL.hh"

using namespace gazebo;

OpenAL *OpenAL::myself = NULL;

////////////////////////////////////////////////////////////////////////////////
///// Constructor
OpenAL::OpenAL()
{
  this->context = NULL;
  this->audioDevice = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
OpenAL::~OpenAL()
{
  this->Fini();
}

////////////////////////////////////////////////////////////////////////////////
// Instance
OpenAL *OpenAL::Instance()
{
  if (!myself)
    myself = new OpenAL();

  return myself;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
void OpenAL::Load(XMLConfigNode *node)
{
  std::string deviceName = "default";

  // Get the audio device name
  if (node)
  {
    deviceName = node->GetString("device", "default", 0);
  }

  // Open the default audio device
  if (deviceName == "default")
    this->audioDevice = alcOpenDevice(NULL);
  else
    this->audioDevice = alcOpenDevice(deviceName.c_str());

  // Make sure that we could open the audio device
  if (this->audioDevice == NULL)
  {
    gzerr(0) << "Unable to open audio device[" << deviceName << "]\n Audio will be disabled.\n";
    return;
  }

  // Create the audio context
  this->context = alcCreateContext(this->audioDevice, NULL);

  if (this->context == NULL)
  {
    gzerr(0) << "Unable to create OpenAL Context.\nAudio will be disabled.\n";
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
/// Initialize
void OpenAL::Init()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize
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
// Create an openal source from XML config node
OpenALSource *OpenAL::CreateSource( XMLConfigNode *node )
{
  // Make sure the xml node is valid
  if (!node)
  {
    gzerr(0) << "Invalid xmlconfig node\n";
    return NULL;
  }

  // Make sure the audio device has been opened
  if (!this->audioDevice)
  {
    gzerr(0) << "Audio device not open\n";
    return NULL;
  }

  // Create a source
  OpenALSource *source = new OpenALSource();

  // Load the source
  source->Load(node);

  // Return a pointer to the source
  return source;
}


////////////////////////////////////////////////////////////////////////////////
/// Set the listener position
void OpenAL::SetListenerPos( const Vector3 pos )
{
  ALenum error;

  // Make sure we have an audio device
  if (!this->audioDevice)
  {
    gzerr(0) << "Audio disabled\n";
    return;
  }

  // Clear error state
  alGetError();

  alListener3f(AL_POSITION, pos.x, pos.y, pos.z);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr(0) << " Error: [" << error << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the listener velocity
void OpenAL::SetListenerVel( const Vector3 vel )
{
  ALenum error;

  // Make sure we have an audio device
  if (!this->audioDevice)
  {
    gzerr(0) << "Audio disabled\n";
  }

  // Clear error state
  alGetError();

  alListener3f(AL_VELOCITY, vel.x, vel.y, vel.z);
  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr(0) << " Error: [" <<  error << "]\n";
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Set the listener orientation
void OpenAL::SetListenerOrient( float cx, float cy, float cz,
                               float ux, float uy, float uz )
{
  ALenum error;
  ALfloat orient[]={cx, cy, cz, ux, uy, uz};

  // Make sure we have an audio device
  if (!this->audioDevice)
  {
    gzerr(0) << "Audio disabled\n";
  }

  // Clear error state
  alGetError();

  alListenerfv( AL_ORIENTATION, orient );

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr(0) << " Error: [" << error << "]\n";
  }

}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//                      OPENAL SOURCE
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Constructor
OpenALSource::OpenALSource()
{
  //Create 1 source
  alGenSources(1, &this->alSource);

  // Create 1 buffer
  alGenBuffers(1, &this->alBuffer);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
OpenALSource::~OpenALSource()
{
  alDeleteSources(1, &this->alSource);
  alDeleteBuffers(1, &this->alBuffer);
}

////////////////////////////////////////////////////////////////////////////////
// Load from xml node
void OpenALSource::Load(XMLConfigNode *node)
{
  // Set the pitch of the source
  this->SetPitch( node->GetDouble("pitch",1.0,0) );

  // Set the gain of the source
  this->SetGain( node->GetDouble("gain",1.0,0) );

  // Set whether the source should loop when played 
  this->SetLoop( node->GetBool("loop",false,0) );

  if (node->GetChild("mp3") != NULL)
    this->FillBufferFromFile(node->GetString("mp3","",1));
}

////////////////////////////////////////////////////////////////////////////////
// Set the position of the source
int OpenALSource::SetPos(const Vector3 &pos)
{
  ALfloat p[3] = {pos.x, pos.y, pos.z};
  ALenum error;

  // Clear error state
  alGetError();

  alSourcefv( this->alSource, AL_POSITION, p);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr(0) << "Error: [" << error << "]\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set the position of the source
int OpenALSource::SetVel(const Vector3 &vel)
{
  ALenum error;
  ALfloat v[3] = {vel.x, vel.y, vel.z};

  // Clear error state
  alGetError();

  alSourcefv( this->alSource, AL_VELOCITY, v);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr(0) << "Error: [" << error << "]\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set the pitch of the source
int OpenALSource::SetPitch(float p)
{
  ALenum error;

  // clear error state
  alGetError();

  alSourcef(this->alSource, AL_PITCH, p);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr(0) << " Error: [" << error << "]\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set the pitch of the source
int OpenALSource::SetGain(float g)
{

  ALenum error;

  // clear error state
  alGetError();

  alSourcef(this->alSource, AL_GAIN, g);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr(0) << "Error: [" << error << "]\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set whether the source loops the audio 
int OpenALSource::SetLoop(bool state)
{
  ALenum error;

  // clear error state
  alGetError();

  // Set looping state 
  alSourcei(this->alSource, AL_LOOPING, state);

  if ((error = alGetError()) != AL_NO_ERROR)
  {
    gzerr(0) << " Error: [" << error << "]\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Play the source
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
// Pause the source
void OpenALSource::Pause()
{
  int sourceState;
  alGetSourcei(this->alSource, AL_SOURCE_STATE, &sourceState);

  // Pause the source if it playing
  if (sourceState == AL_PLAYING)
    alSourcePause( this->alSource );
}

////////////////////////////////////////////////////////////////////////////////
// Stop the source
void OpenALSource::Stop()
{
  int sourceState;
  alGetSourcei(this->alSource, AL_SOURCE_STATE, &sourceState);

  // Stop the source if it is not already stopped
  if (sourceState != AL_STOPPED)
    alSourcePause( this->alSource);
}

////////////////////////////////////////////////////////////////////////////////
/// Rewind the sound to the beginning
void OpenALSource::Rewind()
{
  alSourceRewind(this->alSource);
}

////////////////////////////////////////////////////////////////////////////////
/// Is the audio playing
bool OpenALSource::IsPlaying()
{
  int sourceState;
  alGetSourcei(this->alSource, AL_SOURCE_STATE, &sourceState);

  return sourceState == AL_PLAYING;
}


////////////////////////////////////////////////////////////////////////////////
// Fill the audio buffer from PCM data
void OpenALSource::FillBufferFromPCM(uint8_t *pcmData, unsigned int dataCount, 
                                     int sampleRate )
{
  // First detach the buffer
  alSourcei(this->alSource, AL_BUFFER, 0 );

  // Copy raw buffer into AL buffer
  // AL_FORMAT_MONO8, AL_FORMAT_MONO16, AL_FORMAT_STEREO8,
  // AL_FORMAT_STEREO16
  alBufferData( this->alBuffer, AL_FORMAT_MONO16, pcmData, dataCount, 
                sampleRate);

  // Attach buffer to source
  alSourcei(this->alSource, AL_BUFFER, this->alBuffer );

  if ( alGetError() != AL_NO_ERROR)
  {
    gzthrow("Unable to copy data into openAL buffer\n");
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Fill the OpenAL audio buffer with data from a sound file
void OpenALSource::FillBufferFromFile( const std::string &audioFile )
{

  std::string fullPathAudioFile = audioFile;

  // Try to open the audio file in the current directory
  FILE *testFile = fopen(fullPathAudioFile.c_str(), "r");

  // If the audio file couldn't be opened, try the gazebo paths
  if (testFile == NULL)
  {
    std::list<std::string>::iterator iter;
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
    }
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
