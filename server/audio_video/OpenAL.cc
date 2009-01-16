#include <stdio.h>
#include <unistd.h>
#include <iostream>

#include <AL/alut.h>
#include <AL/alc.h>

#include "AudioDecoder.hh"
#include "OpenAL.hh"

#ifndef NULL
#define NULL 0
#endif


////////////////////////////////////////////////////////////////////////////////
///// Constructor
OpenAL::OpenAL()
{
  this->context = NULL;
  this->audioDevice = NULL;

  this->pos[0] = this->pos[1] = this->pos[2] = 0.0;
  this->pos[0] = -10.0;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
OpenAL::~OpenAL()
{
  if (this->context && this->audioDevice)
  {
    this->context = alcGetCurrentContext();
    this->audioDevice = alcGetContextsDevice(this->context);
    alcMakeContextCurrent(NULL);
    alcDestroyContext(this->context);
    alcCloseDevice(this->audioDevice);
  }

  this->sources.clear();
  this->buffers.clear();
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize
int OpenAL::Init()
{
  // Open the default audio device
  this->audioDevice = alcOpenDevice("ALSA Software on HDA Intel");
  if (this->audioDevice == NULL)
  {
    printf("Unable to open audio device\n");
    return -1;
  }

  this->context = alcCreateContext(this->audioDevice, NULL);

  alcMakeContextCurrent(this->context);

  //Clear error code
  alGetError();

  // TODO: put in function to set distance model
  //alDistanceModel(AL_EXPONENT_DISTANCE);
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Create a sound source
unsigned int OpenAL::CreateSource()
{
  unsigned int source;

  //Create 1 source
  alGenSources(1, &source);

  this->sources.push_back(source);

  return this->sources.size()-1;
}

////////////////////////////////////////////////////////////////////////////////
/// Create an audio data buffer
unsigned int OpenAL::CreateBuffer(const std::string &audioFile)
{
  unsigned int buffer;
  uint8_t *dataBuffer = NULL;
  unsigned int dataBufferSize;

  // Create an audio decoder
  AudioDecoder audioDecoder;

  // Create and openAL audio buffer
  alGenBuffers(1, &buffer);

  // Store the openAL buffer
  this->buffers.push_back(buffer);

  // Set the audio file to decode
  audioDecoder.SetFile(audioFile);
  audioDecoder.Decode(&dataBuffer, &dataBufferSize);

  // Fill the openAL data buffer
  this->SetData(buffer, dataBuffer, dataBufferSize, 
                audioDecoder.GetSampleRate() );

  if (dataBuffer)
    delete [] dataBuffer; 

  return this->buffers.size()-1;
}


////////////////////////////////////////////////////////////////////////////////
/// Update all the sources and the listener
void OpenAL::Update()
{
  //ALfloat pos[3];
  ALfloat vel[3];

  //for (int i=0; i< this->sourceCount; i++)
  {

    //alGetSourcefv(0, AL_VELOCITY, vel);
    //alGetSourcefv(0, AL_POSITION, pos);

    vel[0] = 0.1;
    vel[1] = 0.0;
    vel[2] = 0.0;

    this->pos[0] += 0.01;
    this->pos[1] = 0;
    this->pos[2] = 0;

    //printf("p[%f %f %f] v[%f %f %f]\n", pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]);
    this->SetSourcePos(0, this->pos[0], this->pos[1], this->pos[2] );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the data
int OpenAL::SetData(unsigned int index, uint8_t *data, unsigned int dataSize, unsigned int freq)
{
  // Clear the error buffer;
  alGetError();

  printf("Set Data Freq[%d] DataSize[%d]\n",freq, dataSize);

  // Copy raw buffer into AL buffer 0
  // AL_FORMAT_MONO8, AL_FORMAT_MONO16, AL_FORMAT_STEREO8,
  // AL_FORMAT_STEREO16
  alBufferData( this->buffers[0], AL_FORMAT_MONO16, data, dataSize, freq);

  if ( alGetError() != AL_NO_ERROR)
  {
    printf("Unable to copy data into openAL buffer\n");

    alDeleteBuffers(1, &this->buffers[index]);
    return -1;
  }


  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Attach a buffer to a source
int OpenAL::SetSourceBuffer(unsigned int sourceIndex, unsigned int bufferIndex)
{
  if (sourceIndex >= this->sources.size())
  {
    std::cerr << "Invalid source index\n";
    return -1;
  }

  if (bufferIndex >= this->buffers.size())
  {
    std::cerr << "Invalid buffer index\n";
    return -1;
  }

  // Buffer data must be set before calling this function
  
  // Attach buffer to source
  alSourcei(this->sources[sourceIndex], AL_BUFFER, this->buffers[bufferIndex] );

  if ((this->error = alGetError()) != AL_NO_ERROR)
  {
    std::cerr << "OpenAL SetSourceBuffer Error: [%d]\n" << this->error << "\n";
    return -1;
  }


  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Play the source
void OpenAL::Play( unsigned int index )
{
  // Play the source
  alSourcePlay( this->sources[index] );


  // Wait until finished
  /*ALint state;
  do
  {
    usleep(100000);
    alGetSourcei(this->sources[index], AL_SOURCE_STATE, &state);
  } while ( state == AL_PLAYING);
  */

}

////////////////////////////////////////////////////////////////////////////////
/// Set the listener position
int OpenAL::SetListenerPos( float x, float y, float z )
{
  // Clear error state
  alGetError();

  alListener3f(AL_POSITION, x, y, z);

  if ((this->error = alGetError()) != AL_NO_ERROR)
  {
    std::cerr << "OpenAL SetListenerPos Error: [%d]\n" << this->error << "\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the listener velocity
int OpenAL::SetListenerVel( float x, float y, float z )
{
  // Clear error state
  alGetError();

  alListener3f(AL_VELOCITY, x, y, z);
  if ((this->error = alGetError()) != AL_NO_ERROR)
  {
    std::cerr << "OpenAL SetListenerVel Error: [%d]" << this->error << "\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the listener orientation
int OpenAL::SetListenerOrient( float cx, float cy, float cz,
                               float ux, float uy, float uz )
{
  ALfloat orient[]={cx, cy, cz, ux, uy, uz};

  // Clear error state
  alGetError();

  alListenerfv( AL_ORIENTATION, orient );

  if ((this->error = alGetError()) != AL_NO_ERROR)
  {
    std::cerr << "OpenAL SetListenerOrientation Error: [%d]" << this->error << "\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set the position of the source
int OpenAL::SetSourcePos(unsigned int index, float x, float y, float z)
{
  ALfloat p[3] = {x, y, z};

  if (index >= this->sources.size())
  {
    std::cerr << "Invalid source index[" << index <<" ]\n";
    return -1;
  }

  // Clear error state
  alGetError();

  alSourcefv( this->sources[index], AL_POSITION, p);

  if ((this->error = alGetError()) != AL_NO_ERROR)
  {
    std::cerr << "OpenAL::SetSourcePos Error: [%d]" << this->error << "\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set the position of the source
int OpenAL::SetSourceVel(unsigned int index, float x, float y, float z)
{
  ALfloat v[3] = {x, y, z};

  if (index >= this->sources.size())
  {
    std::cerr << "Invalid source index[" << index <<" ]\n";
    return -1;
  }

  // Clear error state
  alGetError();

  alSourcefv( this->sources[index], AL_VELOCITY, v);

  if ((this->error = alGetError()) != AL_NO_ERROR)
  {
    std::cerr << "OpenAL::SetSourceVel Error: [%d]" << this->error << "\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set the pitch of the source
int OpenAL::SetSourcePitch(unsigned int index, float p)
{
  if (index >= this->sources.size())
  {
    std::cerr << "Invalid source index[" << index <<" ]\n";
    return -1;
  }

  // clear error state
  alGetError();

  alSourcef(this->sources[index], AL_PITCH, p);

  if ((this->error = alGetError()) != AL_NO_ERROR)
  {
    std::cerr << "OpenAL::SetSourcePitch Error: [%d]\n" << this->error;
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set the pitch of the source
int OpenAL::SetSourceGain(unsigned int index, float g)
{
  if (index >= this->sources.size())
  {
    std::cerr << "Invalid source index[" << index <<" ]\n";
    return -1;
  }

  // clear error state
  alGetError();

  alSourcef(this->sources[index], AL_GAIN, g);

  if ((this->error = alGetError()) != AL_NO_ERROR)
  {
    std::cerr << "OpenAL::SetSourceGain Error: [%d]\n" << this->error;
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set whether the source loops the audio 
int OpenAL::SetSourceLoop(unsigned int index, bool state)
{
  if (index >= this->sources.size())
  {
    std::cerr << "Invalid source index[" << index <<" ]\n";
    return -1;
  }

  // clear error state
  alGetError();

  // Set looping state 
  alSourcei(this->sources[index], AL_LOOPING, state);

  if ((this->error = alGetError()) != AL_NO_ERROR)
  {
    std::cerr << "OpenAL::SetSourceLoop Error: [%d]\n" << this->error << "\n";
    return -1;
  }


  return 0;
}
