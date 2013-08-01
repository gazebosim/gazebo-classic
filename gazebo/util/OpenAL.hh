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
#ifndef _GAZEBO_OPENAL_HH_
#define _GAZEBO_OPENAL_HH_

#include <string>
#include <sdf/sdf.hh>

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/util/UtilTypes.hh"

#include "gazebo/gazebo_config.h"

#ifdef HAVE_OPENAL

struct ALCcontext_struct;
struct ALCdevice_struct;

namespace gazebo
{
  namespace util
  {
    class OpenALIface;
    class OpenALSource;
    class OpenALSink;

    class OpenAL :public SingletonT<OpenAL>
    {
      /// \brief Constructor
      private: OpenAL();

      /// \brief Destructor
      private: virtual ~OpenAL();

      /// \brief Load the OpenAL server.
      /// \return True on success.
      public: bool Load();

      /// \brief Finalize.
      public: void Fini();

      /// \brief Create an OpenALSource object.
      /// \param[in] _sdf SDF element parameters for an audio_source.
      /// \return A pointer to an OpenALSource object.
      public: OpenALSourcePtr CreateSource(sdf::ElementPtr _sdf);

      /// \brief Create an audio listener. Currenly, only one listener may be
      /// created.
      /// \param[in] _sdf SDF element parameters for an audio_source.
      /// \return A pointer to an OpenALSink object.
      public: OpenALSinkPtr CreateSink(sdf::ElementPtr _sdf);

      /// \brief OpenAL audio context pointer.
      private: ALCcontext_struct *context;

      /// \brief OpenAL audio device pointer.
      private: ALCdevice_struct *audioDevice;

      /// \brief OpenAL sink pointer.
      private: OpenALSinkPtr sink;

      /// \brief This is a singleton
      private: friend class SingletonT<OpenAL>;
    };

    /// \brief OpenAL Listener. This can be thought of as a microphone.
    class OpenALSink
    {
      /// \brief Constructor
      public: OpenALSink();

      /// \brief Destructor
      public: virtual ~OpenALSink();

      /// \brief Set the position of the sink.
      /// \param[in] _pose New pose of the sink.
      /// \return True on success.
      public: bool SetPose(const math::Pose &_pose);

      /// \brief Set the velocity of the sink
      /// \param[in] _vel Velocity of the sink.
      /// \return True on success.
      public: bool SetVelocity(const math::Vector3 &_vel);
    };

    /// \brief OpenAL Source. This can be thought of as a speaker.
    class OpenALSource
    {
      /// \brief Constructor
      public: OpenALSource();

      /// \brief Destructor
      public: virtual ~OpenALSource();

      /// \brief Load the source from sdf.
      /// \param[in] _sdf SDF element parameters for an audio_source.
      public: bool Load(sdf::ElementPtr _sdf);

      /// \brief Set the position of the source.
      /// \param[in] _pose New pose of the source.
      /// \return True on success.
      public: bool SetPose(const math::Pose &_pose);

      /// \brief Set the velocity of the source
      /// \param[in] _vel New velocity of the source
      /// \return True on success.
      public: bool SetVelocity(const math::Vector3 &_vel);

      /// \brief Set the pitch of the source
      /// \param[in] _p Pitch value.
      /// \return True on success.
      public: bool SetPitch(float _p);

      /// \brief Set the pitch of the source
      /// \param[in] _g Gain value.
      /// \return True on success.
      public: bool SetGain(float _g);

      /// \brief Set whether the source loops the audio
      /// \param[in] _state True to cause playback to loop.
      /// \return True on success.
      public: bool SetLoop(bool _state);

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
      /// \param[in] _pcmData Pointer to the PCM audio data.
      /// \param[in] _dataCount Size of the PCM data.
      /// \param[in] _sampleRate Sample rate for the PCM data.
      /// \return True on success.
      public: bool FillBufferFromPCM(uint8_t *_pcmData, unsigned int _dataCount,
                                     int _sampleRate);

      /// \brief Fill the OpenAL audio buffer with data from a sound file
      public: void FillBufferFromFile(const std::string &_audioFile);

      // OpenAL source index
      private: unsigned int alSource;

      // OpenAL buffer index
      private: unsigned int alBuffer;
    };
  }
}

#endif
#endif
