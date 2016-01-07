/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_UTIL_OPENAL_HH_
#define _GAZEBO_UTIL_OPENAL_HH_

#include <string>
#include <vector>
#include <sdf/sdf.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/util/UtilTypes.hh"

#include "gazebo/gazebo_config.h"
#include "gazebo/util/system.hh"

#ifdef HAVE_OPENAL

namespace gazebo
{
  namespace util
  {
    // Forward declare private openal data class
    class OpenALPrivate;

    // Forward declare private openal sourcedata class
    class OpenALSourcePrivate;

    /// \addtogroup gazebo_util Utility
    /// \{

    /// \class OpenAL OpenAL.hh util/util.hh
    /// \brief 3D audio setup and playback.
    class GZ_UTIL_VISIBLE OpenAL : public SingletonT<OpenAL>
    {
      /// \brief Constructor
      private: OpenAL();

      /// \brief Destructor
      private: virtual ~OpenAL();

      /// \brief Load the OpenAL server.
      /// \return True on success.
      public: bool Load(sdf::ElementPtr _sdf = sdf::ElementPtr());

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

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<OpenALPrivate> dataPtr;

      /// \brief This is a singleton
      private: friend class SingletonT<OpenAL>;
    };

    /// \class OpenALSink OpenALSink.hh util/util.hh
    /// \brief OpenAL Listener. This can be thought of as a microphone.
    class GZ_UTIL_VISIBLE OpenALSink
    {
      /// \brief Constructor
      public: OpenALSink();

      /// \brief Destructor
      public: virtual ~OpenALSink();

      /// \brief Set the position of the sink.
      /// \param[in] _pose New pose of the sink.
      /// \return True on success.
      public: bool SetPose(const ignition::math::Pose3d &_pose);

      /// \brief Set the velocity of the sink
      /// \param[in] _vel Velocity of the sink.
      /// \return True on success.
      public: bool SetVelocity(const ignition::math::Vector3d &_vel);
    };

    /// \class OpenALSource OpenALSource.hh util/util.hh
    /// \brief OpenAL Source. This can be thought of as a speaker.
    class GZ_UTIL_VISIBLE OpenALSource
    {
      /// \brief Constructor.
      public: OpenALSource();

      /// \brief Destructor.
      public: virtual ~OpenALSource();

      /// \brief Load the source from sdf.
      /// \param[in] _sdf SDF element parameters for an audio_source.
      /// \return True on success.
      public: bool Load(sdf::ElementPtr _sdf);

      /// \brief Set the position of the source.
      /// \param[in] _pose New pose of the source.
      /// \return True on success.
      public: bool SetPose(const ignition::math::Pose3d &_pose);

      /// \brief Set the velocity of the source.
      /// \param[in] _vel New velocity of the source.
      /// \return True on success.
      public: bool SetVelocity(const ignition::math::Vector3d &_vel);

      /// \brief Set the pitch of the source.
      /// \param[in] _p Pitch value.
      /// \return True on success.
      public: bool SetPitch(float _p);

      /// \brief Set the pitch of the source.
      /// \param[in] _g Gain value.
      /// \return True on success.
      public: bool SetGain(float _g);

      /// \brief Set whether the source loops the audio.
      /// \param[in] _state True to cause playback to loop.
      /// \return True on success.
      public: bool SetLoop(bool _state);

      /// \brief Return true if the audio source is played on contact with
      /// another object. Contact is determine based on a set of
      /// collision objects.
      /// \return True if audio is played on contact.
      /// \sa AddCollision()
      /// \deprecated See OnContact() const
      public: bool GetOnContact() const GAZEBO_DEPRECATED(7.0);

      /// \brief Return true if the audio source is played on contact with
      /// another object. Contact is determine based on a set of
      /// collision objects.
      /// \return True if audio is played on contact.
      /// \sa AddCollision()
      public: bool OnContact() const;

      /// \brief Get a vector of all the collision names.
      /// \return All the collision names used to trigger audio playback on
      /// contact.
      /// \deprecated See CollisionNames() const
      public: std::vector<std::string> GetCollisionNames() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get a vector of all the collision names.
      /// \return All the collision names used to trigger audio playback on
      /// contact.
      public: std::vector<std::string> CollisionNames() const;

      /// \brief Get whether the source has a collision name set.
      /// \param[in] _name Name of a collision to check for.
      /// \return True if the collision name was found.
      public: bool HasCollisionName(const std::string &_name) const;

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

      /// \brief Fill the OpenAL audio buffer with data from a sound file.
      /// \param[in] _audioFile Name and an audio file.
      public: void FillBufferFromFile(const std::string &_audioFile);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<OpenALSourcePrivate> dataPtr;
    };
    /// \}
  }
}
#endif
#endif
