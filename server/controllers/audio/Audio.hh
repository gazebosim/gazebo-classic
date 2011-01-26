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
/*
 * Desc: Audio controller
 * Author: Jordi Polo
 * Date: 23 Feb 2008
 */
#ifndef AUDIOCONTROLLER_HH
#define AUDIOCONTROLLER_HH

#include "gazebo_config.h"

#include "Controller.hh"
#include "Entity.hh"

#ifdef HAVE_OPENAL

namespace libgazebo
{
  class AudioIface;
}

namespace gazebo
{
  class Model;
  class OpenALSource;

/// \addtogroup gazebo_controller
/// \{
/** \defgroup audio audio

  \brief Audio output controller

  Controller for output of sounds 

  \verbatim
    <controller:audio name="audio_1">
      <pitch>1</pitch>
      <gain>1</gain>
      <loop>false</loop>
      <interface:audio name="audio_iface_0"/>
    </controller:audio>
  \endverbatim
  \{
*/

/// \brief Audio controller
class AudioController : public Controller
{
  /// Constructor
  public: AudioController(Entity *parent );

  /// Destructor
  public: virtual ~AudioController();

  /// Load the controller
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// Init the controller
  protected: virtual void InitChild();

  /// \brief Reset the controller
  protected: virtual void ResetChild();

  /// Update the controller
  protected: virtual void UpdateChild();

  /// Finalize the controller
  protected: virtual void FiniChild();

  /// Update the data in the interface
  private: void PutAudioData();

  /// Get the position command from libgazebo
  private: void GetAudioCmd();

  /// The Position interface
  private: libgazebo::AudioIface *audioIface;
  
  private: OpenALSource *openALSource;

  private: Model *myParent;
};

/** \} */
/// \}

}

#endif
#endif

