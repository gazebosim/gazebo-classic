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

