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


#include "Controller.hh"
#include "Entity.hh"
#include "OgreAL.h"

namespace gazebo
{
  class AudioIface;
  class OgreAL::SoundManager;

/// \addtogroup gazebo_controller
/// \{
/** \defgroup audio audio

  \brief Audio controller

  Controller for the sounds 

  \verbatim
    <controller:audio name="talking_box">
      <loop>false</loop>
      <stream>false</stream>
      <interface:audio name="audio_iface_0">
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
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// Init the controller
  protected: virtual void InitChild();

  /// \brief Reset the controller
  protected: virtual void ResetChild();

  /// Update the controller
  protected: virtual void UpdateChild(UpdateParams &params);

  /// Finalize the controller
  protected: virtual void FiniChild();

  /// Update the data in the interface
  private: void PutAudioData();

  /// Get the position command from libgazebo
  private: void GetAudioCmd();

  /// The Position interface
  private: AudioIface *myIface;
  
  private: bool loopSound;
  private: bool stream;
  private: int state;
  private: bool cmdPlay;
  private: bool cmdPause;
  private: bool cmdStop;
  private: float gain;
  private: std::string url;
  private: OgreAL::SoundManager *soundManager;


};

/** \} */
/// \}

}

#endif

