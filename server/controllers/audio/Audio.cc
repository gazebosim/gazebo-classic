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
 * Desc: Audio Output controller 
 * Author: Nate Koenig
 * Date: 23 Jan 2009
 */

#include "Global.hh"
#include "World.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "Simulator.hh"
#include "gz.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "OpenAL.hh"
#include "Audio.hh"

#include <string.h>

#ifdef HAVE_OPENAL

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("audio", AudioController);

////////////////////////////////////////////////////////////////////////////////
// Constructor
AudioController::AudioController(Entity *parent )
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("AudioController controller requires a geom as its parent");

  this->audioIface = NULL;
  this->openALSource = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
AudioController::~AudioController()
{
  if (this->openALSource)
    delete this->openALSource;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void AudioController::LoadChild(XMLConfigNode *node)
{
  this->audioIface = dynamic_cast<libgazebo::AudioIface*>(this->GetIface("audio"));
  this->openALSource = OpenAL::Instance()->CreateSource( node );

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void AudioController::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void AudioController::ResetChild()
{
  this->openALSource->Rewind();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void AudioController::UpdateChild()
{
  // Update the position of the audio source, if it exists
  if (this->openALSource)
  {
    this->GetAudioCmd();

    this->PutAudioData();

    this->openALSource->SetPos(this->myParent->GetWorldPose().pos);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void AudioController::FiniChild()
{
  delete this->openALSource;
  this->openALSource = NULL;
}

//////////////////////////////////////////////////////////////////////////////
// Get commands from the external interface
void AudioController::GetAudioCmd()
{

  if (this->audioIface->Lock(1))
  {
    // Control output of the audio
    if (this->audioIface->data->cmd_play)
      this->openALSource->Play();
    else if (this->audioIface->data->cmd_pause)
      this->openALSource->Pause();
    else if (this->audioIface->data->cmd_stop)
      this->openALSource->Stop();
    else if (this->audioIface->data->cmd_rewind)
      this->openALSource->Rewind();

    // Loop the audio
    if (this->audioIface->data->cmd_loop)
      this->openALSource->SetLoop(1);

    // Set the gain and pitch
    if (this->audioIface->data->cmd_gain > 0)
      this->openALSource->SetGain( this->audioIface->data->cmd_gain );
    if (this->audioIface->data->cmd_pitch > 0)
      this->openALSource->SetPitch(this->audioIface->data->cmd_pitch );
 
    // Reset the cmd variables
    this->audioIface->data->cmd_play=0;
    this->audioIface->data->cmd_pause=0;
    this->audioIface->data->cmd_stop=0;
    this->audioIface->data->cmd_rewind=0;
    this->audioIface->data->cmd_loop=0;

    this->audioIface->data->cmd_pitch = 0;
    this->audioIface->data->cmd_gain = 0;
 
    this->audioIface->Unlock();
  }

}

//////////////////////////////////////////////////////////////////////////////
// Update the data in the interface
void AudioController::PutAudioData()
{
  if (this->audioIface->Lock(1))
  {
    this->audioIface->data->head.time = this->myParent->GetWorld()->GetSimTime().Double();
    this->audioIface->data->state = this->openALSource->IsPlaying();
    this->audioIface->Unlock();
  }
}

#endif
