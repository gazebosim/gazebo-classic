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
