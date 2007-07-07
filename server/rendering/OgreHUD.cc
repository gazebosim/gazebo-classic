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
/* Desc: Renders a Heads Up Display
 * Author: Nate Koenig
 * Date: 7 July 2007
 * SVN: $Id$
 */

#include <sstream>

#include "CameraSensor.hh"
#include "OgreAdaptor.hh"
#include "GazeboError.hh"
#include "Global.hh"
#include "Pose3d.hh"
#include "OgreHUD.hh"

using namespace gazebo;

OgreHUD *OgreHUD::myself = NULL;

////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
OgreHUD::OgreHUD()
{
  this->ogreAdaptor = OgreAdaptor::Instance();

  Ogre::Overlay *hudOverlay;

  this->overlayMgr = Ogre::OverlayManager::getSingletonPtr();

  /// This overlay is used to display a 2D Heads Up Display
  hudOverlay = this->overlayMgr->create("__GAZEBO_HUD__");

  this->hudPanel = static_cast<Ogre::OverlayContainer*>(this->overlayMgr->createOverlayElement("Panel", "__GAZEBO_HUD_PANEL_1__"));
  this->hudPanel->setDimensions(1, 1);
  this->hudPanel->setPosition(0, 0);
  hudOverlay->add2D(this->hudPanel);

  // This overlay element is used to display the ouptut from one of the 
  // cameras in the  Gazebo world. It renders the texture output from a 
  // camera to a 2D overlay.
  this->cameraPanel = static_cast<Ogre::OverlayContainer*>(overlayMgr->createOverlayElement("Panel", "__GAZEBO_CAMERA_PANEL_1__"));
  this->cameraPanel->setDimensions(1, 1);
  this->cameraPanel->setPosition(0, 0);
  hudOverlay->add2D(this->cameraPanel);


  // Create the text box used to display the camera's pose
  this->AddTextBox("__GAZEBO_HUD_CAMERA_POSE__", "", 10, 10, 100, 20, Ogre::ColourValue::White);

  this->backgroundPanel = static_cast<Ogre::OverlayContainer*>(overlayMgr->createOverlayElement("Panel", "__GAZEBO_BACKGROUND_PANEL__"));
  this->backgroundPanel->setDimensions(1, 1);
  this->backgroundPanel->setPosition(0, 0);
  this->backgroundPanel->setMaterialName("Gazebo/FlatBlack");
  hudOverlay->add2D(this->backgroundPanel);

  hudOverlay->show();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
OgreHUD::~OgreHUD()
{
}

////////////////////////////////////////////////////////////////////////////////
// Get a pointer to the text renderer
OgreHUD *OgreHUD::Instance()
{
  if (!myself)
    myself = new OgreHUD;

  return myself;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the camera pose text box 
void OgreHUD::SetCamera(const CameraSensor *camera)
{
  if (!camera)
    gzthrow("Camera is NULL");

  // Display the proper camera
  this->cameraPanel->setMaterialName(camera->GetMaterialName());

  float width = (float)camera->GetImageWidth() / this->ogreAdaptor->viewport->getActualWidth();
  float height = (float)camera->GetImageHeight() / this->ogreAdaptor->viewport->getActualHeight();

  float left = (1.0 - width) / 2.0;
  float top = (1.0 - height) / 2.0;

  this->cameraPanel->setDimensions(width, height);
  this->cameraPanel->setPosition(left, top);

  std::ostringstream stream;
  Pose3d pose = camera->GetWorldPose();

  stream.precision(2);
  stream.flags(std::ios::showpoint | std::ios::fixed);
  stream.fill('0');

  stream << camera->GetName() << "\n";
  stream << "\tXYZ [" 
    << pose.pos.x  << " "
    << pose.pos.y << " "
    << pose.pos.z << "]\n";

  stream << "\tRPY [" 
    << RTOD(pose.rot.GetRoll()) << " " 
    << RTOD(pose.rot.GetPitch()) << " "
    << RTOD(pose.rot.GetYaw()) << "]";

  this->SetText("__GAZEBO_HUD_CAMERA_POSE__", stream.str());
}

////////////////////////////////////////////////////////////////////////////////
/// Add a text box
void OgreHUD::AddTextBox( const std::string& id,
                                   const std::string& text,
                                   Ogre::Real x, Ogre::Real y,
                                   Ogre::Real width, Ogre::Real height,
                                   const Ogre::ColourValue& color)
{
  Ogre::OverlayElement* textBox = this->overlayMgr->createOverlayElement("TextArea", id);
  textBox->setDimensions(width, height);
  textBox->setMetricsMode(Ogre::GMM_PIXELS);
  textBox->setPosition(x, y);
  textBox->setWidth(width);
  textBox->setHeight(height);
  textBox->setParameter("font_name", "Arial");
  textBox->setParameter("char_height", "16");
  textBox->setColour(color);

  textBox->setCaption(text);

  this->hudPanel->addChild(textBox);
}

////////////////////////////////////////////////////////////////////////////////
// Hide a text box
void OgreHUD::HideTextBox(const std::string &id)
{
  this->hudPanel->getChild(id)->hide();
}

////////////////////////////////////////////////////////////////////////////////
// Show a text box
void OgreHUD::ShowTextBox(const std::string &id)
{
  this->hudPanel->getChild(id)->show();
}

////////////////////////////////////////////////////////////////////////////////
/// Remove a text box
void OgreHUD::RemoveTextBox(const std::string& ID)
{
  this->hudPanel->removeChild(ID);
  this->overlayMgr->destroyOverlayElement(ID);
}

////////////////////////////////////////////////////////////////////////////////
/// Set text 
void OgreHUD::SetText(const std::string& ID, const std::string& Text)
{
  Ogre::OverlayElement* textBox = this->overlayMgr->getOverlayElement(ID);
  textBox->setCaption(Text);
}
