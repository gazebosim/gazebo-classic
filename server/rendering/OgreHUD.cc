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
#include <Ogre.h>

#include "World.hh"
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

  if (!this->ogreAdaptor->window)
    return;

  Ogre::Overlay *hudOverlay;

  this->overlayMgr = Ogre::OverlayManager::getSingletonPtr();

  /// This overlay is used to display a 2D Heads Up Display
  hudOverlay = this->overlayMgr->create("__GAZEBO_HUD__");

  this->hudPanel = static_cast<Ogre::OverlayContainer*>(this->overlayMgr->createOverlayElement("Panel", "__GAZEBO_HUD_PANEL_1__"));
  this->hudPanel->setDimensions(1, 1);
  this->hudPanel->setPosition(0, 0);
  this->hudPanel->setMetricsMode(Ogre::GMM_RELATIVE);
  hudOverlay->add2D(this->hudPanel);

  // This overlay element is used to display the ouptut from one of the 
  // cameras in the  Gazebo world. It renders the texture output from a 
  // camera to a 2D overlay.
  this->cameraPanel = static_cast<Ogre::PanelOverlayElement*>(overlayMgr->createOverlayElement("Panel", "__GAZEBO_CAMERA_PANEL_1__"));
  this->cameraPanel->setDimensions(1, 1);
  this->cameraPanel->setPosition(0, 0);
  hudOverlay->add2D(this->cameraPanel);

/*  this->helpPanel = static_cast<Ogre::OverlayContainer*>(overlayMgr->createOverlayElement("Panel", "__GAZEBO_HELP_PANEL_1__"));
  this->helpPanel->setDimensions(1, 1);
  this->helpPanel->setPosition(0, 0);
  this->helpPanel->hide();
  hudOverlay->add2D(this->helpPanel);

  this->CreateHelp();
  this->CreateTextBoxes();
  */

  /*this->backgroundPanel = static_cast<Ogre::OverlayContainer*>(overlayMgr->createOverlayElement("Panel", "__GAZEBO_BACKGROUND_PANEL__"));
  this->backgroundPanel->setDimensions(1, 1);
  this->backgroundPanel->setPosition(0, 0);
  this->backgroundPanel->setMaterialName("Gazebo/FlatBlack");
  hudOverlay->add2D(this->backgroundPanel);
  */

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
// Get a pointer to the text renderer
void OgreHUD::Close()
{
  GZ_DELETE(myself)
}

////////////////////////////////////////////////////////////////////////////////
/// Update the HUD.
void OgreHUD::Update()
{
/*  std::ostringstream stream;
  if (Global::GetUserPause())
  {
    this->SetText("__GAZEBO_HUD_PAUSE__","Paused");
  }
  else if (Global::GetUserStep())
  {
    this->SetText("__GAZEBO_HUD_PAUSE__","Step");
  }
  else
  {
    this->SetText("__GAZEBO_HUD_PAUSE__","");
  }

  float lastFPS, avgFPS, bestFPS, worstFPS;
  this->ogreAdaptor->window->getStatistics(lastFPS, avgFPS, bestFPS, worstFPS);

  stream.precision(2);
  stream.flags(std::ios::showpoint | std::ios::fixed);
  stream.fill('0');

  stream << "FPS        [" << (int)avgFPS << "]\n"
         << "Iterations [" << Global::GetIterations() << "]\n"
         << "Time       [" << World::Instance()->GetRealTime() << " " 
         << World::Instance()->GetSimTime() << " " 
         << World::Instance()->GetPauseTime() << "]";


  this->SetText("__GAZEBO_HUD_STATS__",stream.str());

  stream.str("");
  */

}

////////////////////////////////////////////////////////////////////////////////
/// Set the camera pose text box 
void OgreHUD::SetCamera(const CameraSensor *camera)
{
  if (!camera)
    gzthrow("Camera is NULL");

  // Display the proper camera
  this->cameraPanel->setMaterialName(camera->GetMaterialName());

  // TODO: Fix this. It current displays the whole render texture, which is
  // not the actual size of the camera image
  float width = (float)camera->GetTextureWidth() / this->ogreAdaptor->viewport->getActualWidth();
  float height = (float)camera->GetTextureHeight() /this->ogreAdaptor->viewport->getActualHeight();

  float left = (1.0 - width) / 2.0;
  float top = (1.0 - height) / 2.0;
  this->cameraPanel->setDimensions(width, height);
  this->cameraPanel->setPosition(left,top);

  /*float guiWidth = this->ogreAdaptor->viewport->getActualWidth();
  float guiHeight = this->ogreAdaptor->viewport->getActualHeight();

  float width = (float)camera->GetImageWidth() / camera->GetTextureWidth();
  float height = (float)camera->GetImageHeight() / camera->GetTextureHeight();

  float guiXScale = guiWidth / camera->GetImageWidth();
  float guiYScale = guiHeight / camera->GetImageHeight();

  printf("Viewport Size[%d %d]\n",this->ogreAdaptor->viewport->getActualWidth(), this->ogreAdaptor->viewport->getActualHeight());

  printf("Texture Size[%d %d] [%4.4f %4.4f]\n",camera->GetTextureWidth(), camera->GetTextureHeight(), width, height);

  printf("Image Size[%d %d]\n",camera->GetImageWidth(), camera->GetImageHeight());

  printf("Gui Size[%f %f] Scale[%f %f]\n",guiWidth, guiHeight, guiXScale, guiYScale);

  printf("GUI Ratio[%f] Texture Ratio[%f]\n", guiWidth/guiHeight, width/height);
  printf("\n");

  this->cameraPanel->setUV((1.0-width)/2.0,(1.0-height)/2.0,width, height);
  float left = (1.0 - guiXScale) / 2.0;
  float top = (1.0 - guiYScale) / 2.0;

  //this->cameraPanel->setDimensions(1.0, height*(1.0/width));
  //this->cameraPanel->setPosition(0,(1-height*(1.0/width)) / 2.0);
  */

  /*std::ostringstream stream;
  Pose3d pose = camera->GetWorldPose();

  stream.precision(2);
  stream.flags(std::ios::showpoint | std::ios::fixed);
  stream.fill('0');

  stream << camera->GetName() << " [" << camera->GetImageWidth() << " x " << camera->GetImageHeight() << "]\n";
 
  pose -= Global::poseOffset;

  stream << "  XYZ [" 
    << pose.pos.x   << " "
    << pose.pos.y << " "
    << pose.pos.z << "]\n";

  stream << "  RPY [" 
    << RTOD(pose.rot.GetRoll()) << " " 
    << RTOD(pose.rot.GetPitch()) << " "
    << RTOD(pose.rot.GetYaw()) << "]";

  this->SetText("__GAZEBO_HUD_CAMERA_POSE__", stream.str());
  */
}

////////////////////////////////////////////////////////////////////////////////
/// Add a text box
void OgreHUD::AddTextBox( const std::string& id,
                          const std::string &panelId,
                          const std::string& text,
                          Ogre::Real x, Ogre::Real y,
                          Ogre::Real width, Ogre::Real height,
                          const Ogre::ColourValue& color
                          )
{
  Ogre::OverlayElement* textBox = this->overlayMgr->createOverlayElement("TextArea", id);
  textBox->setMetricsMode(Ogre::GMM_RELATIVE);
  textBox->setHorizontalAlignment(Ogre::GHA_LEFT);
  textBox->setVerticalAlignment(Ogre::GVA_TOP);
  textBox->setDimensions(width, height);
  textBox->setPosition(x, y);
  textBox->setParameter("font_name", "Console");
  textBox->setParameter("char_height", "0.03");
  textBox->setColour(color);

  textBox->setCaption(text);

  this->overlayMgr->getByName("__GAZEBO_HUD__")->getChild(panelId)->addChild(textBox);
  //panel->addChild(textBox);
}

////////////////////////////////////////////////////////////////////////////////
/// Toggle the visibility of the HUD
void OgreHUD::ToggleVisible()
{
  if (this->hudPanel->isVisible())
    this->hudPanel->hide();
  else
    this->hudPanel->show();

}

////////////////////////////////////////////////////////////////////////////////
// Toggle display of the help
void OgreHUD::ToggleHelp()
{
  /*if (this->helpPanel->isVisible())
  {
    this->helpPanel->hide();
    Global::SetUserPause(false);
  }
  else
  {
    Global::SetUserPause(true);
    this->helpPanel->show();
  }*/
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

////////////////////////////////////////////////////////////////////////////////
/// Helper function to create the HUD text boxes
void OgreHUD::CreateTextBoxes()
{
  float left, top;
  float width, height;

  //
  // Camera Pose text box
  //
  left = 0.015;
  top = 0.015;
  width = 0.15;
  height = 0.08;

  this->AddTextBox("__GAZEBO_HUD_CAMERA_POSE__","__GAZEBO_HUD_PANEL_1__", "", left, top, width, height, Ogre::ColourValue::White);

  //
  // Pause/step text box
  //
  left = 0.9;
  top = 0.97;
  width = 0.08;
  height = 0.03;

  this->AddTextBox("__GAZEBO_HUD_PAUSE__","__GAZEBO_HUD_PANEL_1__","",left,top,width,height, Ogre::ColourValue(1.0, 0.0, 0.0, 1.0));

  //
  // Iterations text box
  //
  left = 0.015;
  top = 0.90;
  width = 0.15;
  height = 0.03;
  this->AddTextBox("__GAZEBO_HUD_STATS__","__GAZEBO_HUD_PANEL_1__","",left,top,width,height,Ogre::ColourValue::White);
/*
  //
  // FPS text box
  //
  left = 0.015;
  top = 0.94;
  width = 0.15;
  height = 0.04;
  this->AddTextBox("__GAZEBO_HUD_FPS__","__GAZEBO_HUD_PANEL_1__","",left,top,width,height,Ogre::ColourValue::White);

  //
  // Time text box
  //
  left = 0.015;
  top = 0.94;
  width = 0.15;
  height = 0.04;
  this->AddTextBox("__GAZEBO_HUD_FPS__","__GAZEBO_HUD_PANEL_1__","",left,top,width,height,Ogre::ColourValue::White);
  */


}

////////////////////////////////////////////////////////////////////////////////
/// Create help text boxes
void OgreHUD::CreateHelp()
{
  float left, top;
  float width, height;

  std::string text;
  text = "Keyboard Commands:\n";
  text += "\t SPACE : Pause/Start physics engine.\n";
  text += "\t         Step one iteration when stepping is enabled.\n";
  text += "\t TAB   : Show/Hide heads up display\n";
  text += "\t ESC   : Quit\n";
  text += "\t [     : Switch to previous camera, if available\n";
  text += "\t ]     : Switch to next camera, if available\n";
  text += "\t T     : Enable stepping. Use SPACE to increment iterations\n";
  text += "\t W     : Translate camera forward\n";
  text += "\t A     : Translate camera left\n";
  text += "\t S     : Translate camera back\n";
  text += "\t D     : Translate camera right\n";
  text += "\t Q     : Translate camera up\n";
  text += "\t E     : Translate camera down\n";
  text += "\t H     : Display this help menu\n";
  text += "\t B     : Toggle display of ODE bounding boxes\n";

  left = 0.25;
  top = 0.25;
  width = 0.5;
  height = 0.5;

  this->AddTextBox("__GAZEBO_HELP_TEXT__","__GAZEBO_HELP_PANEL_1__",text,left,top,width, height);
}
