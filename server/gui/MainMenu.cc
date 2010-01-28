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
/* Desc: FLTK main menu
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#include <FL/Fl_File_Chooser.H>

#include "Global.hh"
#include "World.hh"
#include "Simulator.hh"
#include "Gui.hh"
#include "CameraManager.hh"
#include "OgreCamera.hh"
#include "OgreAdaptor.hh"
#include "MainMenu.hh"

#include <boost/thread.hpp>

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
MainMenu::MainMenu(int x, int y, int w, int h, char *name)
    : Fl_Menu_Bar(x,y,w,h,name)
{
  this->box(FL_NO_BOX);
  this->color(BG_COLOR, BG_COLOR);

  const Fl_Menu_Item menuitems[] =
  {
    { "File", 0, 0, 0, FL_SUBMENU,  FL_NORMAL_LABEL, 0, 14, 0 },
    //  { "Open", 0, &gazebo::MainMenu::OpenCB, 0, 0, FL_NORMAL_LABEL,0, 14,0 },
    { "Save World", 0, &gazebo::MainMenu::SaveWorldCB, 0, 0, FL_NORMAL_LABEL,0, 14,0 },
    { "Save Frames", 0, &gazebo::MainMenu::SaveFramesCB, 0, FL_MENU_TOGGLE, FL_NORMAL_LABEL,0, 14,0 },
    { "Reset", 0, &gazebo::MainMenu::ResetCB, 0, 0, FL_NORMAL_LABEL,0, 14,0 },
    { "Quit", 0, &gazebo::MainMenu::QuitCB, 0, 0, FL_NORMAL_LABEL,0, 14,0 },
    { 0 },

    { "View", 0, 0, 0, FL_SUBMENU, FL_NORMAL_LABEL, 0, 14, 0},
    { "Wireframe", 0, &gazebo::MainMenu::WireframeCB,0, FL_MENU_TOGGLE, FL_NORMAL_LABEL, 0, 14, 0},
    { "Show Physics", 0, &gazebo::MainMenu::ShowPhysicsCB,0, FL_MENU_TOGGLE, FL_NORMAL_LABEL, 0, 14, 0},
    { "Show Bounding Boxes", 0, &gazebo::MainMenu::ShowBoundingBoxesCB,0, FL_MENU_TOGGLE, FL_NORMAL_LABEL, 0, 14, 0},
    { "Show Joints", 0, &gazebo::MainMenu::ShowJointsCB,0, FL_MENU_TOGGLE, FL_NORMAL_LABEL, 0, 14, 0},
    { "Show Contacts", 0, &gazebo::MainMenu::ShowContactsCB,0, FL_MENU_TOGGLE, FL_NORMAL_LABEL, 0, 14, 0},
    { "Show Lights", 0, &gazebo::MainMenu::ShowLightsCB,0, FL_MENU_TOGGLE, FL_NORMAL_LABEL, 0, 14, 0},
    { "Show Cameras", 0, &gazebo::MainMenu::ShowCamerasCB,0, FL_MENU_TOGGLE, FL_NORMAL_LABEL, 0, 14, 0},
    { "Per-Pixel Lighting", 0, &gazebo::MainMenu::PerPixelLightingCB,0, FL_MENU_TOGGLE|FL_MENU_VALUE, FL_NORMAL_LABEL, 0, 14, 0},
    { 0 },

    { 0 }
  };


  this->copy(menuitems);
}

////////////////////////////////////////////////////////////////////////////////
// Open Callback
void MainMenu::OpenCB(Fl_Widget * /*w*/, void * /*data*/)
{
  Fl_File_Chooser *fileChooser = new Fl_File_Chooser(getenv("PWD"),"*.world",Fl_File_Chooser::SINGLE,"Open World File");

  fileChooser->show();

  while (fileChooser->shown())
    Fl::wait();
}

////////////////////////////////////////////////////////////////////////////////
// Save world Callback
void MainMenu::SaveWorldCB(Fl_Widget * /*w*/, void * /*data*/)
{
  Fl_File_Chooser *fileChooser = new Fl_File_Chooser(getenv("PWD"),"*.world",Fl_File_Chooser::CREATE,"Save World File");

  fileChooser->show();

  while (fileChooser->shown())
    Fl::wait();

  if (fileChooser->count() == 1)
  {
    Simulator::Instance()->Save( fileChooser->value(1) );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Save Frames Callback
void MainMenu::SaveFramesCB(Fl_Widget * /*w*/, void * /*data*/)
{
  OgreCamera *camera = CameraManager::Instance()->GetActiveCamera();
  camera->ToggleSaveFrame();
}

////////////////////////////////////////////////////////////////////////////////
// Quit Callback
void MainMenu::QuitCB(Fl_Widget * /*w*/, void * /*data*/)
{
  Simulator::Instance()->SetUserQuit();
}

////////////////////////////////////////////////////////////////////////////////
// Show Bounding boxes callback
void MainMenu::ShowBoundingBoxesCB(Fl_Widget * /*w*/, void * /*data*/)
{
  World::Instance()->SetShowBoundingBoxes( !World::Instance()->GetShowBoundingBoxes() );
}

////////////////////////////////////////////////////////////////////////////////
// Show Bounding boxes callback
void MainMenu::ShowJointsCB(Fl_Widget * /*w*/, void * /*data*/)
{
  World::Instance()->SetShowJoints( !World::Instance()->GetShowJoints() );
}

////////////////////////////////////////////////////////////////////////////////
// Show Bounding boxes callback
void MainMenu::ShowPhysicsCB(Fl_Widget * /*w*/, void * /*data*/)
{
  World::Instance()->SetShowPhysics( !World::Instance()->GetShowPhysics() );
}

////////////////////////////////////////////////////////////////////////////////
// Reset the world
void MainMenu::ResetCB(Fl_Widget * /*w*/, void * /*data*/)
{
  // stop simulation when this is happening
  boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
  World::Instance()->Reset();
}

////////////////////////////////////////////////////////////////////////////////
// View the world as wireframe
void MainMenu::WireframeCB(Fl_Widget * /*w*/, void * /*data*/)
{
  World::Instance()->SetWireframe( !World::Instance()->GetWireframe() );
}

////////////////////////////////////////////////////////////////////////////////
// View the contacts
void MainMenu::ShowContactsCB(Fl_Widget * /*w*/, void * /*data*/)
{
  World::Instance()->SetShowContacts( !World::Instance()->GetShowContacts() );
}

////////////////////////////////////////////////////////////////////////////////
// View the light source visuals
void MainMenu::ShowLightsCB(Fl_Widget * /*w*/, void * /*data*/)
{
  World::Instance()->SetShowLights( !World::Instance()->GetShowLights() );
}

////////////////////////////////////////////////////////////////////////////////
// View the light source visuals
void MainMenu::ShowCamerasCB(Fl_Widget * /*w*/, void * /*data*/)
{
  World::Instance()->SetShowCameras( !World::Instance()->GetShowCameras() );
}

////////////////////////////////////////////////////////////////////////////////
// Use per-pixel lighting
void MainMenu::PerPixelLightingCB(Fl_Widget * /*w*/, void * /*data*/)
{
  World::Instance()->SetPerPixelLighting( !World::Instance()->GetPerPixelLighting() );
}
