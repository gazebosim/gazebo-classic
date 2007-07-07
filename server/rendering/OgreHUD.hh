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

#ifndef OGREHUD_HH
#define OGREHUD_HH

#include <Ogre.h>

namespace gazebo
{
  class CameraSensor;
  class OgreAdaptor;

  class OgreHUD
  {
    /// \brief Constructor
    private: OgreHUD();

    /// \brief Destructor
    private: ~OgreHUD();

    /// \brief Get a pointer to the text renderer
    public: static OgreHUD *Instance();

    /// \brief Set the camera to display
    public: void SetCamera(const CameraSensor *camera);

    /// \brief Add a text box
    public: void AddTextBox( const std::string& id,
                const std::string& text,
                Ogre::Real x, 
                Ogre::Real y,
                Ogre::Real width, 
                Ogre::Real height,
                const Ogre::ColourValue& color = 
                Ogre::ColourValue(1.0, 1.0, 1.0, 1.0));

    /// \brief Hide a text box from being displayed
    public: void HideTextBox(const std::string &id);

    /// \brief Show a text box
    public: void ShowTextBox(const std::string &id);

    /// \brief Remove a text box
    public: void RemoveTextBox(const std::string& id);

    /// \brief Set text 
    public: void SetText(const std::string& id, const std::string& Text);

    private: Ogre::OverlayManager *overlayMgr;
    private: Ogre::OverlayContainer *hudPanel;
    private: Ogre::OverlayContainer *cameraPanel;
    private: Ogre::OverlayContainer *backgroundPanel;

    private: static OgreHUD *myself;

    private: OgreAdaptor *ogreAdaptor;
  };

}
#endif
