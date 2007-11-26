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
/* Desc: OGRE frame listener
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id:$
 */

#ifndef OGREFRAMELISTENER_HH
#define OGREFRAMELISTENER_HH

#include <OgreFrameListener.h>
//#include <OgreEventListeners.h>
#include <Ogre.h>


namespace Ogre
{
  class EventProcessor;
  class InputReader;
  class MouseEvent;
  class RaySceneQuery;
}

namespace gazebo
{
  
  class OgreAdaptor;
 
  /// \brief Ogre frame listener
  class OgreFrameListener : public Ogre::FrameListener//, public Ogre::WindowEventListener
  {
    /// \brief Constructor
    public: OgreFrameListener();

    /// \brief Destructor
    public: virtual ~OgreFrameListener();
 
    /// \brief Frame has started event
    public: virtual bool frameStarted( const Ogre::FrameEvent &evt);

    /// \brief Frame has ended event
    public: virtual bool frameEnded( const Ogre::FrameEvent &evt);
  
    private: Ogre::Vector3 directionVec;
  
    private: float moveAmount;
    private: float moveScale;
    private: float rotateAmount;
  
  };

}
#endif
