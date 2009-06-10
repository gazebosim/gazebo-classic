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
/* Desc: FLTK GL Frame
 * Author: Nate Koenig
 * Date: 18 Jun 2008
 * SVN: $Id$
 */

#ifndef GLFRAME_HH
#define GLFRAME_HH

#include <FL/Fl_Group.H>
#include <FL/Fl_Choice.H>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Output.H>
#include <string>

#include "Pose3d.hh"

namespace gazebo
{
  class GLWindow;
  class OgreCamera;
  class XMLConfigNode;

  class GLFrame : public Fl_Group
  {
    /// \brief Constructor
    public: GLFrame(int x, int y, int w, int h, const std::string &name="");

    /// \brief Destructor
    public: virtual ~GLFrame();

    /// \brief Load the frame
    public: void Load( XMLConfigNode *node );

    /// \brief Create user cameras
    public: void CreateCameras();

    /// \brief Initialize 
    public: void Init();

    /// \brief Update 
    public: void Update();

    /// \brief Get the pose of the camera attached to this frame
    public: Pose3d GetCameraPose() const;

    /// \brief Set the pose of the camera attached to this frame
    public: void SetCameraPose( const Pose3d &pose );

    /// \brief Get a pointer to the render window
    public: GLWindow *GetWindow() const;

    /// \brief Boost slot, called when a new camera is added.
    public: void CameraAddedSlot(OgreCamera *newCamera);

    /// \brief Split frame callback
    private: static void SplitCB(Fl_Widget *widget, void *data);

    /// \brief Switch view callback
    private: static void ViewCB(Fl_Widget *widget, void *data);

    /// Pointer to the actual render window
    private: GLWindow *glWindow;

    /// Header toolbar 
    private: Fl_Group *headerBar;

    /// Footer toolbar 
    private: Fl_Group *footerBar;

    /// Viewpoint chooser
    private: Fl_Choice *viewChoice;

    /// Split window chooser
    private: Fl_Choice *splitChoice;

    private: Fl_Output *outputXYZ;
    private: Fl_Output *outputRPY;

    /// Starting pose of the camera
    private: Pose3d startPose;

    private: bool saveFrames;
    private: std::string savePathname;
  };
}

#endif
