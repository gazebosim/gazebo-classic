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
/* Desc: FLTK GL Frame manager
 * Author: Nate Koenig
 * Date: 18 Jun 2008
 * SVN: $Id:$
 */

#include "XMLConfig.hh"

#include "GLFrame.hh"
#include "GLWindow.hh"
#include "GLFrameManager.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
GLFrameManager::GLFrameManager(int x, int y, int w, int h, const std::string &name) : Fl_Tile(x,y,w,h, name.c_str())
{
  //GLFrame *frame = NULL;

  // Create a deafult view
  /*frame = new GLFrame(x, y, w, h,"");
  this->frames.push_back(frame);
  this->add(frame);
  */

  this->end();

  this->resizable(this);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GLFrameManager::~GLFrameManager()
{
  std::vector<GLFrame *>::iterator iter;
  for (iter = this->frames.begin(); iter != this->frames.end(); iter++)
  {
    if (*iter) 
      delete (*iter);
  }

}

////////////////////////////////////////////////////////////////////////////////
// Load from xml
void GLFrameManager::Load( XMLConfigNode *node )
{
  GLFrame *frame = NULL;
  int windowCount = 0;
  XMLConfigNode *rowNode = NULL;
  XMLConfigNode *camNode = NULL;

  if (node)
  {
    rowNode = node->GetChild("row");

    float frameHeight, frameWidth;
    int width = this->w();
    int height = this->h();
    int x = this->x();
    int y = this->y();

    // Process each row
    while (rowNode)
    {
      camNode = rowNode->GetChild("camera");

      std::string heightStr = rowNode->GetString("height","",1);

      frameHeight = atof( heightStr.substr(0, heightStr.find( "%", 0 ) ).c_str() );

      frameHeight = frameHeight/100.0 * height;

      // Process each camera
      while (camNode)
      {
        std::string widthStr = camNode->GetString("width","",1);
        frameWidth = atof( widthStr.substr(0, widthStr.find( "%", 0 ) ).c_str() );

        frameWidth = frameWidth/100.0 * width;

        windowCount = this->children();

        // Create the frame
        frame = new GLFrame( x, y, (int)frameWidth, (int)frameHeight, "" );
        frame->Load(camNode);

        this->frames.push_back( frame );
        this->insert(*frame, windowCount);

        x += (int)frameWidth;

        camNode = camNode->GetNext();
      }

      y += (int)frameHeight;
      x = this->x();

      rowNode = rowNode->GetNext();
    }
  }

  // Create a big default window if no frames have been defined
  if (this->frames.size() == 0)
  {
    frame = new GLFrame( this->x(), this->y(), this->w(), this->h(), "" );
    this->frames.push_back( frame );
    this->insert(*frame, windowCount);
  }
  
}

////////////////////////////////////////////////////////////////////////////////
/// Save the gui params in xml format
void GLFrameManager::Save(std::string &prefix, std::ostream &stream)
{
  std::vector<GLFrame *>::iterator iter;
  std::vector<int> rows;

  stream << prefix << "<frames>\n";

  int cy = -1;
  for (iter = this->frames.begin(); iter != this->frames.end(); iter++)
  {
    int height = (int)( ((float)(*iter)->h() / this->h()) * 100 );
    int width = (int)( ((float)(*iter)->w() / this->w()) * 100 );
    Pose3d pose = (*iter)->GetCameraPose();

    if ((*iter)->y() != cy)
    {
      if (cy != -1)
        stream << prefix << "  </row>\n";

      cy = (*iter)->y();
      stream << prefix << "  <row height=\"" << height << "%\">\n";
    }

    stream << prefix << "    <camera width=\"" << width << "%\">\n";
    stream << prefix << "      <xyz>" << pose.pos << "</xyz>\n";
    stream << prefix << "      <rpy>" << pose.rot << "</rpy>\n";
    stream << prefix << "    </camera>\n";
  }

  stream << prefix << "  </row>\n";
  stream << prefix << "</frames>\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Initalize the window manager
void GLFrameManager::Init()
{
  std::vector<GLFrame *>::iterator iter;
  for (iter = this->frames.begin(); iter != this->frames.end(); iter++)
  {
    (*iter)->Init();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Update the window manager
void GLFrameManager::Update()
{
  std::vector<GLFrame *>::iterator iter;

  for (iter = this->frames.begin(); iter != this->frames.end(); iter++)
  {
    (*iter)->Update();
  }
}


////////////////////////////////////////////////////////////////////////////////
// Split a frame
void GLFrameManager::Split(GLFrame *parent, const std::string &type)
{
  GLFrame *glFrame = NULL;
  unsigned int origWidth = parent->w();
  unsigned int origHeight = parent->h();
  unsigned int newWidth, newHeight, originX, originY;
  int windowCount = this->children();

  if (type == "horz")
  {
    newWidth = origWidth / 2;
    newHeight = origHeight;

    originX = parent->x()+newWidth;
    originY = parent->y();
  }
  else if (type == "vert")
  {
    newWidth = origWidth;
    newHeight = origHeight / 2;

    originX = parent->x();
    originY = parent->y() + newHeight;
  }

  parent->resize(parent->x(), parent->y(), newWidth, newHeight);
  if (type == "horz")
    glFrame = new GLFrame(originX, originY, newWidth, newHeight, "GL Window 2");
  else
    glFrame = new GLFrame(originX, originY, newWidth, newHeight, "GL Window 2");

  this->insert(*glFrame, windowCount);
  glFrame->Init();
  this->frames.push_back(glFrame);

  this->redraw();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the avg FPS
float GLFrameManager::GetFPS() const
{
  std::vector<GLFrame*>::const_iterator iter;
  float sum = 0;

  for (iter = this->frames.begin(); iter != this->frames.end(); iter++)
  {
    sum += (*iter)->GetWindow()->GetAvgFPS();
  }

  //printf("Sum[%f] Size[%d]\n", sum, this->frames.size());

  return sum / this->frames.size();
}


