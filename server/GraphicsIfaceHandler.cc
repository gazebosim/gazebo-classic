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
/* Desc: Handles messages from the graphics ifaces
 * Author: Nate Koenig
 * Date: 9 Mar 2009
 * SVN: $Id:$
 */

#include "gazebo.h"
#include "World.hh"
#include "Entity.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "IfaceFactory.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "OgreDynamicLines.hh"
#include "OgreMovableText.hh"
#include "GraphicsIfaceHandler.hh"
#include "OgreAdaptor.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
GraphicsIfaceHandler::GraphicsIfaceHandler()
{
  this->threeDIface = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
GraphicsIfaceHandler::~GraphicsIfaceHandler()
{
  if (this->threeDIface)
  {
    this->threeDIface->Close();
    delete this->threeDIface;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Load the graphics handler
void GraphicsIfaceHandler::Load(const std::string &_name, Entity *_parent)
{
  this->name = _name;

  // Create the graphics3d interface
  try
  {
    this->threeDIface = (Graphics3dIface*)IfaceFactory::NewIface("graphics3d");

    this->threeDIface->Create(World::Instance()->GetGzServer(), _name);
  }
  catch (std::string err)
  {
    gzerr(0) << "Error: Unable to make graphics3d interface[" << _name << "]\n";
    gzthrow(err);
  }

  this->parent = _parent;
}

////////////////////////////////////////////////////////////////////////////////
/// Init the graphics handler
void GraphicsIfaceHandler::Init()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Update the graphics handler
void GraphicsIfaceHandler::Update()
{
  OgreVisual *vis = NULL;
  std::map<std::string, OgreVisual* >::iterator iter;

  this->threeDIface->Lock(1);
  for (unsigned int i=0; i < this->threeDIface->data->cmdCount; i++)
  {
    // Get the name of visual to create/modify
    std::string visName = this->threeDIface->data->commands[i].name;

    iter = this->visuals.find(visName);

    if (iter == this->visuals.end())
    {
      std::ostringstream nodeName;
      nodeName << "GraphicsIfaceHandler_" << this->name << ": " 
               << this->visuals.size();

      if (this->parent)
        vis = OgreCreator::Instance()->CreateVisual(nodeName.str(), 
            this->parent->GetVisualNode());
      else
        vis = OgreCreator::Instance()->CreateVisual(nodeName.str());

      this->visuals[visName] = vis;
    }
    else
      vis = iter->second;

    switch( this->threeDIface->data->commands[i].drawMode )
    {
      case Graphics3dDrawData::POINTS:
      case Graphics3dDrawData::LINES:
      case Graphics3dDrawData::LINE_STRIP:
        this->DrawSimple(vis, &this->threeDIface->data->commands[i]);
        break;

      case Graphics3dDrawData::PLANE:
      case Graphics3dDrawData::SPHERE:
      case Graphics3dDrawData::CUBE:
      case Graphics3dDrawData::BILLBOARD:
      case Graphics3dDrawData::CONE:
        this->DrawShape(vis, &this->threeDIface->data->commands[i] );
        break;

      case Graphics3dDrawData::TEXT:
        this->DrawText(vis, &this->threeDIface->data->commands[i] );
        break;

      default:
        gzerr(0) << "Unknown draw mode[" 
          << this->threeDIface->data->commands[i].drawMode << "\n";
        break;
    }
  }

  this->threeDIface->data->cmdCount = 0;
  this->threeDIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Helper funciton used to draw simple elements
void GraphicsIfaceHandler::DrawSimple(OgreVisual *vis, Graphics3dDrawData *data)
{
  Vector3 pos;
  bool attached = false;
  OgreDynamicRenderable::OperationType opType;
  OgreDynamicLines *line;

  switch(data->drawMode)
  {
    case Graphics3dDrawData::POINTS:
      opType = OgreDynamicRenderable::OT_POINT_LIST;
      break;
    case Graphics3dDrawData::LINES:
      opType = OgreDynamicRenderable::OT_LINE_LIST;
      break;
    case Graphics3dDrawData::LINE_STRIP:
      opType = OgreDynamicRenderable::OT_LINE_STRIP;
      break;
    case Graphics3dDrawData::TRIANGLES:
      opType = OgreDynamicRenderable::OT_TRIANGLE_LIST;
      break;
    case Graphics3dDrawData::TRIANGLE_STRIP:
      opType = OgreDynamicRenderable::OT_TRIANGLE_STRIP;
      break;
    case Graphics3dDrawData::TRIANGLE_FAN:
      opType = OgreDynamicRenderable::OT_TRIANGLE_FAN;
      break;
    default:
      gzerr(0) << "Unknown draw operation mode[" 
               << data->drawMode << "]\n";
      return;
  }


  if (vis->GetNumAttached() > 0)
  {
    line = (OgreDynamicLines*)(vis->GetAttached(0));
    attached = true;
  }
  else
    line = OgreCreator::Instance()->CreateDynamicLine(opType);

  line->setMaterial(OgreCreator::CreateMaterial( data->color.r,
                                                 data->color.g,
                                                 data->color.b,
                                                 data->color.a ));

  // Set the line vertices
  for (unsigned int i=0; i < data->pointCount; i++)
  {
    pos.Set( data->points[i].x, data->points[i].y, data->points[i].z);

    if (attached && i < line->GetNumPoints())
      line->SetPoint(i,pos);
    else
      line->AddPoint(pos);
  }

  if (!attached)
    vis->AttachObject(line);
}

////////////////////////////////////////////////////////////////////////////////
/// Helper funciton used to draw shapes
void GraphicsIfaceHandler::DrawShape(OgreVisual *vis, Graphics3dDrawData *data)
{
  switch(data->drawMode)
  {
    case Graphics3dDrawData::CUBE:
      {
        if (vis->GetNumAttached() <= 0)
          vis->AttachMesh("unit_box_U1V1");

        vis->SetMaterial( OgreCreator::CreateMaterial( data->color.r,
                                                 data->color.g,
                                                 data->color.b,
                                                 data->color.a ));
        vis->SetScale(Vector3(data->size.x, data->size.y, data->size.z) );
        vis->SetPosition(Vector3(data->pose.pos.x, 
                                 data->pose.pos.y, 
                                 data->pose.pos.z) );
        break;
      }

    case Graphics3dDrawData::CYLINDER:
      {
        if (vis->GetNumAttached() <= 0)
          vis->AttachMesh("unit_cylinder");

        vis->SetMaterial( OgreCreator::CreateMaterial( data->color.r,
                                                 data->color.g,
                                                 data->color.b,
                                                 data->color.a ));


        vis->SetScale(Vector3(data->size.x, data->size.y, data->size.z) );
        vis->SetPosition(Vector3(data->pose.pos.x, 
                                 data->pose.pos.y, 
                                 data->pose.pos.z) );
        break;
      }

    case Graphics3dDrawData::CONE:
      {
        if (vis->GetNumAttached() <= 0)
          vis->AttachMesh("unit_cone");

        vis->SetMaterial( OgreCreator::CreateMaterial( data->color.r,
                                                 data->color.g,
                                                 data->color.b,
                                                 data->color.a ));

        vis->SetScale(Vector3(data->size.x, data->size.y, data->size.z) );
        vis->SetPosition(Vector3(data->pose.pos.x, 
                                 data->pose.pos.y, 
                                 data->pose.pos.z) );
        break;
      }

    case Graphics3dDrawData::SPHERE:
      {
        if (vis->GetNumAttached() <= 0)
          vis->AttachMesh("unit_sphere");

        vis->SetMaterial( OgreCreator::CreateMaterial( data->color.r,
                                                 data->color.g,
                                                 data->color.b,
                                                 data->color.a ));

        vis->SetScale(Vector3(data->size.x, data->size.y, data->size.z) );
        vis->SetPosition(Vector3(data->pose.pos.x, 
                                 data->pose.pos.y, 
                                 data->pose.pos.z) );
        break;
      }
    case Graphics3dDrawData::BILLBOARD:
      {
        bool attached = false;
        Ogre::BillboardSet *bset = NULL;

        if (vis->GetNumAttached() >0)
        {
          attached = true;
          bset = (Ogre::BillboardSet *)vis->GetAttached(0);
          bset->clear();
        }
        else
        {
          std::ostringstream bname;

          bname << "BILLBOARD_" << this->name;
          bset = OgreAdaptor::Instance()->sceneMgr->createBillboardSet(
              bname.str().c_str());
        }

        Ogre::Billboard *billboard = bset->createBillboard(
            Ogre::Vector3(data->pose.pos.x,
              data->pose.pos.y,
              data->pose.pos.z));

        billboard->setDimensions(data->size.x, data->size.y);

        std::string textureName = data->billboardTexture;

        if (textureName.find(".") != std::string::npos)
          bset->setMaterialName( textureName );
        else 
          bset->setMaterialName( 
              OgreCreator::CreateMaterialFromTexFile( textureName ));

        if (!attached)
          vis->AttachObject(bset);

        break;
      }
    default:
      {
        gzerr(0) << "Unknown draw mode for shapes[" 
          << data->drawMode << "]\n";
        break;
      }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Helper funciton used to draw text
void GraphicsIfaceHandler::DrawText(OgreVisual *vis, Graphics3dDrawData *data)
{
  bool attached = false;
  OgreMovableText* msg = NULL;

  if (vis->GetNumAttached() > 0)
  {
    attached = true;
    msg = (OgreMovableText*)(vis->GetAttached(0));
  }
  else
    msg = OgreCreator::Instance()->CreateMovableText();

  try
  {
    msg->Load(data->name, data->text, "Arial", data->size.x);
  }
  catch (Ogre::Exception e)
  {
    std::ostringstream stream;
    stream <<  "Unable to create the text. " << e.getDescription() <<std::endl;
    gzthrow(stream.str() );
  }

  msg->SetTextAlignment(OgreMovableText::H_CENTER, OgreMovableText::V_ABOVE);

  vis->SetPosition(Vector3(data->pose.pos.x,
                           data->pose.pos.y,
                           data->pose.pos.z));

  if (!attached)
    vis->AttachObject( msg );
}

