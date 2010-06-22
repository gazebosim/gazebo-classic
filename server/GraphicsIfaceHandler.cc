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

#include <stdint.h>

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
    this->threeDIface = NULL;
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
    this->threeDIface = (libgazebo::Graphics3dIface*)libgazebo::IfaceFactory::NewIface("graphics3d");

    this->threeDIface->Create(World::Instance()->GetGzServer(), _name);
    this->threeDIface->data->cmdCount = 0;
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
      case libgazebo::Graphics3dDrawData::POINTS:
      case libgazebo::Graphics3dDrawData::LINES:
      case libgazebo::Graphics3dDrawData::LINE_STRIP:
        this->DrawSimple(vis, &this->threeDIface->data->commands[i]);
        break;

      case libgazebo::Graphics3dDrawData::PLANE:
      case libgazebo::Graphics3dDrawData::SPHERE:
      case libgazebo::Graphics3dDrawData::CUBE:
      case libgazebo::Graphics3dDrawData::BILLBOARD:
      case libgazebo::Graphics3dDrawData::CONE:
        this->DrawShape(vis, &this->threeDIface->data->commands[i] );
        break;

      case libgazebo::Graphics3dDrawData::TEXT:
        this->DrawText(vis, &this->threeDIface->data->commands[i] );
        break;

      case libgazebo::Graphics3dDrawData::METERBAR:
        this->DrawMeterBar(vis, &this->threeDIface->data->commands[i] );
        break;

      case libgazebo::Graphics3dDrawData::RIBBONTRAIL:
        vis->SetRibbonTrail(this->threeDIface->data->commands[i].intVar);
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
void GraphicsIfaceHandler::DrawSimple(OgreVisual *vis, libgazebo::Graphics3dDrawData *data)
{
  Vector3 pos;
  bool attached = false;
  OgreDynamicRenderable::OperationType opType;
  OgreDynamicLines *line;

  switch(data->drawMode)
  {
    case libgazebo::Graphics3dDrawData::POINTS:
      opType = OgreDynamicRenderable::OT_POINT_LIST;
      break;
    case libgazebo::Graphics3dDrawData::LINES:
      opType = OgreDynamicRenderable::OT_LINE_LIST;
      break;
    case libgazebo::Graphics3dDrawData::LINE_STRIP:
      opType = OgreDynamicRenderable::OT_LINE_STRIP;
      break;
    case libgazebo::Graphics3dDrawData::TRIANGLES:
      opType = OgreDynamicRenderable::OT_TRIANGLE_LIST;
      break;
    case libgazebo::Graphics3dDrawData::TRIANGLE_STRIP:
      opType = OgreDynamicRenderable::OT_TRIANGLE_STRIP;
      break;
    case libgazebo::Graphics3dDrawData::TRIANGLE_FAN:
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
void GraphicsIfaceHandler::DrawShape(OgreVisual *vis, libgazebo::Graphics3dDrawData *data)
{
  switch(data->drawMode)
  {
    case libgazebo::Graphics3dDrawData::CUBE:
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

    case libgazebo::Graphics3dDrawData::CYLINDER:
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

    case libgazebo::Graphics3dDrawData::CONE:
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

    case libgazebo::Graphics3dDrawData::SPHERE:
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

    case libgazebo::Graphics3dDrawData::BILLBOARD:
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
void GraphicsIfaceHandler::DrawText(OgreVisual *vis, libgazebo::Graphics3dDrawData *data)
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


////////////////////////////////////////////////////////////////////////////////
// Helper function used to draw a progress bar
void GraphicsIfaceHandler::DrawMeterBar(OgreVisual *vis, 
                                        libgazebo::Graphics3dDrawData *data )
{
  bool attached = false;
  Ogre::BillboardSet *bset = NULL;
  std::ostringstream bname;
  Ogre::TexturePtr texture;
  Ogre::MaterialPtr material;

  unsigned int width = 64;
  unsigned int height = 64;
  unsigned int rowWidth = width*4;

  bname << "METERBAR_" << this->name;

  if (vis->GetNumAttached() >0)
  {
    bset = (Ogre::BillboardSet *)vis->GetAttached(0);
    bset->clear();
    attached = true;
    texture = Ogre::TextureManager::getSingleton().getByName(
                                                     bname.str()+"texture");
    material = Ogre::MaterialManager::getSingleton().getByName(
                                                     bname.str()+"material");
  }
  else
  {
    bset = OgreAdaptor::Instance()->sceneMgr->createBillboardSet(
        bname.str().c_str());

    // Create the texture
    texture = Ogre::TextureManager::getSingleton().createManual(
        bname.str()+"texture",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        width, height,
        0,                // number of mipmaps
        Ogre::PF_BYTE_RGBA,     // pixel format
        Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);      // usage; 
                             //should be TU_DYNAMIC_WRITE_ONLY_DISCARDABLE 
                             //for textures updated very often (e.g. each frame)
                             
                             // Create a material using the texture
    material = Ogre::MaterialManager::getSingleton().create(
        bname.str()+"material", 
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    material->getTechnique(0)->getPass(0)->createTextureUnitState(
        bname.str()+"texture");
    material->getTechnique(0)->getPass(0)->setSceneBlending(
        Ogre::SBT_TRANSPARENT_ALPHA);
  }

  Ogre::Billboard *billboard = bset->createBillboard(
      Ogre::Vector3(data->pose.pos.x,
        data->pose.pos.y,
        data->pose.pos.z));

  billboard->setDimensions(data->size.x, data->size.y);

  // Get the pixel buffer
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture->getBuffer();

  // Lock the pixel buffer and get a pixel box
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);

  // for best performance use HBL_DISCARD!
  const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();

  uint8_t* pDest = static_cast<uint8_t*>(pixelBox.data);

  char red = data->color.b*255; 
  char green = data->color.g*255; 
  char blue = data->color.r*255; 

  // Fill in some pixel data.
  for (size_t j = 0; j < height; j++)
  {
    for(size_t i = 0; i < width; i++)
    {
      pDest[j*rowWidth + i*4 + 0] = red; 
      pDest[j*rowWidth + i*4 + 1] = green; 
      pDest[j*rowWidth + i*4 + 2] = blue;

      if (i < width * data->fltVar)
      {
        pDest[j*rowWidth + i*4 + 3] = 180;
      }
      else
        pDest[j*rowWidth + i*4 + 3] = 0;
    }
  }

  
  for (size_t j =0; j < width; j++)
  {
    // 2-pixel top border
    pDest[j*4+0] = red;
    pDest[j*4+1] = green;
    pDest[j*4+2] = blue;
    pDest[j*4+3] = 255;

    pDest[rowWidth +j*4+0] = red;
    pDest[rowWidth +j*4+1] = green;
    pDest[rowWidth +j*4+2] = blue;
    pDest[rowWidth +j*4+3] = 255;

    // 2-pixel bottom border
    pDest[(height-1)*rowWidth+j*4+0] = red;
    pDest[(height-1)*rowWidth+j*4+1] = green;
    pDest[(height-1)*rowWidth+j*4+2] = blue;
    pDest[(height-1)*rowWidth+j*4+3] = 255;

    pDest[(height-2)*rowWidth+j*4+0] = red;
    pDest[(height-2)*rowWidth+j*4+1] = green;
    pDest[(height-2)*rowWidth+j*4+2] = blue;
    pDest[(height-2)*rowWidth+j*4+3] = 255;
  }


  for (size_t j =0; j < height; j++)
  {
    // 2-pixel left border
    pDest[rowWidth*j+0] = red;
    pDest[rowWidth*j+1] = green;
    pDest[rowWidth*j+2] = blue;
    pDest[rowWidth*j+3] = 255;

    pDest[rowWidth*j+4] =  red;
    pDest[rowWidth*j+5] =  green;
    pDest[rowWidth*j+6] =  blue;
    pDest[rowWidth*j+7] = 255;

    // 2-pixel right border
    pDest[rowWidth*j + (width-1)*4+0] = red;
    pDest[rowWidth*j + (width-1)*4+1] = green;
    pDest[rowWidth*j + (width-1)*4+2] = blue;
    pDest[rowWidth*j + (width-1)*4+3] = 255;

    pDest[rowWidth*j + (width-2)*4+0] = red;
    pDest[rowWidth*j + (width-2)*4+1] = green;
    pDest[rowWidth*j + (width-2)*4+2] = blue;
    pDest[rowWidth*j + (width-2)*4+3] = 255;
  }

  // Unlock the pixel buffer
  pixelBuffer->unlock();

  bset->setMaterialName(bname.str()+"material");

  if (!attached)
  {
    vis->AttachObject(bset);
  }

}
