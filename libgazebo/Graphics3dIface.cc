#include "gazebo.h"

#include <iostream>

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Graphics3dIface::Graphics3dIface()
  :Iface("graphics3d", sizeof(Graphics3dIface)+sizeof(Graphics3dData)) 
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Graphics3dIface::~Graphics3dIface() 
{
  this->data = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Create the interface (used by Gazebo server)
void Graphics3dIface::Create(Server *server, std::string id)
{
  Iface::Create(server,id); 
  this->data = (Graphics3dData*)this->mMap; 
}

////////////////////////////////////////////////////////////////////////////////
/// Open an existing interface
void Graphics3dIface::Open(Client *client, std::string id)
{
  Iface::Open(client,id); 
  this->data = (Graphics3dData*)this->mMap; 
}

////////////////////////////////////////////////////////////////////////////////
/// Draw a simple object, that is defined by a series of points
void Graphics3dIface::DrawSimple(const char *name,  
                                 Graphics3dDrawData::DrawMode mode,
                                 Vec3 *points, unsigned int numPoints, 
                                 Color clr )
{
  this->Lock(1);
  Graphics3dDrawData *cmd = &(this->data->commands[this->data->cmdCount++]);

  cmd->pointCount = numPoints;

  cmd->drawMode = mode;

  // Set the name of the graphics drawable
  memset( cmd->name, 0, GAZEBO_GRAPHICS3D_MAX_NAME);
  strcpy( cmd->name, name);

  memcpy(cmd->points, points, numPoints * sizeof(Vec3));

  cmd->color.r = clr.r;
  cmd->color.g = clr.g;
  cmd->color.b = clr.b;
  cmd->color.a = clr.a;

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Draw a shape
void Graphics3dIface::DrawShape(const char *name, 
                                Graphics3dDrawData::DrawMode mode,
                                Vec3 pos, Vec3 size, Color clr)
{
  this->Lock(1);
  Graphics3dDrawData *cmd = &(this->data->commands[this->data->cmdCount++]);

  if (mode != Graphics3dDrawData::SPHERE && 
      mode != Graphics3dDrawData::CUBE && 
      mode != Graphics3dDrawData::CYLINDER)
  {
    std::cerr << "Invalid shape draw mode[" << mode << "]\n";
    return;
  }

  cmd->drawMode = mode;

  // Set the name of the graphics drawable
  memset( cmd->name, 0, GAZEBO_GRAPHICS3D_MAX_NAME);
  strcpy( cmd->name, name);

  cmd->pose.pos.x = pos.x;
  cmd->pose.pos.y = pos.y;
  cmd->pose.pos.z = pos.z;

  cmd->size.x = size.x;
  cmd->size.y = size.y;
  cmd->size.z = size.z;


  cmd->color.r = clr.r;
  cmd->color.g = clr.g;
  cmd->color.b = clr.b;
  cmd->color.a = clr.a;

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Draw a billboard
void Graphics3dIface::DrawBillboard(const char *name, const char *texture, 
                                    Vec3 pos, Vec2 size)
{
  this->Lock(1);
  Graphics3dDrawData *cmd = &(this->data->commands[this->data->cmdCount++]);

  cmd->drawMode = Graphics3dDrawData::BILLBOARD;

  // Set the name of the graphics drawable
  memset( cmd->name, 0, GAZEBO_GRAPHICS3D_MAX_NAME);
  strcpy( cmd->name, name);

  // Set the material to draw on the billboard
  memset( cmd->billboardTexture, 0, GAZEBO_GRAPHICS3D_MAX_NAME);
  strcpy( cmd->billboardTexture, texture);

  cmd->pose.pos.x = pos.x;
  cmd->pose.pos.y = pos.y;
  cmd->pose.pos.z = pos.z;

  cmd->size.x = size.x;
  cmd->size.y = size.y;
  cmd->size.z = 0;

  this->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Draw text
void Graphics3dIface::DrawText(const char *name, const char *text, Vec3 pos, 
                               float fontSize)
{
  this->Lock(1);
  Graphics3dDrawData *cmd = &(this->data->commands[this->data->cmdCount++]);

  cmd->drawMode = Graphics3dDrawData::TEXT;

  // Set the name of the graphics drawable
  memset( cmd->name, 0, GAZEBO_GRAPHICS3D_MAX_NAME);
  strcpy( cmd->name, name);

  // Set the text to draw
  memset( cmd->text, 0, GAZEBO_GRAPHICS3D_MAX_NAME);
  strcpy( cmd->text, text);

  cmd->pose.pos.x = pos.x;
  cmd->pose.pos.y = pos.y;
  cmd->pose.pos.z = pos.z;

  cmd->size.x = fontSize;

  this->Unlock();
}
