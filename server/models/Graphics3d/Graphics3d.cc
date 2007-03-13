#include <Ogre.h>
#include "ModelFactory.hh"
#include "XMLConfig.hh"
#include "Graphics3d.hh"

GZ_REGISTER_STATIC("Graphics3d", Graphics3d);

Graphics3d::Graphics3d()
{
}

Graphics3d::~Graphics3d()
{
}

// Load the child model
int Graphics3d::LoadChild(XMLConfigNode *node)
{
  return 0;
}

// Initialize the child model
int Graphics3d::InitChild()
{
  return 0;
}

// Update the child model
int Graphics3d::UpdateChild()
{
/*  Graphics3dIface *gIface = (Graphics3dIface*)(this->ifaces["graphics3d"]);

  gIface->Lock(1);
  std::cout << "Point Count[" << gIface->data->point_count << "]\n";
  gIface->Unlock();
  */

  return 0;
}

// Finilaize thie child model
int Graphics3d::FiniChild()
{
  return 0;
}
