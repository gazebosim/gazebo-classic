#include "CameraManager.hh"
#include "OgreCamera.hh"
#include "EntityMaker.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
EntityMaker::EntityMaker()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
EntityMaker::~EntityMaker()
{
}

////////////////////////////////////////////////////////////////////////////////
void EntityMaker::MousePushCB(const MouseEvent &event)
{
}

////////////////////////////////////////////////////////////////////////////////
void EntityMaker::MouseReleaseCB(const MouseEvent &event)
{
}

////////////////////////////////////////////////////////////////////////////////
void EntityMaker::MouseDragCB(const MouseEvent &event)
{
}

////////////////////////////////////////////////////////////////////////////////
// Get point on a plane
Vector3 EntityMaker::GetWorldPointOnPlane(int x, int y, Vector3 planeNorm, double d)
{
  Vector3 origin, dir;
  double dist;

  // Cast two rays from the camera into the world
  CameraManager::Instance()->GetActiveCamera()->GetCameraToViewportRay(x, y, origin, dir);

  dist = origin.GetDistToPlane(dir, planeNorm, d);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  return origin + dir * dist; 
}
