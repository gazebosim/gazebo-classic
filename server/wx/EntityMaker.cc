#include "CameraManager.hh"
#include "OgreCamera.hh"
#include "EntityMaker.hh"

using namespace gazebo;

bool EntityMaker::snapToGrid = true;
double EntityMaker::snapDistance = 0.2;
double EntityMaker::snapGridSize = 1.0;

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
/// Set whether to snap to grid
void EntityMaker::SetSnapToGrid(bool snap)
{
  snapToGrid = snap;
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
// Get a point snapped to a grid
Vector3 EntityMaker::GetSnappedPoint(Vector3 p)
{
  Vector3 result = p;

  if (this->snapToGrid)
  {
    Vector3 rounded = (p / this->snapGridSize).GetRounded() * this->snapGridSize;
    if (p.Distance( rounded ) < this->snapDistance)
      result = rounded;
  }

  return result;
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
