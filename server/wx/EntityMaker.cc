#include "Camera.hh"
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
