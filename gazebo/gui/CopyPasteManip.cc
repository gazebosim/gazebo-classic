#include "gazebo/gui/CopyPasteManip.hh"

using namespace gazebo;
using namespace gui;

CopyPasteManip::CopyPasteManip(rendering::UserCameraPtr _userCamera,
                   common::MouseEvent *_mouseEvent) :
                   userCamera(_userCamera), mouseEvent(_mouseEvent)
{
}

void CopyPasteManip::Paste(EntityMaker *entityMaker)
{
  entityMaker->Start(this->userCamera);
  // this makes the entity appear at the mouse cursor
  entityMaker->OnMouseMove(*(this->mouseEvent));
  gui::Events::manipMode("make_entity");
}
