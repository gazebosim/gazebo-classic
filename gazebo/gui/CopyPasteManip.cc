#include <gazebo/gui/CopyPasteManip.hh>

CopyPasteManip::CopyPasteManip(rendering::UserCameraPtr _userCamera,
                   common::MouseEventPtr _mouseEvent) :
                   userCamera(_userCamera), mouseEvent(_mouseEvent)
{
}

void CopyPasteManip::Paste(EntityMaker *entityMaker)
{
  entityMaker->Start(*(this->userCamera));
  // this makes the entity appear at the mouse cursor
  entityMaker->OnMouseMove(*(this->mouseEvent));
  gui::Events::manipMode("make_entity");
}
