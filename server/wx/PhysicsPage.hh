#ifndef PHYSICS_PAGE
#define PHYSICS_PAGE

#include "ParamPage.hh"

class wxPropertyGrid;
class wxPropertyGridEvent;

namespace gazebo
{
  class PropertyManager;

  class PhysicsPage : public ParamPage
  {
    public: PhysicsPage(wxWindow *parent);
    public: virtual ~PhysicsPage();

    public: virtual void Apply();

    private: void OnPropertyChanged(wxPropertyGridEvent &event);

    private: wxPropertyGrid *propGrid;
    private: PropertyManager *propManager;
  };
}

#endif
