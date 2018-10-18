#ifndef SPOTLIGHTMAKER_HH
#define SPOTLIGHTMAKER_HH

#include "Vector2.hh"
#include "EntityMaker.hh"

namespace gazebo
{
  class SpotLightMaker : public EntityMaker
  {
    public: SpotLightMaker();
    public: virtual ~SpotLightMaker();
  
    public: virtual void Start();
    public: virtual void Stop();
    public: virtual bool IsActive() const;

    public: virtual void MousePushCB(const MouseEvent &event);
    public: virtual void MouseReleaseCB(const MouseEvent &event);
    public: virtual void MouseDragCB(const MouseEvent &event);
  
    private: virtual void CreateTheEntity();
    private: int state;
    private: Vector2<int> mousePushPos;
    private: std::string lightName;
    private: int index;
  };
}

#endif
