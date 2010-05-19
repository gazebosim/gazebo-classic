#ifndef ENTITYMAKER_HH
#define ENTITYMAKER_HH

namespace gazebo
{
  class MouseEvent;

  class EntityMaker
  {
    /// \brief Constructor
    public: EntityMaker();

    /// \brief Destructor
    public: virtual ~EntityMaker();

    public: virtual void Start() = 0;
    public: virtual void Stop() = 0;
    public: virtual bool IsActive() const = 0;

    public: virtual void MousePushCB(const MouseEvent &event);
    public: virtual void MouseReleaseCB(const MouseEvent &event);
    public: virtual void MouseDragCB(const MouseEvent &event);

    protected: virtual void CreateTheEntity() = 0;
  };
}

#endif
