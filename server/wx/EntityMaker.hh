#ifndef ENTITYMAKER_HH
#define ENTITYMAKER_HH

#include "Vector3.hh"

namespace gazebo
{
  class MouseEvent;

  class EntityMaker
  {
    /// \brief Constructor
    public: EntityMaker();

    /// \brief Destructor
    public: virtual ~EntityMaker();

    /// \brief Set whether to snap to grid
    public: static void SetSnapToGrid(bool snap);

    public: virtual void Start() = 0;
    public: virtual void Stop() = 0;
    public: virtual bool IsActive() const = 0;

    public: virtual void MousePushCB(const MouseEvent &event);
    public: virtual void MouseReleaseCB(const MouseEvent &event);
    public: virtual void MouseDragCB(const MouseEvent &event);

    // \brief Get a point snapped to a grid
    protected: Vector3 GetSnappedPoint(Vector3 p);

    public: Vector3 GetWorldPointOnPlane(int x, int y, 
                                         Vector3 planeNorm, double d);

    protected: virtual void CreateTheEntity() = 0;

    private: static bool snapToGrid;
    private: static double snapDistance;
    private: static double snapGridSize;
  };
}

#endif
