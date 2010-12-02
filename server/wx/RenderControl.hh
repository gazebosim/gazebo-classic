#ifndef RENDERCONTROL_HH
#define RENDERCONTROL_HH

#include <wx/wx.h>

#include "SphereMaker.hh"
#include "BoxMaker.hh"
#include "CylinderMaker.hh"
#include "PointLightMaker.hh"
#include "SpotLightMaker.hh"
#include "DirectionalLightMaker.hh"

#include "MouseEvent.hh"

namespace gazebo
{
  class UserCamera;
  class Entity;

  class RenderControl : public wxControl
  {
    DECLARE_CLASS(RenderControl)
    DECLARE_EVENT_TABLE()

    /// \brief Constructor
    public: RenderControl(wxWindow *parent);

    /// \brief Destructor
    public: virtual ~RenderControl();

    /// \brief Get an ogre window handle
    public: std::string GetOgreHandle() const;

    public: bool Reparent(wxWindowBase* new_parent);

    public: wxSize DoGetBestSize () const;

    public: void OnMouseEvent( wxMouseEvent &event);

    public: void OnKeyUp( wxKeyEvent &event);
    public: void OnKeyDown( wxKeyEvent &event);

    public: void Init();

    /// \brief Create the camera
    public: void CreateCamera(unsigned int scene);

    /// \brief Get the camera
    public: UserCamera *GetCamera();

    /// \brief Get the cursor state
    public: std::string GetCursorState() const;

    /// \brief Set the state of the cursor
    public: void SetCursorState(const std::string &state);

    protected: virtual void OnSize( wxSizeEvent &evt );

    private: void CreateEntity(std::string name);

    private: void ManipModeCB(bool mode);
    private: void MoveModeCB(bool mode);

    private: void SetSelectedEntityCB(const std::string &name);

    /// \brief Rotate and entity, or apply torque
    private: void EntityRotate(Entity *entity);

    /// \brief Translate an entity, or apply force
    private: void EntityTranslate(Entity *entity);

    public: Vector3 GetWorldPointOnPlane(int x, int y, 
                Vector3 planeNorm, double d);

    private: UserCamera *userCamera;

    private: MouseEvent mouseEvent;

    private: EntityMaker *currMaker;
    private: std::string cursorState;
    private: std::string mouseModifier;

    private: CylinderMaker cylinderMaker;
    private: BoxMaker boxMaker;
    private: SphereMaker sphereMaker;
    private: PointLightMaker pointLightMaker;
    private: SpotLightMaker spotLightMaker;
    private: DirectionalLightMaker directionalLightMaker;
  };

}

#endif
