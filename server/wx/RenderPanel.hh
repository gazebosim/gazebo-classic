#ifndef RENDERPANEL_HH
#define RENDERPANEL_HH

#include <wx/panel.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>


namespace gazebo
{
  class RenderControl;

  class RenderPanel : public wxPanel
  {
    /// \brief Constructor
    public: RenderPanel(wxWindow *parent);

    /// \brief Destructor
    public: virtual ~RenderPanel();

    /// \brief Init
    public: void Init();

    /// \brief Update
    public: void Update();

    private: void OnXPosSetFocus(wxFocusEvent &event);
    private: void OnXPosKillFocus(wxFocusEvent &event);

    private: void OnYPosSetFocus(wxFocusEvent &event);
    private: void OnYPosKillFocus(wxFocusEvent &event);

    private: void OnZPosSetFocus(wxFocusEvent &event);
    private: void OnZPosKillFocus(wxFocusEvent &event);

    private: void OnRollSetFocus(wxFocusEvent &event);
    private: void OnRollKillFocus(wxFocusEvent &event);

    private: void OnPitchSetFocus(wxFocusEvent &event);
    private: void OnPitchKillFocus(wxFocusEvent &event);

    private: void OnYawSetFocus(wxFocusEvent &event);
    private: void OnYawKillFocus(wxFocusEvent &event);

    public: void CreateCamera(unsigned int sceneMgr);

    private: RenderControl *renderControl;
    private: wxTextCtrl *fpsCtrl;
    private: wxTextCtrl *xPosCtrl;
    private: wxTextCtrl *yPosCtrl;
    private: wxTextCtrl *zPosCtrl;
    private: wxTextCtrl *rollCtrl;
    private: wxTextCtrl *pitchCtrl;
    private: wxTextCtrl *yawCtrl;

    private: bool xUpdate;
    private: bool yUpdate;
    private: bool zUpdate;

    private: bool rollUpdate;
    private: bool pitchUpdate;
    private: bool yawUpdate;


  };
}

#endif
