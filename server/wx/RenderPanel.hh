/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
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
    public: void MyUpdate();

    /// \brief Get the camera
    public: UserCamera *GetCamera();

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

    public: void ViewScene(Scene *scene);

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
