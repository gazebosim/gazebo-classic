#ifndef GRIDPAGE_HH
#define GRIDPAGE_HH

#include <wx/textctrl.h>
#include <wx/clrpicker.h>

#include "ParamPage.hh"

namespace gazebo
{
  class GridPage : public ParamPage
  {
    public: GridPage(wxWindow *parent);
    public: virtual ~GridPage();

    public: virtual void Apply();

    /// \brief On colour changed
    private: void OnColorChange(wxColourPickerEvent &event);

    private: wxTextCtrl *gridSizeCtrl;
    private: wxTextCtrl *spacingCtrl;
    private: wxColourPickerCtrl *colorCtrl;
  };
}

#endif
