#ifndef FOGPAGE_HH
#define FOGPAGE_HH

#include <wx/valtext.h>
#include <wx/combobox.h>
#include <wx/clrpicker.h>

#include "ParamPage.hh"

namespace gazebo
{
  class FogPage : public ParamPage
  {
    /// \brief Constructor
    public: FogPage(wxWindow *parent);

    /// \brief Destructor
    public: virtual ~FogPage();

    /// \brief Apply the current parameters
    public: virtual void Apply();

    private: wxComboBox *typeBox;
    private: wxTextCtrl *startCtrl;
    private: wxTextCtrl *endCtrl;
    private: wxTextCtrl *densityCtrl;
    private: wxColourPickerCtrl *colorCtrl;
  };
}

#endif
