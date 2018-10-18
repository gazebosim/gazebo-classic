#ifndef PARAMSNOTEBOOK_HH
#define PARAMSNOTEBOOK_HH

#include <wx/dialog.h>


namespace gazebo
{
  class ParamsNotebook : public wxDialog
  {
    /// \brief Constructor
    public: ParamsNotebook( wxWindow *parent );

    /// \brief Destructor
    public: virtual ~ParamsNotebook();

    /// \brief On done event
    public: void OnDone(wxCommandEvent &event);

    /// \brief On apply event
    public: void OnApply(wxCommandEvent &event);

    private: wxNotebook *notebook;
  };
}
#endif
