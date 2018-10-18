#ifndef PARAMPAGE_HH
#define PARAMPAGE_HH

#include <wx/panel.h>

namespace gazebo
{
  class ParamPage : public wxPanel
  {
    public: ParamPage( wxWindow *parent ) : wxPanel( parent, wxID_ANY) {}
    public: virtual ~ParamPage() {}

    public: virtual void Apply() = 0;
  };
}

#endif
