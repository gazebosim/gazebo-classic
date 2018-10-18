#ifndef RENDERINGPAGE_HH
#define RENDERINGPAGE_HH

#include "ParamPage.hh"

namespace gazebo
{
  class RenderingPage : public ParamPage
  {
    public: RenderingPage(wxWindow *parent);
    public: virtual ~RenderingPage();

    public: virtual void Apply();
  };
}

#endif
