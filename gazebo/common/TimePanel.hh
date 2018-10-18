#ifndef TIME_PANEL_HH
#define TIME_PANEL_HH

#include <wx/panel.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include "Time.hh"
namespace gazebo
{
  class TimePanel : public wxPanel
  {
    public: TimePanel( wxWindow *parent );
    public: virtual ~TimePanel();

    public: void Update();

    private: wxStaticText *percentRealTimeText;
    private: wxTextCtrl *percentRealTimeCtrl;

    private: wxStaticText *simTimeText;
    private: wxTextCtrl *simTimeCtrl;

    private: wxStaticText *realTimeText;
    private: wxTextCtrl *realTimeCtrl;

    private: wxStaticText *pauseTimeText;
    private: wxTextCtrl *pauseTimeCtrl;

    private: Time lastUpdateTime,statusUpdatePeriod;
    private: Time percentLastRealTime, percentLastSimTime;
  };
}

#endif
