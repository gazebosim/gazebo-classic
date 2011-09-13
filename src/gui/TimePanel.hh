#ifndef TIME_PANEL_HH
#define TIME_PANEL_HH

#include <QWidget>
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Event.hh"
#include "common/Time.hh"

class QLineEdit;
class QLabel;

namespace gazebo
{
  namespace gui
  {
    class TimePanel : public QWidget
    {
      Q_OBJECT
      public: TimePanel( QWidget *parent = 0 );
      public: virtual ~TimePanel();

      private slots: void Update();

      private: void OnFullScreen(bool &_value);
      private: void OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg);

      private: QLineEdit *percentRealTimeEdit;
      private: QLineEdit *simTimeEdit;
      private: QLineEdit *realTimeEdit;
      private: QLabel *pauseLabel;

      private: common::Time lastUpdateTime,statusUpdatePeriod;
      private: common::Time simTime, realTime, pauseTime;
      private: transport::SubscriberPtr statsSub;

      private: transport::NodePtr node;

      private: std::vector<event::ConnectionPtr> connections;
    };
  }
}

#endif
