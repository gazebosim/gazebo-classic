#ifndef TIME_PANEL_HH
#define TIME_PANEL_HH

#include <QWidget>
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"

class QLineEdit;

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

      private: void OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &msg);

      private: QLineEdit *percentRealTimeEdit;
      private: QLineEdit *simTimeEdit;
      private: QLineEdit *realTimeEdit;
      private: QLineEdit *pauseTimeEdit;

      private: common::Time lastUpdateTime,statusUpdatePeriod;
      private: common::Time simTime, realTime, pauseTime;
      private: transport::SubscriberPtr statsSub;

      private: transport::NodePtr node;
    };
  }
}

#endif
