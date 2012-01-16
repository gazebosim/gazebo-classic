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
      public: TimePanel(QWidget *_parent = 0);
      public: virtual ~TimePanel();

      private slots: void Update();

      private: void OnFullScreen(bool &_value);
      private: void OnStats(
                   ConstWorldStatisticsPtr &_msg);

      private slots: void OnTimeReset();

      private: QLineEdit *percentRealTimeEdit;
      private: QLineEdit *simTimeEdit;
      private: QLineEdit *realTimeEdit;
      private: QLabel *pauseLabel;

      private: common::Time lastUpdateTime, statusUpdatePeriod;
      private: common::Time simTime, realTime, pauseTime;

      private: transport::NodePtr node;
      private: transport::SubscriberPtr statsSub;
      private: transport::PublisherPtr worldControlPub;

      private: std::vector<event::ConnectionPtr> connections;
    };
  }
}

#endif

