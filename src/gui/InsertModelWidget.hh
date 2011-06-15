#ifndef INSERT_MODEL_WIDGET_HH
#define INSERT_MODEL_WIDGET_HH

#include <QWidget>

#include "transport/TransportTypes.hh"

class QTreeWidget;
class QTreeWidgetItem;

namespace gazebo
{
  namespace gui
  {
    class InsertModelWidget : public QWidget
    {
      Q_OBJECT
      public: InsertModelWidget( QWidget *parent = 0 );
      public: virtual ~InsertModelWidget();

      private slots: void OnModelSelection();

      private: QTreeWidget *fileTreeWidget;

      private: transport::NodePtr node;
      private: transport::PublisherPtr factoryPub, visualPub, selectionPub;
    };
  }
}
#endif
