#ifndef MODEL_LIST_WIDGET_HH
#define MODEL_LIST_WIDGET_HH

#include <QWidget>

#include "sdf/sdf.h"
#include "msgs/msgs.h"
#include "transport/TransportTypes.hh"
#include "rendering/RenderTypes.hh"

class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;

namespace gazebo
{
  namespace gui
  {
    class ModelListWidget : public QWidget
    {
      Q_OBJECT
      public: ModelListWidget( QWidget *parent = 0 );
      public: virtual ~ModelListWidget();

      private slots: void OnModelSelection(QTreeWidgetItem *item, int column);
      private: void OnEntities( const boost::shared_ptr<msgs::Entities const> &_msg );
      private: void OnEntity( const boost::shared_ptr<msgs::Entity const> &_msg );
      private slots: void OnMoveTo();
      private slots: void OnDelete();
      private slots: void OnCustomContextMenu(const QPoint &_pt);

      private: void ProcessEntity( const msgs::Entity &_msg );

      private: QTreeWidget *modelTreeWidget;
      private: QAction *moveToAction;
      private: QAction *deleteAction;


      private: transport::NodePtr node;
      private: transport::PublisherPtr entitiesRequestPub, entityPub;
      private: transport::SubscriberPtr entitiesSub, newEntitySub;

      private: rendering::VisualPtr modelVisual;
      private: std::list<rendering::VisualPtr> visuals;
      private: sdf::SDFPtr modelSDF;
    };
  }
}
#endif
