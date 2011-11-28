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
class QtTreePropertyBrowser;
class QtVariantPropertyManager;
class QtProperty;
class QtTreePropertyItem;
class QtBrowserItem;
class QtVariantEditorFactory;

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
  namespace gui
  {
    class ModelEditWidget;

    class ModelListWidget : public QWidget
    {
      Q_OBJECT
      public: ModelListWidget( QWidget *parent = 0 );
      public: virtual ~ModelListWidget();

      private slots: void OnModelSelection(QTreeWidgetItem *item, int column);
      private slots: void Update();
      private slots: void OnPropertyChanged(QtProperty *_item);
      private slots: void OnMoveTo();
      private slots: void OnDelete();
      private slots: void OnFollow();
      private slots: void OnCustomContextMenu(const QPoint &_pt);
      private slots: void OnCurrentPropertyChanged(QtBrowserItem *);
      private slots:void OnShowCollision();

      private: void OnResponse(
                   const boost::shared_ptr<msgs::Response const> &_msg );

      private: void OnModel(const boost::shared_ptr<msgs::Model const> &_msg);

      private: void OnRequest(
                   const boost::shared_ptr<msgs::Request const> &_msg);

      private: void AddModelToList(const msgs::Model &_msg);

      private: void FillPropertyTree(const msgs::Model &_msg,
                                     QtProperty *_parentItem);

      private: void FillPropertyTree(const msgs::Link &_msg,
                                     QtProperty *_parentItem);

      private: void FillPropertyTree(const msgs::Collision &_msg,
                                     QtProperty *_parent);

      private: void FillMsgField(QtProperty *_item, 
                   google::protobuf::Message *_message,
                   const google::protobuf::Reflection *_reflection,
                   const google::protobuf::FieldDescriptor *_field);

      private: void FillMsg(QtProperty *_item, 
                   google::protobuf::Message *_message,
                   const google::protobuf::Descriptor *_descriptor);

      private: void FillGeometryMsg(QtProperty *_item, 
                   google::protobuf::Message *_message,
                   const google::protobuf::Descriptor *_descriptor);

      private: void FillPoseMsg(QtProperty *_item, 
                   google::protobuf::Message *_message,
                   const google::protobuf::Descriptor *_descriptor);

      private: QtProperty *PopChildItem(QList<QtProperty*> &_list,
                                        const std::string &_name);
      private: QtProperty *GetChildItem(QtProperty *_item, 
                                        const std::string &_name);

      private: void RemoveEntity(const std::string &_name);

      private: QTreeWidgetItem *GetModelListItem( const std::string &_name );

      private: void FillVector3dProperty(const msgs::Vector3d &_msg,
                                         QtProperty *_parent);

      private: void FillPoseProperty(const msgs::Pose &_msg,
                                     QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Surface &_msg,
                                       QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Visual &_msg,
                                       QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Geometry &_msg,
                                       QtProperty *_parent);

      private: QTreeWidget *modelTreeWidget;
      private: QtTreePropertyBrowser *propTreeBrowser;
      private: QAction *moveToAction;
      private: QAction *deleteAction;
      private: QAction *followAction;
      private: QAction *showCollisionAction;

      private: transport::NodePtr node;
      private: transport::PublisherPtr requestPub, modelPub;
      private: transport::PublisherPtr selectionPub, factoryPub;
      private: transport::SubscriberPtr responseSub, newEntitySub;
      private: transport::SubscriberPtr requestSub;

      private: rendering::VisualPtr modelVisual;
      private: std::list<rendering::VisualPtr> visuals;
      private: sdf::SDFPtr modelSDF;

      private: ModelEditWidget *modelEditWidget;
      private: QtVariantPropertyManager *variantManager;
      private: QtVariantEditorFactory *variantFactory;
      private: boost::recursive_mutex *propMutex;
      private: sdf::ElementPtr sdfElement;
      private: std::string selectedModelName;
      private: bool fillingPropertyTree;
      private: QtProperty *selectedProperty;

      private: msgs::Request *requestMsg;


      private: msgs::Model modelMsg;
      private: bool fillPropertyTree;
    };
  }
}
#endif
