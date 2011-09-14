#include <QtGui>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "sdf/sdf.h"
#include "sdf/sdf_parser.h"
#include "common/SystemPaths.hh"
#include "common/Console.hh"

#include "rendering/Rendering.hh"
#include "rendering/Scene.hh"
#include "rendering/UserCamera.hh"
#include "rendering/Visual.hh"
#include "gui/Gui.hh"

#include "transport/Node.hh"
#include "transport/Publisher.hh"

#include "gui/qtpropertybrowser/qttreepropertybrowser.h"
#include "gui/qtpropertybrowser/qtvariantproperty.h"
#include "gui/ModelListWidget.hh"

using namespace gazebo;
using namespace gui;

ModelListWidget::ModelListWidget( QWidget *parent )
  : QWidget( parent )
{
  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->modelTreeWidget = new QTreeWidget();
  this->modelTreeWidget->setColumnCount(1);
  this->modelTreeWidget->setContextMenuPolicy( Qt::CustomContextMenu );
  this->modelTreeWidget->header()->hide();
  connect(this->modelTreeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
          this, SLOT(OnModelSelection(QTreeWidgetItem *, int)) );
  connect(this->modelTreeWidget, 
      SIGNAL( customContextMenuRequested(const QPoint &)),
      this, SLOT(OnCustomContextMenu(const QPoint &)));

  this->variantManager = new QtVariantPropertyManager();
  this->propTreeBrowser = new QtTreePropertyBrowser();
  QtVariantEditorFactory *variantFactory = new QtVariantEditorFactory();
  this->propTreeBrowser->setFactoryForManager( this->variantManager, variantFactory);


  //this->propTreeBrowser->setHeaderLabel(tr("Properties"));
  //this->propTreeBrowser->setColumnCount(1);
  //this->propTreeBrowser->setContextMenuPolicy( Qt::CustomContextMenu );

  mainLayout->addWidget(this->modelTreeWidget);
  mainLayout->addWidget(this->propTreeBrowser);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(2,2,2,2);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->entitiesRequestPub = this->node->Advertise<msgs::Request>("~/request_entities");
  this->entitiesSub = this->node->Subscribe("~/entities", &ModelListWidget::OnEntities, this);
  this->entityPub = this->node->Advertise<msgs::Entity>("~/entity");
  this->newEntitySub = this->node->Subscribe("~/entity", &ModelListWidget::OnEntity, this);

  /*msgs::Request msg;
  msg.set_index( 1 );
  msg.set_request( "entities" );
  this->entitiesRequestPub->Publish( msg );
  */

  this->moveToAction = new QAction(tr("Move To"), this);
  this->moveToAction->setStatusTip(tr("Move camera to the selection"));
  connect(this->moveToAction, SIGNAL(triggered()), this, SLOT(OnMoveTo()));

  this->deleteAction = new QAction(tr("Delete"), this);
  this->deleteAction->setStatusTip(tr("Delete the selection"));
  connect(this->deleteAction, SIGNAL(triggered()), this, SLOT(OnDelete()));

}

ModelListWidget::~ModelListWidget()
{
}

void ModelListWidget::FillPropertyTree(sdf::ElementPtr &_elem)
{
  QtProperty *topItem = this->variantManager->addProperty(QtVariantPropertyManager::groupTypeId(), QLatin1String(_elem->GetName().c_str()));

  this->propTreeBrowser->addProperty(topItem);

  for (sdf::Param_V::iterator iter = _elem->attributes.begin(); 
      iter != _elem->attributes.end(); iter++)
  {
    QtVariantProperty *item;
    if ((*iter)->IsStr())
    {
      item = this->variantManager->addProperty( QVariant::String, 
          QLatin1String( (*iter)->GetKey().c_str() ));
      std::string value;
      (*iter)->Get( value ); 
      item->setValue( value.c_str() );
      topItem->addSubProperty(item);
    }
    else if ((*iter)->IsBool())
    {
      item = this->variantManager->addProperty( QVariant::Bool, 
          QLatin1String( (*iter)->GetKey().c_str() ));
      bool value;
      (*iter)->Get(value); 
      item->setValue( value );
      topItem->addSubProperty(item);
    }
    else if ( (*iter)->IsPose() )
    {
      math::Pose value;
      (*iter)->Get(value); 
      math::Vector3 rpy = value.rot.GetAsEuler();

      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( "X" ));
      item->setValue( value.pos.x );
      topItem->addSubProperty(item);

      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( "Y" ));
      item->setValue( value.pos.y );
      topItem->addSubProperty(item);

      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( "Z" ));
      item->setValue( value.pos.z );
      topItem->addSubProperty(item);

      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( "Roll" ));
      item->setValue( rpy.x );
      topItem->addSubProperty(item);

      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( "Pitch" ));
      item->setValue( rpy.y );
      topItem->addSubProperty(item);

      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( "Yaw" ));
      item->setValue( rpy.z );
      topItem->addSubProperty(item);
    }
    else if ( (*iter)->IsInt() )
    {
      item = this->variantManager->addProperty( QVariant::Int, 
          QLatin1String( (*iter)->GetKey().c_str() ));
      int value;
      (*iter)->Get(value); 
      item->setValue( value );
      topItem->addSubProperty(item);
    }
    else if ( (*iter)->IsInt() )
    {
      item = this->variantManager->addProperty( QVariant::Int, 
          QLatin1String( (*iter)->GetKey().c_str() ));
      int value;
      (*iter)->Get(value); 
      item->setValue( value );
      topItem->addSubProperty(item);
    }


    std::cout << "Param[" << (*iter)->GetKey() 
              << "] Type[" << (*iter)->GetTypeName() << "]\n";
  }

  for (sdf::ElementPtr_V::iterator iter = _elem->elementDescriptions.begin();
      iter != _elem->elementDescriptions.end(); iter++)
  {
    if ((*iter)->GetRequired() == "0" || (*iter)->GetRequired() == "1")
    {
      std::cout << "Elem[" << (*iter)->GetName() << "]\n";
      this->FillPropertyTree((*iter));
    }
  }
}

void ModelListWidget::OnModelSelection(QTreeWidgetItem *_item, int /*_column*/)
{
  if (_item)
  {
    std::string data = _item->data(0, Qt::UserRole).toString().toStdString();
    std::cout << "Data[" << data << "]\n";
    sdf::ElementPtr sdf(new sdf::Element);
    sdf::initFile( data, sdf);



    this->FillPropertyTree( sdf );
  }
}

void ModelListWidget::OnEntity( const boost::shared_ptr<msgs::Entity const> &_msg )
{
  this->ProcessEntity(*_msg);
  /*std::string name = _msg->name();

  QList<QTreeWidgetItem*> list = this->modelTreeWidget->findItems( 
      name.c_str(), Qt::MatchExactly);

  if (list.size() == 0)
  {
    if (!_msg->has_request_delete() || !_msg->request_delete())
    {
      // Create a top-level tree item for the path
      QTreeWidgetItem *topItem = new QTreeWidgetItem( (QTreeWidgetItem*)0, 
          QStringList(QString("%1").arg( QString::fromStdString(name)) ));
      this->modelTreeWidget->addTopLevelItem(topItem);
    }
  }
  else
  {
    if (_msg->has_request_delete() && _msg->request_delete())
    {
      QList<QTreeWidgetItem*>::Iterator iter;
      for (iter = list.begin(); iter != list.end(); iter++)
      {
        int i = this->modelTreeWidget->indexOfTopLevelItem(*iter);
        this->modelTreeWidget->takeTopLevelItem(i);
        delete *iter;
      }
    }
  }
  */

}

void ModelListWidget::ProcessEntity( const msgs::Entity &_msg )
{
  std::string name = _msg.name();

  QList<QTreeWidgetItem*> list = this->modelTreeWidget->findItems( 
      name.c_str(), Qt::MatchExactly);

  if (list.size() == 0)
  {
    if (!_msg.has_request_delete() || !_msg.request_delete())
    {
      // Create a top-level tree item for the path
      QTreeWidgetItem *topItem = new QTreeWidgetItem( (QTreeWidgetItem*)0, 
          QStringList(QString("%1").arg( QString::fromStdString(name)) ));
      topItem->setData(0, Qt::UserRole, QVariant("/sdf/model.sdf"));
      this->modelTreeWidget->addTopLevelItem(topItem);
    }
  }
  else
  {
    if (_msg.has_request_delete() && _msg.request_delete())
    {
      QList<QTreeWidgetItem*>::Iterator iter;
      for (iter = list.begin(); iter != list.end(); iter++)
      {
        int i = this->modelTreeWidget->indexOfTopLevelItem(*iter);
        this->modelTreeWidget->takeTopLevelItem(i);
        delete *iter;
      }
    }
  }
}

void ModelListWidget::OnEntities( const boost::shared_ptr<msgs::Entities const> &_msg )
{
  for (int i=0; i < _msg->entities_size(); i++)
  {
    this->ProcessEntity(_msg->entities(i));

    /*std::string name = _msg->entities(i).name();

    // Create a top-level tree item for the path
    QTreeWidgetItem *topItem = new QTreeWidgetItem( (QTreeWidgetItem*)0, 
        QStringList(QString("%1").arg( QString::fromStdString(name)) ));
    this->modelTreeWidget->addTopLevelItem(topItem);
    */
  }
}

void ModelListWidget::OnDelete()
{
  QTreeWidgetItem *item = this->modelTreeWidget->currentItem();
  std::string modelName = item->text(0).toStdString();

  msgs::Entity msg;
  msgs::Init(msg, modelName);
  msg.set_name( modelName );
  msg.set_request_delete( true );

  this->entityPub->Publish(msg);
}

void ModelListWidget::OnMoveTo()
{
  QTreeWidgetItem *item = this->modelTreeWidget->currentItem();
  std::string modelName = item->text(0).toStdString();

  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->MoveTo(modelName);
}

void ModelListWidget::OnCustomContextMenu(const QPoint &_pt)
{
  QTreeWidgetItem *item = this->modelTreeWidget->itemAt(_pt);

  if (item)
  {
    QMenu menu(this->modelTreeWidget);
    menu.addAction(moveToAction);
    menu.addAction(deleteAction);
    menu.exec(this->modelTreeWidget->mapToGlobal(_pt));
  }
}
