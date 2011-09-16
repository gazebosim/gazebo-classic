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
  connect( this->variantManager, 
           SIGNAL(valueChanged(QtProperty*, const QVariant &)), 
           this, SLOT(OnPropertyChanged(QtProperty *, const QVariant &)));

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
  this->entityInfoSub = this->node->Subscribe("~/entity_info", &ModelListWidget::OnEntityInfo, this);
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

void ModelListWidget::FillPropertyTree(sdf::ElementPtr &_elem,
                                       QtProperty *_parentItem)
{
  QtProperty *topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), 
      QLatin1String( _elem->GetName().c_str() ));

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
    else if ( (*iter)->IsUInt() )
    {
      item = this->variantManager->addProperty( QVariant::Int, 
          QLatin1String( (*iter)->GetKey().c_str() ));
      unsigned int value;
      (*iter)->Get(value); 
      item->setValue( value );
      item->setAttribute( "minimum", 0 );
      topItem->addSubProperty(item);
    }
    else if ( (*iter)->IsFloat() )
    {
      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( (*iter)->GetKey().c_str() ));
      float value;
      (*iter)->Get(value); 
      item->setValue( value );
      topItem->addSubProperty(item);
    }
    else if ( (*iter)->IsDouble() )
    {
      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( (*iter)->GetKey().c_str() ));
      double value;
      (*iter)->Get(value); 
      item->setValue( value );
      topItem->addSubProperty(item);
    }
    else if ( (*iter)->IsChar() )
    {
      item = this->variantManager->addProperty( QVariant::Char, 
          QLatin1String( (*iter)->GetKey().c_str() ));
      char value;
      (*iter)->Get(value); 
      item->setValue( value );
      topItem->addSubProperty(item);
    }
    else if ( (*iter)->IsVector3() )
    {
      math::Vector3 value;
      (*iter)->Get(value); 

      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( "X" ));
      item->setValue( value.x );
      topItem->addSubProperty(item);

      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( "Y" ));
      item->setValue( value.y );
      topItem->addSubProperty(item);

      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( "Z" ));
      item->setValue( value.z );
      topItem->addSubProperty(item);
    }
    else if ( (*iter)->IsQuaternion() )
    {
      math::Quaternion value;
      math::Vector3 rpy;
      (*iter)->Get(value); 
      rpy = value.GetAsEuler();

      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( "Roll" ));
      item->setValue( value.x );
      topItem->addSubProperty(item);

      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( "Pitch" ));
      item->setValue( value.y );
      topItem->addSubProperty(item);

      item = this->variantManager->addProperty( QVariant::Double, 
          QLatin1String( "Yaw" ));
      item->setValue( value.z );
      topItem->addSubProperty(item);
    }
    else if ( (*iter)->IsColor() )
    {
      common::Color value;
      (*iter)->Get(value); 

      item = this->variantManager->addProperty( QVariant::Color, 
          QLatin1String( (*iter)->GetKey().c_str() ));
      topItem->addSubProperty(item);
    }
  }

  if (_parentItem)
    _parentItem->addSubProperty(topItem);
  else
  this->propTreeBrowser->addProperty(topItem);


  for (sdf::ElementPtr_V::iterator iter = _elem->elements.begin();
       iter != _elem->elements.end(); iter++)
  {
    this->FillPropertyTree((*iter), topItem);
  }
  
}

void ModelListWidget::OnModelSelection(QTreeWidgetItem *_item, int /*_column*/)
{
  if (_item)
  {
    std::string modelName = _item->text(0).toStdString();

    msgs::Entity msg;
    msgs::Init(msg, modelName);
    msg.set_name( modelName );
    msg.set_request_info( true );

    QTimer::singleShot(200,this,SLOT(Update()));
    this->entityPub->Publish(msg);
  }
}

void ModelListWidget::Update()
{
  if (this->sdfElement)
  {
    this->propTreeBrowser->clear();
    this->FillPropertyTree(this->sdfElement, NULL);
  }
  else
  {
    QTimer::singleShot(200,this,SLOT(Update));
  }
}

void ModelListWidget::OnEntityInfo( const boost::shared_ptr<msgs::Factory const> &_msg )
{
  printf("OnEntityInfo\n");
  this->sdfElement.reset(new sdf::Element);
  sdf::initFile(_msg->sdf_description_filename(), this->sdfElement);
  sdf::readString( _msg->sdf(), this->sdfElement );
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
      name.c_str(), Qt::MatchExactly | Qt::MatchRecursive);

  if (list.size() == 0)
  {
    if (!_msg.has_request_delete() || !_msg.request_delete())
    {
      // Create a top-level tree item for the path
      QTreeWidgetItem *topItem = new QTreeWidgetItem( (QTreeWidgetItem*)0, 
          QStringList(QString("%1").arg( QString::fromStdString(name)) ));

      topItem->setData(0, Qt::UserRole, QVariant(name.c_str()));
      this->modelTreeWidget->addTopLevelItem(topItem);

      for (int i=0; i < _msg.link_names_size(); i++)
      {
        std::string linkName = _msg.link_names(i);
        int index = linkName.rfind("::") + 2;
        std::string linkNameShort = linkName.substr(
            index, linkName.size() - index);

        QTreeWidgetItem *linkItem = new QTreeWidgetItem( topItem, 
          QStringList(QString("%1").arg( 
              QString::fromStdString( linkNameShort )) ));

        linkItem->setData(0, Qt::UserRole, QVariant( linkName.c_str() ) );
        this->modelTreeWidget->addTopLevelItem( linkItem );
      }
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

void ModelListWidget::OnPropertyChanged(QtProperty *_item, const QVariant &_val)
{
  std::string name = _item->propertyName().toStdString();
  std::string value = _item->valueText().toStdString();

  // TODO: convert the property browser back to SDF, and transmitt over the
  // wire.
}


