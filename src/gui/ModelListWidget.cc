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
           SIGNAL(propertyChanged(QtProperty*)), 
           this, SLOT(OnPropertyChanged(QtProperty *)));

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

  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");

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
    std::string listText = _item->text(0).toStdString();
    std::string listData = _item->data(0, Qt::UserRole ).toString().toStdString();

    msgs::Entity msg;
    msgs::Init(msg, listData);
    msg.set_name( listData );
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
    this->fillingPropertyTree = true;
    this->FillPropertyTree(this->sdfElement, NULL);
    this->fillingPropertyTree = false;
  }
  else
  {
    QTimer::singleShot(200,this,SLOT(Update));
  }
}

void ModelListWidget::OnEntityInfo( const boost::shared_ptr<msgs::Factory const> &_msg )
{
  this->sdfElement.reset(new sdf::Element);
  sdf::initFile(_msg->sdf_description_filename(), this->sdfElement);
  sdf::readString( _msg->sdf(), this->sdfElement );
}

void ModelListWidget::OnEntity( const boost::shared_ptr<msgs::Entity const> &_msg )
{
  this->ProcessEntity(*_msg);
}

void ModelListWidget::ProcessEntity( const msgs::Entity &_msg )
{
  std::string name = _msg.name();

  QTreeWidgetItem *listItem = NULL;

  // Find an existing element with the name from the message
  for (int i=0; i < this->modelTreeWidget->topLevelItemCount() && !listItem;i++)
  {
    QTreeWidgetItem *item = this->modelTreeWidget->topLevelItem(i);
    std::string data = item->data(0, Qt::UserRole).toString().toStdString();
    if (data == name)
    {
      listItem = item;
      break;
    }

    for (int j=0; j < item->childCount(); j++)
    {
      QTreeWidgetItem *childItem = item->child(j);
      data = childItem->data(0, Qt::UserRole).toString().toStdString();
      if (data == name)
      {
        listItem = childItem;
        break;
      }
    }
  }

  if (!listItem)
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
      int i = this->modelTreeWidget->indexOfTopLevelItem(listItem);
      this->modelTreeWidget->takeTopLevelItem(i);
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

void ModelListWidget::OnPropertyChanged(QtProperty *_item)
{
  if (this->fillingPropertyTree)
    return;

  msgs::Factory msg;
  msgs::Init(msg,"update");
  msg.set_edit_name( this->sdfElement->GetValueString("name") );

  QList<QtProperty*> properties = this->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin(); 
       iter != properties.end(); iter++)
  {
    if ( (*iter)->propertyName().toStdString() == this->sdfElement->GetName())
    {
      this->FillSDF( (*iter), this->sdfElement);
    }
  }

  msg.set_sdf( this->sdfElement->ToString("") );

  std::string str =  this->sdfElement->ToString("");
  std::cout << "ToString[" << str << "]\n";
  this->factoryPub->Publish(msg);
}

void ModelListWidget::FillSDF(QtProperty *_item, sdf::ElementPtr &_elem)
{
  // Set all the attribute values
  for (sdf::Param_V::iterator iter = _elem->attributes.begin(); 
      iter != _elem->attributes.end(); iter++)
  {
    if ( (*iter)->IsPose())
    {
      math::Pose pose;
      math::Vector3 rpy;

      QList<QtProperty*> subProperties = _item->subProperties();
      for (QList<QtProperty*>::iterator propIter = subProperties.begin();
          propIter != subProperties.end(); propIter++)
      {
        if ( (*propIter)->propertyName() == "X")
          pose.pos.x = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName() == "Y")
          pose.pos.y = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName() == "Z")
          pose.pos.z = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName() == "Roll")
          rpy.x = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName() == "Pitch")
          rpy.y = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName() == "Yaw")
          rpy.z = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
      }
      pose.rot.SetFromEuler( rpy );
      (*iter)->Set( pose );
    }
    else if ( (*iter)->IsVector3() )
    {
      math::Vector3 xyz;
      QList<QtProperty*> subProperties = _item->subProperties();
      for (QList<QtProperty*>::iterator propIter = subProperties.begin();
          propIter != subProperties.end(); propIter++)
      {
        if ( (*propIter)->propertyName() == "X")
          xyz.x = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName() == "Y")
          xyz.y = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName() == "Z")
          xyz.z = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
      }

      (*iter)->Set(xyz);
    }
    else if ( (*iter)->IsQuaternion())
    {
      math::Quaternion q;
      math::Vector3 rpy;
      QList<QtProperty*> subProperties = _item->subProperties();
      for (QList<QtProperty*>::iterator propIter = subProperties.begin();
          propIter != subProperties.end(); propIter++)
      {
        if ( (*propIter)->propertyName() == "Roll")
          rpy.x = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName() == "Pitch")
          rpy.y = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName() == "Yaw")
          rpy.z = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
      }
      q.SetFromEuler( rpy );
      (*iter)->Set(q);
    }
    else if ((*iter)->IsColor())
    {
      QtProperty *childItem = this->GetChildItem(_item, (*iter)->GetKey() );
      std::string color = childItem->valueText().toStdString();
      gzerr << "Setting color needs to be implemented[" << color << "]\n";
    }
    else
    {
      QtProperty *childItem = this->GetChildItem(_item, (*iter)->GetKey() );
      (*iter)->SetFromString( childItem->valueText().toStdString() );
    }
  }

  for (sdf::ElementPtr_V::iterator iter = _elem->elements.begin();
       iter != _elem->elements.end(); iter++)
  {
    this->FillSDF( this->GetChildItem( _item, (*iter)->GetName()), (*iter) );
  }
}

QtProperty *ModelListWidget::GetChildItem(QtProperty *_item, const std::string &_name)
{
  QList<QtProperty*> subProperties = _item->subProperties();
  for (QList<QtProperty*>::iterator iter = subProperties.begin(); 
      iter != subProperties.end(); iter++)
  {
    if ( (*iter)->propertyName().toStdString() == _name )
    {
      return (*iter);
    }
  }

  return NULL;
}

/*void ModelListWidget::FillSDF( QtProperty *_item, QtProperty *_parent, 
                               std::string &_sdf )
{
  std::string name = _item->propertyName().toStdString();
  std::string value = _item->valueText().toStdString();

  if (_item->hasValue())
  {
    std::cout << " " << name << "='" << value << "'";
  }
  else
  {
    std::cout << "<" << name;

    bool closed = false;
    QList<QtProperty*> subProperties = _item->subProperties();
    for (QList<QtProperty*>::iterator iter = subProperties.begin(); 
        iter != subProperties.end(); iter++)
    {
      if ( !(*iter)->hasValue() && !closed )
      {
        closed = true;
        std::cout << ">\n";
      }
      this->FillSDF(*iter, _item, _sdf);
    }

    std::cout << "</" << name << ">\n";
  }
}*/


