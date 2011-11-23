#include <QtGui>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/recursive_mutex.hpp>

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

#include "math/Angle.hh"
#include "math/Helpers.hh"

#include "gui/qtpropertybrowser/qttreepropertybrowser.h"
#include "gui/qtpropertybrowser/qtvariantproperty.h"
#include "gui/ModelListWidget.hh"

using namespace gazebo;
using namespace gui;

const unsigned short dgrsUnicode = 0x00b0;
const std::string dgrsStdStr = QString::fromUtf16(&dgrsUnicode, 1).toStdString();
const std::string rollLbl = std::string("Roll") + dgrsStdStr;
const std::string pitchLbl = std::string("Pitch") + dgrsStdStr;
const std::string yawLbl = std::string("Yaw") + dgrsStdStr;
const std::string xLbl = std::string("X");
const std::string yLbl = std::string("Y");
const std::string zLbl = std::string("Z");


ModelListWidget::ModelListWidget( QWidget *parent )
  : QWidget( parent )
{
  this->propMutex = new boost::recursive_mutex();
  this->fillPropertyTree = false;

  setMinimumWidth(280);
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
  this->variantFactory = new QtVariantEditorFactory();
  this->propTreeBrowser->setFactoryForManager( this->variantManager, 
                                               this->variantFactory);
  connect( this->variantManager, 
           SIGNAL(propertyChanged(QtProperty*)), 
           this, SLOT(OnPropertyChanged(QtProperty *)));
  connect( this->propTreeBrowser, 
           SIGNAL(currentItemChanged(QtBrowserItem*)), 
           this, SLOT(OnCurrentPropertyChanged(QtBrowserItem *)));


  mainLayout->addWidget(this->modelTreeWidget,0);
  mainLayout->addWidget(this->propTreeBrowser,1);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(2,2,2,2);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->requestPub = this->node->Advertise<msgs::Request>("~/request");
  this->responseSub = this->node->Subscribe("~/response", 
                                            &ModelListWidget::OnResponse, this);

  this->requestSub = this->node->Subscribe("~/request",
      &ModelListWidget::OnRequest, this);

  this->newEntitySub = this->node->Subscribe("~/model/info", 
      &ModelListWidget::OnModel, this);

  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
  this->selectionPub = this->node->Advertise<msgs::Selection>("~/selection");

  this->followAction = new QAction(tr("Follow"), this);
  this->followAction->setStatusTip(tr("Follow the selection"));
  connect(this->followAction, SIGNAL(triggered()), this, SLOT(OnFollow()));

  this->moveToAction = new QAction(tr("Move To"), this);
  this->moveToAction->setStatusTip(tr("Move camera to the selection"));
  connect(this->moveToAction, SIGNAL(triggered()), this, SLOT(OnMoveTo()));

  this->deleteAction = new QAction(tr("Delete"), this);
  this->deleteAction->setStatusTip(tr("Delete the selection"));
  connect(this->deleteAction, SIGNAL(triggered()), this, SLOT(OnDelete()));

  this->showCollisionAction = new QAction(tr("Show Collision"), this);
  this->showCollisionAction->setStatusTip(tr("Show Collision Entity"));
  this->showCollisionAction->setCheckable(true);
  connect(this->showCollisionAction, SIGNAL(triggered()), this, 
          SLOT(OnShowCollision()));


  this->fillingPropertyTree = false;
  this->selectedProperty = NULL;

  this->requestMsg = msgs::CreateRequest("entity_list");
  this->requestPub->Publish(*this->requestMsg);

  QTimer::singleShot( 500, this, SLOT(Update()) );
}

ModelListWidget::~ModelListWidget()
{
  delete this->propMutex;
}


void ModelListWidget::OnModelSelection(QTreeWidgetItem *_item, int /*_column*/)
{
  if (_item)
  {
    msgs::Selection msg;

    if (!this->selectedModelName.empty())
    {
      msg.set_name( this->selectedModelName );
      msg.set_selected( false );
      this->selectionPub->Publish(msg);
    }

    this->propTreeBrowser->clear();
    this->selectedModelName = 
      _item->data(0, Qt::UserRole).toString().toStdString();
    msg.set_name( this->selectedModelName );
    msg.set_selected( true );
    this->selectionPub->Publish(msg);

    this->requestMsg = msgs::CreateRequest("entity_info",
                                           this->selectedModelName );
    this->requestPub->Publish( *this->requestMsg );
  }
  else
    this->selectedModelName.clear();
}

void ModelListWidget::Update()
{
  if (this->fillPropertyTree)
  {
    this->FillPropertyTree(this->modelMsg,NULL);
    this->fillPropertyTree = NULL;
  }

  QTimer::singleShot( 500, this, SLOT(Update()) );

  /*if (!this->selectedModelName.empty())
  {
    this->requestMsg = msgs::CreateRequest("entity_info",
                                           this->selectedModelName );
    this->requestPub->Publish( *this->requestMsg );
  }

  if (this->sdfElement)
  {
    this->propMutex->lock();
    this->fillingPropertyTree = true;

    for (sdf::ElementPtr_V::iterator iter = this->sdfElement->elements.begin();
         iter != this->sdfElement->elements.end(); iter++)
    {
      this->FillPropertyTree((*iter), NULL);
    }
    this->fillingPropertyTree = false;
    this->propMutex->unlock();
  }
  else
  {
  }
  */
}

void ModelListWidget::OnModel(const boost::shared_ptr<msgs::Model const> &_msg )
{
  this->AddModelToList(*_msg);
}

void ModelListWidget::AddModelToList( const msgs::Model &_msg )
{
  std::string name = _msg.name();

  QTreeWidgetItem *listItem = this->GetModelListItem(name);

  if (!listItem)
  {
    if (!_msg.has_deleted() || !_msg.deleted())
    {
      // Create a top-level tree item for the path
      QTreeWidgetItem *topItem = new QTreeWidgetItem( (QTreeWidgetItem*)0, 
          QStringList(QString("%1").arg( QString::fromStdString(name)) ));

      topItem->setData(0, Qt::UserRole, QVariant(name.c_str()));
      this->modelTreeWidget->addTopLevelItem(topItem);

      for (int i=0; i < _msg.links_size(); i++)
      {
        std::string linkName = _msg.links(i).name();
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
    if (_msg.has_deleted() && _msg.deleted())
    {
      int i = this->modelTreeWidget->indexOfTopLevelItem(listItem);
      this->modelTreeWidget->takeTopLevelItem(i);
    }
  }
}

void ModelListWidget::OnResponse(
    const boost::shared_ptr<msgs::Response const> &_msg)
{
  if (!this->requestMsg || _msg->id() != this->requestMsg->id())
    return;

  msgs::Model_V modelVMsg;
  msgs::Model modelMsg;
  msgs::String stringMsg;

  if (_msg->has_type() && _msg->type() == modelVMsg.GetTypeName())
  {
    modelVMsg.ParseFromString( _msg->serialized_data() );

    for (int i=0; i < modelVMsg.models_size(); i++)
    {
      this->AddModelToList( modelVMsg.models(i) );
    }
  }
  else if (_msg->has_type() && _msg->type() == this->modelMsg.GetTypeName())
  {
    this->propMutex->lock();
    this->modelMsg.ParseFromString(_msg->serialized_data());
    this->propTreeBrowser->clear();
    this->fillPropertyTree = true;
    this->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() == stringMsg.GetTypeName())
  {
  }
  else if (_msg->has_type() && _msg->type() == "error")
  {
    if (_msg->response() == "nonexistant")
    {
      this->RemoveEntity(this->selectedModelName);
    }
  }

  delete this->requestMsg;
  this->requestMsg = NULL;
}

void ModelListWidget::RemoveEntity(const std::string &_name)
{
  QTreeWidgetItem *listItem = this->GetModelListItem( _name );
  if (listItem)
  {
    int i = this->modelTreeWidget->indexOfTopLevelItem(listItem);
    this->modelTreeWidget->takeTopLevelItem(i);

    this->propTreeBrowser->clear();
    this->selectedModelName.clear();
    this->sdfElement.reset();
  }
}

QTreeWidgetItem *ModelListWidget::GetModelListItem( const std::string &_name )
{
  QTreeWidgetItem *listItem = NULL;

  // Find an existing element with the name from the message
  for (int i=0; i < this->modelTreeWidget->topLevelItemCount() && !listItem;i++)
  {
    QTreeWidgetItem *item = this->modelTreeWidget->topLevelItem(i);
    std::string data = item->data(0, Qt::UserRole).toString().toStdString();
    if (data == _name)
    {
      listItem = item;
      break;
    }

    for (int j=0; j < item->childCount(); j++)
    {
      QTreeWidgetItem *childItem = item->child(j);
      data = childItem->data(0, Qt::UserRole).toString().toStdString();
      if (data == _name)
      {
        listItem = childItem;
        break;
      }
    }
  }

  return listItem;
}

void ModelListWidget::OnShowCollision()
{
  QTreeWidgetItem *item = this->modelTreeWidget->currentItem();
  std::string modelName = item->text(0).toStdString();

  if (this->showCollisionAction->isChecked())
    this->requestMsg = msgs::CreateRequest( "show_collision", modelName );
  else
    this->requestMsg = msgs::CreateRequest( "hide_collision", modelName );

  this->requestPub->Publish(*this->requestMsg);
}

void ModelListWidget::OnDelete()
{
  QTreeWidgetItem *item = this->modelTreeWidget->currentItem();
  std::string modelName = item->text(0).toStdString();

  this->requestMsg = msgs::CreateRequest( "entity_delete", modelName );
  this->requestPub->Publish(*this->requestMsg);
}

void ModelListWidget::OnFollow()
{
  QTreeWidgetItem *item = this->modelTreeWidget->currentItem();
  std::string modelName = item->text(0).toStdString();

  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->TrackVisual(modelName);
}

void ModelListWidget::OnMoveTo()
{
  QTreeWidgetItem *item = this->modelTreeWidget->currentItem();
  std::string modelName = item->text(0).toStdString();

  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->MoveToVisual(modelName);
}

void ModelListWidget::OnCustomContextMenu(const QPoint &_pt)
{
  QTreeWidgetItem *item = this->modelTreeWidget->itemAt(_pt);

  if (item)
  {
    QMenu menu(this->modelTreeWidget);
    menu.addAction(this->moveToAction);
    menu.addAction(this->followAction);
    menu.addAction(this->deleteAction);
    menu.addAction(this->showCollisionAction);
    menu.exec(this->modelTreeWidget->mapToGlobal(_pt));
  }
}

void ModelListWidget::OnCurrentPropertyChanged(QtBrowserItem * /*_item*/)
{
  //this->selectedProperty = _item->property();
}

void ModelListWidget::OnPropertyChanged(QtProperty * /*_item*/)
{
  /*this->propMutex->lock();

  if (this->fillingPropertyTree)
  {
    this->propMutex->unlock();
    return;
  }

  msgs::Factory msg;
  msg.set_edit_name( this->sdfElement->GetValueString("name") );

  QList<QtProperty*> properties = this->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin(); 
       iter != properties.end(); iter++)
  {
    sdf::ElementPtr elem = this->sdfElement->GetElement(
        (*iter)->propertyName().toStdString() );
    if (elem)
    {
      std::cout << "PropName[" << (*iter)->propertyName().toStdString() 
                << "] ElementName[" << elem->GetName() << "]\n";
      this->FillSDF( (*iter), elem, _item);
    }
  }

  msg.set_sdf( this->sdfElement->ToString("") );

  std::string str =  this->sdfElement->ToString("");
  std::cout << "ToString[" << str << "]\n";
  this->factoryPub->Publish(msg);

  this->propMutex->unlock();
  */
}

void ModelListWidget::FillSDF(QtProperty *_item, sdf::ElementPtr &_elem, QtProperty *_changedItem)
{
  if (!_item)
    return;

  // Set all the attribute values
  for (sdf::Param_V::iterator iter = _elem->attributes.begin(); 
      iter != _elem->attributes.end(); iter++)
  {
    if ( (*iter)->IsPose())
    {
      math::Pose pose;
      math::Vector3 rpy;
      (*iter)->Get(pose);
      rpy = pose.rot.GetAsEuler();

      bool changed = false;
      QList<QtProperty*> subProperties = _item->subProperties();
      for (QList<QtProperty*>::iterator propIter = subProperties.begin();
          propIter != subProperties.end(); propIter++)
      {
        if ( (*propIter) != _changedItem)
          continue;

        changed = true;

        if ( (*propIter)->propertyName().toStdString() == xLbl)
          pose.pos.x = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName().toStdString() == yLbl)
          pose.pos.y = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName().toStdString() == zLbl)
          pose.pos.z = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName().toStdString() == rollLbl)
          rpy.x = DTOR( boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString()) );
        else if ((*propIter)->propertyName().toStdString() == pitchLbl)
          rpy.y = DTOR( boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString()) );
        else if ( (*propIter)->propertyName().toStdString() == yawLbl)
          rpy.z = DTOR( boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString()) );
      }

      if (changed)
      {
        pose.rot.SetFromEuler( rpy );
        (*iter)->Set( pose );
        (*iter)->Get(pose);
      }
    }
    else if ( (*iter)->IsVector3() )
    {
      math::Vector3 xyz;
      (*iter)->Get(xyz);

      bool changed = false;

      QList<QtProperty*> subProperties = _item->subProperties();
      for (QList<QtProperty*>::iterator propIter = subProperties.begin();
          propIter != subProperties.end(); propIter++)
      {
        if ( (*propIter) != _changedItem)
          continue;
        changed = true;

        if ( (*propIter)->propertyName().toStdString() == xLbl)
          xyz.x = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName().toStdString() == yLbl)
          xyz.y = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
        else if ( (*propIter)->propertyName().toStdString() == zLbl)
          xyz.z = boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString());
      }

      if (changed)
      {
        (*iter)->Set(xyz);
      }
    }
    else if ( (*iter)->IsQuaternion())
    {
      math::Quaternion q;
      (*iter)->Get(q);
      math::Vector3 rpy;
      rpy = q.GetAsEuler();

      bool changed = false;

      QList<QtProperty*> subProperties = _item->subProperties();
      for (QList<QtProperty*>::iterator propIter = subProperties.begin();
          propIter != subProperties.end(); propIter++)
      {
        if ( (*propIter) != _changedItem)
          continue;
        changed = true;

        if ( (*propIter)->propertyName().toStdString() == rollLbl)
          rpy.x = DTOR( boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString()) );
        else if ((*propIter)->propertyName().toStdString() == pitchLbl)
          rpy.y = DTOR( boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString()) );
        else if ((*propIter)->propertyName().toStdString() == yawLbl)
          rpy.z = DTOR( boost::lexical_cast<double>(
              (*propIter)->valueText().toStdString()) );
      }

      if (changed)
      {
        q.SetFromEuler( rpy );
        (*iter)->Set(q);
      }
    }
    else if ((*iter)->IsColor())
    {
      QtProperty *childItem = this->GetChildItem(_item, (*iter)->GetKey() );

      if (childItem && childItem == _changedItem)
      {
        std::string color = childItem->valueText().toStdString();
      }
    }
    else
    {
      QtProperty *childItem = this->GetChildItem(_item, (*iter)->GetKey() );

      if (childItem && childItem == _changedItem)
      {
        (*iter)->SetFromString( childItem->valueText().toStdString() );
      }
    }
  }

  for (sdf::ElementPtr_V::iterator iter = _elem->elements.begin();
       iter != _elem->elements.end(); iter++)
  {
    this->FillSDF( 
        this->GetChildItem(_item, (*iter)->GetName()), (*iter), _changedItem);
  }
}

QtProperty *ModelListWidget::PopChildItem(QList<QtProperty*> &_list,
    const std::string &_name)
{
  for (QList<QtProperty*>::iterator iter = _list.begin(); 
      iter != _list.end(); iter++)
  {
    if ( (*iter)->propertyName().toStdString() == _name )
    {
      _list.erase(iter);
      return (*iter);
    }
  }

  return NULL;
}

QtProperty *ModelListWidget::GetChildItem(QtProperty *_item, const std::string &_name)
{
  if (!_item)
    return NULL;

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



void ModelListWidget::FillPropertyTree(const msgs::Link &_msg,
                                       QtProperty *_parent)
{
  QtProperty *topItem = NULL;
  QtProperty *inertialItem = NULL;
  QtVariantProperty *item = NULL;

  item = this->variantManager->addProperty(QVariant::String,
                                           QLatin1String("name"));
  item->setValue(_msg.name().c_str());
  _parent->addSubProperty(item);


  // Self-collide
  item = this->variantManager->addProperty(QVariant::Bool,
                                           QLatin1String("self collide"));
  if (_msg.has_self_collide())
    item->setValue(_msg.self_collide());
  else
    item->setValue(true);
  _parent->addSubProperty(item);

  // gravity
  item = this->variantManager->addProperty(QVariant::Bool,
                                           QLatin1String("gravity"));
  if (_msg.has_gravity())
    item->setValue(_msg.gravity());
  else
    item->setValue(true);
  _parent->addSubProperty(item);

  // kinematic
  item = this->variantManager->addProperty(QVariant::Bool,
                                           QLatin1String("kinematic"));
  if (_msg.has_kinematic())
    item->setValue(_msg.kinematic());
  else
    item->setValue(false);
  _parent->addSubProperty(item);

  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      QLatin1String("Pose"));
  _parent->addSubProperty(topItem);
  this->FillPoseProperty(_msg.pose(), topItem);

  // Inertial
  inertialItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      QLatin1String("Inertial"));
  _parent->addSubProperty(inertialItem);

  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      QLatin1String("Pose"));
  inertialItem->addSubProperty(topItem);
  this->FillPoseProperty(_msg.inertial().pose(), topItem);

  // Inertial::Mass
  item = this->variantManager->addProperty(QVariant::Double,
                                           QLatin1String("mass"));
  if (_msg.inertial().has_mass())
    item->setValue(_msg.inertial().mass());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::LinearDamping
  item = this->variantManager->addProperty(QVariant::Double,
                                           QLatin1String("linear damping"));
  if (_msg.inertial().has_linear_damping())
    item->setValue(_msg.inertial().linear_damping());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::AngularDamping
  item = this->variantManager->addProperty(QVariant::Double,
                                           QLatin1String("angular damping"));
  if (_msg.inertial().has_angular_damping())
    item->setValue(_msg.inertial().angular_damping());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::ixx
  item = this->variantManager->addProperty(QVariant::Double,
                                           QLatin1String("ixx"));
  if (_msg.inertial().has_ixx())
    item->setValue(_msg.inertial().ixx());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::ixy
  item = this->variantManager->addProperty(QVariant::Double,
                                           QLatin1String("ixy"));
  if (_msg.inertial().has_ixy())
    item->setValue(_msg.inertial().ixy());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::ixz
  item = this->variantManager->addProperty(QVariant::Double,
                                           QLatin1String("ixz"));
  if (_msg.inertial().has_ixz())
    item->setValue(_msg.inertial().ixz());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::iyy
  item = this->variantManager->addProperty(QVariant::Double,
                                           QLatin1String("iyy"));
  if (_msg.inertial().has_iyy())
    item->setValue(_msg.inertial().iyy());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::iyz
  item = this->variantManager->addProperty(QVariant::Double,
                                           QLatin1String("iyz"));
  if (_msg.inertial().has_iyz())
    item->setValue(_msg.inertial().iyz());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::izz
  item = this->variantManager->addProperty(QVariant::Double,
                                           QLatin1String("izz"));
  if (_msg.inertial().has_izz())
    item->setValue(_msg.inertial().izz());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  printf("Visuals[%d]\n",_msg.visuals_size());
  for (int i=0; i < _msg.visuals_size(); i++)
  {
    topItem = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        QLatin1String("visual"));
    _parent->addSubProperty(topItem);
 
    //this->FillPropertyTree(_msg.visuals(i),topItem);
  }

  for (int i=0; i < _msg.collisions_size(); i++)
  {
    topItem = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        QLatin1String("collision"));
    _parent->addSubProperty(topItem);
 
    //this->FillPropertyTree(_msg.collisions(i),topItem);
  }

  for (int i=0; i < _msg.sensors_size(); i++)
  {
    topItem = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        QLatin1String("sensor"));
    _parent->addSubProperty(topItem);
 
    //this->FillPropertyTree(_msg.sensors(i),topItem);
  }

}

void ModelListWidget::FillPropertyTree(const msgs::Model &_msg,
                                       QtProperty *_parent)
{
  QtProperty *topItem = NULL;
  QtVariantProperty *item = NULL;

  item = this->variantManager->addProperty(QVariant::String,
                                           QLatin1String("name"));
  item->setValue(_msg.name().c_str());
  this->propTreeBrowser->addProperty(item);


  item = this->variantManager->addProperty(QVariant::Bool,
                                           QLatin1String("static"));
  if (_msg.has_is_static())
    item->setValue(_msg.is_static());
  else
    item->setValue(false);
  this->propTreeBrowser->addProperty(item);

  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      QLatin1String("Pose"));
  this->propTreeBrowser->addProperty(topItem);
  this->FillPoseProperty(_msg.pose(), topItem);

  for (int i=0; i < _msg.links_size(); i++)
  {
    topItem = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        QLatin1String("link"));
    this->propTreeBrowser->addProperty(topItem);
 
    this->FillPropertyTree(_msg.links(i),topItem);
  }

  /*if (!_parentItem)
  {
    QList<QtProperty*> props = this->propTreeBrowser->properties();
    for (QList<QtProperty*>::iterator iter = props.begin(); 
         iter != props.end(); iter++)
    {
      if ( (*iter)->propertyName().toStdString() == _elem->GetName())
      {
        topItem = (*iter);
        break;
      }
    }

    if (!topItem)
    {
      topItem = this->variantManager->addProperty(
          QtVariantPropertyManager::groupTypeId(), 
          QLatin1String( _elem->GetName().c_str() ));
      this->propTreeBrowser->addProperty(topItem);
    }
  }
  else
  {
    if ( (topItem = this->GetChildItem(_parentItem,_elem->GetName())) == NULL)
    {
      topItem = this->variantManager->addProperty(
          QtVariantPropertyManager::groupTypeId(), 
          QLatin1String( _elem->GetName().c_str() ));
      _parentItem->addSubProperty(topItem);
    }
  }

  QList<QtProperty*> subprops = topItem->subProperties();

  for (sdf::Param_V::iterator iter = _elem->attributes.begin(); 
      iter != _elem->attributes.end(); iter++)
  {
    QtVariantProperty *item = (QtVariantProperty*)this->PopChildItem( subprops, (*iter)->GetKey() );

    //if (item == this->selectedProperty)
      //continue;

    if ((*iter)->IsStr())
    {
      if (!item)
      {
        item = this->variantManager->addProperty( QVariant::String, 
            QLatin1String( (*iter)->GetKey().c_str() ));
        topItem->addSubProperty(item);
      }
      std::string value;
      (*iter)->Get( value ); 
      item->setValue( value.c_str() );
    }
    else if ((*iter)->IsBool())
    {
      if (!item)
      {
        item = this->variantManager->addProperty( QVariant::Bool, 
            QLatin1String( (*iter)->GetKey().c_str() ));
        topItem->addSubProperty(item);
      }
      bool value;
      (*iter)->Get(value); 
      item->setValue( value );
    }
    else if ( (*iter)->IsPose() )
    {
      math::Pose value;
      (*iter)->Get(value); 
      value.Round(6);

      math::Vector3 rpy = value.rot.GetAsEuler();
      rpy.Round(6);

      if ((item = (QtVariantProperty*)this->PopChildItem(subprops,xLbl))== NULL)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QString::fromStdString(xLbl));
        topItem->addSubProperty(item);

        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      //if (item == this->selectedProperty)
        //continue;
      item->setValue( value.pos.x );

      if ((item = (QtVariantProperty*)this->PopChildItem(subprops,yLbl))== NULL)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QString::fromStdString(yLbl));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      //if (item == this->selectedProperty)
        //continue;
      item->setValue( value.pos.y );

      if ((item = (QtVariantProperty*)this->PopChildItem(subprops,zLbl))== NULL)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QString::fromStdString(zLbl));
        topItem->addSubProperty(item);

        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      //if (item == this->selectedProperty)
        //continue;
      item->setValue( value.pos.z );

      if ((item = (QtVariantProperty*)this->PopChildItem(subprops, rollLbl)) == NULL)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QString::fromStdString(rollLbl));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      //if (item == this->selectedProperty)
        //continue;
      item->setValue( RTOD(rpy.x) );

      if ((item = (QtVariantProperty*)this->PopChildItem(subprops, pitchLbl)) == NULL)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QString::fromStdString(pitchLbl));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      //if (item == this->selectedProperty)
        //continue;
      item->setValue( RTOD(rpy.y) );

      if ((item = (QtVariantProperty*)this->PopChildItem(subprops, yawLbl)) == NULL)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QString::fromStdString(yawLbl));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      //if (item == this->selectedProperty)
        //continue;
      item->setValue( RTOD(rpy.z) );
    }
    else if ( (*iter)->IsInt() )
    {
      if (!item)
      {
        item = this->variantManager->addProperty( QVariant::Int, 
            QLatin1String( (*iter)->GetKey().c_str() ));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      int value;
      (*iter)->Get(value); 
      item->setValue( value );
    }
    else if ( (*iter)->IsUInt() )
    {
      if (!item)
      {
        item = this->variantManager->addProperty( QVariant::Int, 
            QLatin1String( (*iter)->GetKey().c_str() ));
        topItem->addSubProperty(item);
      }
      unsigned int value;
      (*iter)->Get(value); 
      item->setValue( value );
      item->setAttribute( "minimum", 0 );
    }
    else if ( (*iter)->IsFloat() )
    {
      if (!item)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QLatin1String( (*iter)->GetKey().c_str() ));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      float value;
      (*iter)->Get(value); 
      value = gazebo::math::precision(value,6);
      item->setValue( value );
    }
    else if ( (*iter)->IsDouble() )
    {
      if (!item)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QLatin1String( (*iter)->GetKey().c_str() ));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      double value;
      (*iter)->Get(value); 
      value = gazebo::math::precision(value,6);
      item->setValue( value );
    }
    else if ( (*iter)->IsChar() )
    {
      if (!item)
      {
        item = this->variantManager->addProperty( QVariant::Char, 
            QLatin1String( (*iter)->GetKey().c_str() ));
        topItem->addSubProperty(item);
      }
      char value;
      (*iter)->Get(value); 
      item->setValue( value );
    }
    else if ( (*iter)->IsVector3() )
    {
      math::Vector3 value;
      (*iter)->Get(value); 

      if ((item = (QtVariantProperty*)this->PopChildItem(subprops,xLbl))== NULL)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QString::fromStdString(xLbl));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      //if (item == this->selectedProperty)
        //continue;
      item->setValue( value.x );

      if ((item = (QtVariantProperty*)this->PopChildItem(subprops,yLbl))== NULL)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QString::fromStdString(yLbl));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      //if (item == this->selectedProperty)
        //continue;
      item->setValue( value.y );


      if ((item = (QtVariantProperty*)this->PopChildItem(subprops,zLbl))== NULL)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QString::fromStdString(zLbl));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      //if (item == this->selectedProperty)
        //continue;
      item->setValue( value.z );
    }
    else if ( (*iter)->IsQuaternion() )
    {
      math::Quaternion value;
      math::Vector3 rpy;
      (*iter)->Get(value); 
      rpy = value.GetAsEuler();

      if ((item = (QtVariantProperty*)this->PopChildItem(subprops,rollLbl)) == NULL)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QString::fromStdString(rollLbl));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      //if (item == this->selectedProperty)
        //continue;
      item->setValue( RTOD(value.x) );

      if ((item = (QtVariantProperty*)this->PopChildItem(subprops,pitchLbl)) == NULL)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QString::fromStdString(pitchLbl));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      //if (item == this->selectedProperty)
        //continue;
      item->setValue( RTOD(value.y) );

      if ((item = (QtVariantProperty*)this->PopChildItem(subprops, yawLbl)) == NULL)
      {
        item = this->variantManager->addProperty( QVariant::Double, 
            QString::fromStdString(yawLbl));
        topItem->addSubProperty(item);
        ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
      }
      //if (item == this->selectedProperty)
        //continue;
      item->setValue( RTOD(value.z) );
    }
    else if ( (*iter)->IsColor() )
    {
      common::Color value;
      (*iter)->Get(value); 

      if (!item)
      {
        item = this->variantManager->addProperty( QVariant::Color, 
            QLatin1String( (*iter)->GetKey().c_str() ));
        topItem->addSubProperty(item);
      }
    }
  }

  for (sdf::ElementPtr_V::iterator iter = _elem->elements.begin();
       iter != _elem->elements.end(); iter++)
  {
    this->PopChildItem( subprops, (*iter)->GetName() ); 
    this->FillPropertyTree((*iter), topItem);
  }

   for (QList<QtProperty*>::iterator siter = subprops.begin(); 
      siter != subprops.end(); siter++)
  {
    topItem->removeSubProperty(*siter);
  }
   */
  
}

void ModelListWidget::FillPoseProperty(const msgs::Pose &_msg,
                                       QtProperty *_parent)
{
  QtVariantProperty *item;
  math::Pose value;
  value = msgs::Convert(_msg);
  value.Round(6);

  math::Vector3 rpy = value.rot.GetAsEuler();
  rpy.Round(6);

  // Add X value
  item = this->variantManager->addProperty(QVariant::Double, "X");
  if (_parent)
    _parent->addSubProperty(item);
  ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
  item->setValue(value.pos.x);

  // Add Y value
  item = this->variantManager->addProperty(QVariant::Double, "Y");
  if (_parent)
    _parent->addSubProperty(item);
  ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
  item->setValue(value.pos.y);

  // Add Z value
  item = this->variantManager->addProperty( QVariant::Double, "Z");
  if (_parent)
    _parent->addSubProperty(item);

  ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
  item->setValue(value.pos.z);

  // Add Roll value
  item = this->variantManager->addProperty( QVariant::Double, "Roll");
  if (_parent)
    _parent->addSubProperty(item);
  ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
  item->setValue(RTOD(rpy.x));

  // Add Pitch value
  item = this->variantManager->addProperty( QVariant::Double, "Pitch");
  if (_parent)
    _parent->addSubProperty(item);
  ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
  item->setValue(RTOD(rpy.y));

  // Add Yaw value
  item = this->variantManager->addProperty( QVariant::Double, "Yaw");
  if (_parent)
    _parent->addSubProperty(item);
  ((QtVariantPropertyManager*)this->variantFactory->propertyManager(item))->setAttribute(item, "decimals", 6);
  item->setValue(RTOD(rpy.z));
}

void ModelListWidget::OnRequest(
    const boost::shared_ptr<msgs::Request const> &_msg)
{
  if (_msg->request() == "entity_delete")
  {
    this->RemoveEntity(_msg->data());
  }
}
